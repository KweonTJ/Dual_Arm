"""Official SDK + pyserial against a pseudo-terminal Protocol 1.0 responder.

No physical serial adapter or motor is opened. Exercises real byte packets,
checksums, ACKs, broadcast Sync Write and per-device readback on Linux.
"""
import os
from pathlib import Path
import pty
import select
import threading
import unittest

from ax12_guard import SDKBus,Motor,MODEL,GOAL,CW,CCW,MAX_TORQUE,TORQUE_LIMIT,ProtectionError
from lowlevel_control import DualArmLowLevel
from mock_bus import MockBus
from lowlevel_cli import probe

ROOT=Path(__file__).resolve().parents[3]


class Protocol1Responder:
    def __init__(self,motors):
        self.master,self.slave=pty.openpty()
        self.port=os.ttyname(self.slave)
        self.bus=MockBus(motors)
        self.packets=[];self.errors=[];self.ignore_sync_id=None;self.status_error_id=None
        self.silent_id=None;self.corrupt_id=None
        self.stop=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()

    def reply(self,motor_id,data=()):
        if motor_id==self.silent_id:return
        error=32 if motor_id==self.status_error_id else 0
        body=[motor_id,len(data)+2,error,*data]
        checksum=(~sum(body))&255
        if motor_id==self.corrupt_id:checksum^=1
        os.write(self.master,bytes([255,255,*body,checksum]))

    def handle(self,packet):
        if sum(packet[2:])&255 !=255:
            self.errors.append('Bad checksum');return
        self.packets.append(list(packet))
        motor_id,instruction=packet[2],packet[4];params=packet[5:-1]
        if instruction==2:
            address,length=params
            self.reply(motor_id,self.bus.read_block(motor_id,address,length))
        elif instruction==3:
            address,*data=params
            self.bus.memory[motor_id][address:address+len(data)]=bytes(data)
            self.reply(motor_id)
        elif instruction==131 and motor_id==254:
            address,length,*data=params
            for offset in range(0,len(data),length+1):
                motor_id,*values=data[offset:offset+length+1]
                if motor_id!=self.ignore_sync_id:
                    self.bus.memory[motor_id][address:address+length]=bytes(values)
        else:self.errors.append('Unsupported packet')

    def run(self):
        buffer=bytearray()
        while not self.stop.is_set():
            ready,_,_=select.select([self.master],[],[],.01)
            if not ready:continue
            try:buffer.extend(os.read(self.master,4096))
            except OSError:continue
            while len(buffer)>=4:
                if buffer[:2]!=b'\xff\xff':del buffer[0];continue
                length=buffer[3]+4
                if len(buffer)<length:break
                packet=buffer[:length];del buffer[:length]
                try:self.handle(packet)
                except Exception as exc:self.errors.append(str(exc))

    def close(self):
        self.stop.set();self.thread.join();os.close(self.slave);os.close(self.master)


class SDKProtocolTests(unittest.TestCase):
    def setUp(self):
        self.motors=[Motor(f'{side}_{role}_joint',motor_id,1,614,512,800,255,-.5,.8)
                     for side,base in [('right',0),('left',5)]
                     for role,motor_id in zip(('gripper','wrist_roll','elbow_pitch','shoulder_roll','shoulder_pitch'),range(base+1,base+6))]
        self.responder=Protocol1Responder(self.motors)
        self.addCleanup(self.responder.close)
        self.bus=SDKBus(self.responder.port,1000000)
        self.addCleanup(self.bus.close)

    def test_real_sdk_packets_sync_position_only_and_readback(self):
        self.assertEqual(self.bus.read(1,MODEL),12)
        self.bus.write(1,GOAL,630)
        self.assertEqual(self.bus.read(1,GOAL),630)
        self.bus.sync_write_positions({1:650,6:700})
        self.assertEqual(self.bus.read(1,GOAL),650);self.assertEqual(self.bus.read(6,GOAL),700)
        sync=[p for p in self.responder.packets if p[4]==131]
        self.assertEqual(sync,[[255,255,254,10,131,30,2,1,138,2,6,188,2,3]])
        self.assertEqual(self.responder.errors,[])

    def test_register_snapshot_and_status_error(self):
        values=self.bus.read_registers(1,(MODEL,CW,CCW,MAX_TORQUE,TORQUE_LIMIT))
        self.assertEqual(values,{MODEL:12,CW:0,CCW:1023,MAX_TORQUE:1023,TORQUE_LIMIT:1023})
        self.responder.status_error_id=1
        with self.assertRaisesRegex(ProtectionError,'0x20'):self.bus.read(1,MODEL)

    def test_real_serial_timeout_and_corrupt_status_are_rejected(self):
        self.responder.silent_id=1
        with self.assertRaises(ProtectionError):self.bus.read(1,MODEL)
        self.responder.silent_id=None;self.responder.corrupt_id=1
        with self.assertRaises(ProtectionError):self.bus.read(1,MODEL)

    def test_probe_uses_only_read_packets_and_no_calibration_defaults(self):
        # Close the SDK handle and release exclusivity so probe can own this PTY.
        self.bus.close()
        import fcntl,termios
        fcntl.ioctl(self.responder.slave,termios.TIOCNXCL)
        self.responder.packets.clear()
        result=probe(self.responder.port,1000000,[1,6])
        self.assertEqual([r['model'] for r in result],[12,12])
        self.assertEqual([r['position_raw'] for r in result],[614,614])
        self.assertTrue(all(p[4]==2 for p in self.responder.packets))

    def test_actual_API_detects_dropped_sync_recipient_and_stops(self):
        controller=DualArmLowLevel(self.bus,self.motors,ROOT/'src/dual_arm_description/config',
                                  settings={'command_timeout_s':2.})
        controller.start(background=False);self.addCleanup(controller.close)
        controller.enable()
        controller.set_positions({'right_wrist_roll_joint':.1})
        self.responder.ignore_sync_id=2
        with self.assertRaisesRegex(ProtectionError,'readback differs'):
            controller.set_positions({'right_wrist_roll_joint':.2})
        self.assertTrue(controller.arm.fault)
        self.assertEqual(controller.snapshot()['torque_off_unconfirmed'],[])
        self.assertTrue(all(memory[24]==0 for memory in self.responder.bus.memory.values()))
        self.assertEqual(self.responder.errors,[])


if __name__=='__main__':unittest.main()
