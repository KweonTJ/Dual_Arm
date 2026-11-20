"""Offline register emulator. Instant position updates, no physical dynamics."""
from ax12_guard import (MODEL,CW,CCW,MAX_TORQUE,RETURN_LEVEL,SHUTDOWN,TORQUE_ENABLE,
                        GOAL,TORQUE_LIMIT,PRESENT,REGISTERED,LOCK,SIZES,ProtectionError)


class MockBus:
    def __init__(self,motors):
        self.memory={m.id:bytearray(50) for m in motors}
        self.events=[];self.closed=False
        self.fail_read_id=None;self.fail_write_id=None;self.ignore_sync_id=None
        for motor in motors:
            for a,v in {MODEL:12,CW:0,CCW:1023,MAX_TORQUE:1023,RETURN_LEVEL:2,SHUTDOWN:36,
                        TORQUE_ENABLE:0,GOAL:motor.zero,TORQUE_LIMIT:1023,PRESENT:motor.zero,
                        REGISTERED:0,LOCK:0,42:111,43:30}.items():
                self.set_register(motor.id,a,v)

    def set_register(self,motor_id,address,value):
        size=SIZES.get(address,1)
        self.memory[motor_id][address:address+size]=value.to_bytes(size,'little')

    def read(self,motor_id,address):
        self.events.append(('read',motor_id,address))
        if motor_id==self.fail_read_id:raise ProtectionError('Injected communication loss')
        return int.from_bytes(self.memory[motor_id][address:address+SIZES[address]],'little')

    def write(self,motor_id,address,value):
        self.events.append(('write',motor_id,address,value))
        if motor_id==self.fail_write_id:raise ProtectionError('Injected write failure')
        self.set_register(motor_id,address,value)

    def read_block(self,motor_id,address,length):
        self.events.append(('read_block',motor_id,address,length))
        if motor_id==self.fail_read_id:raise ProtectionError('Injected communication loss')
        if self.memory[motor_id][TORQUE_ENABLE]:
            self.memory[motor_id][PRESENT:PRESENT+2]=self.memory[motor_id][GOAL:GOAL+2]
        return list(self.memory[motor_id][address:address+length])

    def read_registers(self,motor_id,addresses):
        return {a:self.read(motor_id,a) for a in addresses}

    def sync_write_positions(self,positions):
        self.events.append(('sync_write_positions',dict(positions)))
        for motor_id,value in positions.items():
            if motor_id!=self.ignore_sync_id:self.set_register(motor_id,GOAL,value)

    def close(self):self.closed=True
