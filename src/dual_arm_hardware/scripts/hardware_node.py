#!/usr/bin/env python3
"""Real AX-12A driver: verify protection before enabling; faults latch until restart."""
import signal
from pathlib import Path
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from std_msgs.msg import Empty

from ax12_guard import ProtectionError
from hardware_telemetry import diagnostic_records
from lowlevel_control import DualArmLowLevel


class HardwareNode(Node):
    def __init__(self):
        super().__init__('dual_arm_hardware')
        config_path = self.declare_parameter('hardware_config', '').value
        description_config = self.declare_parameter('description_config', '').value
        loop_config = self.declare_parameter('loop_config', '').value
        sys.path.insert(0,str(Path(description_config).parent/'scripts'))
        from gripper_kinematics import expand_positions
        self.expand_positions = expand_positions
        # Resolve every ID, direction and percentage before opening a device.
        self.controller = DualArmLowLevel.from_config(config_path,description_config,loop_config or None)
        self.controller.start()
        self.arm = self.controller.arm
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, 'dual_arm/diagnostics', 1)
        self.fault_message = ''
        self.last_rejection = ''
        self.subscription = self.create_subscription(JointState, 'dual_arm/joint_targets', self.command, 1)
        self.heartbeat_subscription = self.create_subscription(Empty,'dual_arm/keepalive',self.keepalive,1)
        self.service = self.create_service(SetBool, 'dual_arm/enable', self.enable)
        self.timer = self.create_timer(.2, self.monitor)
        self.get_logger().info('Device limits applied and read back. Torque remains OFF; waiting for /dual_arm/enable.')

    def report_stop(self, errors):
        if errors:
            self.get_logger().fatal('Torque OFF could not be confirmed: '+'; '.join(errors))

    def enable(self, request, response):
        try:
            if request.data:
                self.controller.enable()
                response.message = 'Torque ON, holding measured positions; protection verified'
            else:
                errors = self.controller.disable()
                self.report_stop(errors)
                if errors:
                    raise ProtectionError('Torque OFF not confirmed for every motor')
                response.message = 'Torque OFF verified'
            response.success = True
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(response.message)
            self.report_stop(self.arm.stop_errors)
            if self.arm.fault or self.arm.stop_errors:
                self.arm.fault = True
                self.fault_message = str(exc)
                self.publish_diagnostics()
        return response

    def command(self, msg):
        try:
            if len(msg.name)!=len(msg.position) or len(set(msg.name))!=len(msg.name):
                raise ProtectionError('Duplicate names or mismatched target arrays')
            self.controller.set_positions(dict(zip(msg.name,msg.position)),require_all=True)
            self.last_rejection = ''
        except Exception as exc:
            self.last_rejection = str(exc)
            self.get_logger().error('Command rejected: '+str(exc), throttle_duration_sec=2.0)
            if self.arm.fault:
                self.fault_message = str(exc)
                self.report_stop(self.arm.stop_errors)
                self.publish_diagnostics()

    def keepalive(self, _message):
        try:self.controller.keepalive()
        except ProtectionError as exc:
            self.last_rejection=str(exc)

    def publish_diagnostics(self, samples=None):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for record in diagnostic_records(self.arm, samples, self.fault_message, self.last_rejection):
            status = DiagnosticStatus()
            status.name, status.hardware_id = record['name'], record['hardware_id']
            # ROS 2 Humble maps the .msg `byte` field to a one-byte bytes object.
            status.level, status.message = bytes([record['level']]), record['message']
            status.values = [KeyValue(key=k, value=v) for k, v in record['values'].items()]
            msg.status.append(status)
        self.diagnostics.publish(msg)

    def monitor(self):
        snapshot=self.controller.snapshot()
        if snapshot['fault']:
            self.fault_message=snapshot['fault']
        if self.arm.fault:
            # Keep reporting the latched fault; never label old values as fresh.
            self.publish_diagnostics()
            return
        try:
            if not snapshot['sample_valid']:
                self.publish_diagnostics()
                return
            samples = snapshot['samples']
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            positions = self.expand_positions({name:s['position_rad'] for name,s in samples.items()})
            msg.name = list(positions)
            msg.position = list(positions.values())
            # Passive linkage states are derived from the measured motor angle.
            # Omit velocities rather than inventing passive feedback.
            # Present Load is not measured N.m: do not fabricate effort feedback.
            self.publisher.publish(msg)
            self.publish_diagnostics(samples)
        except Exception as exc:
            self.fault_message = str(exc)
            self.get_logger().fatal('Protection fault; restart required: '+str(exc))
            self.report_stop(self.controller.emergency_stop(str(exc)))
            self.publish_diagnostics()

    def close(self):
        self.report_stop(self.controller.close())


def main():
    rclpy.init()
    node = None
    try:
        node = HardwareNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
