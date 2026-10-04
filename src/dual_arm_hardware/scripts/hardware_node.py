#!/usr/bin/env python3
"""Real AX-12A driver: verify protection before enabling; faults latch until restart."""
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from ax12_guard import ProtectedArm, ProtectionError, SDKBus, load_configuration
from hardware_telemetry import read_telemetry, diagnostic_records


class HardwareNode(Node):
    def __init__(self):
        super().__init__('dual_arm_hardware')
        config_path = self.declare_parameter('hardware_config', '').value
        description_config = self.declare_parameter('description_config', '').value
        # Resolve every ID, direction and percentage before opening a device.
        config, motors = load_configuration(config_path, description_config)
        self.bus = SDKBus(config['port'], config['baudrate'])
        self.arm = ProtectedArm(self.bus, motors)
        try:
            self.arm.prepare(apply=True)
        except Exception:
            self.bus.close()
            raise
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, 'dual_arm/diagnostics', 1)
        self.fault_message = ''
        self.last_rejection = ''
        self.subscription = self.create_subscription(JointState, 'dual_arm/joint_targets', self.command, 1)
        self.service = self.create_service(SetBool, 'dual_arm/enable', self.enable)
        self.timer = self.create_timer(.2, self.monitor)
        self.get_logger().info('Device limits applied and read back. Torque remains OFF; waiting for /dual_arm/enable.')

    def report_stop(self, errors):
        if errors:
            self.get_logger().fatal('Torque OFF could not be confirmed: '+'; '.join(errors))

    def enable(self, request, response):
        try:
            if request.data:
                self.arm.enable()
                response.message = 'Torque ON, holding measured positions; protection verified'
            else:
                errors = self.arm.stop()
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
            self.arm.command(msg.name, msg.position)
            self.last_rejection = ''
        except Exception as exc:
            self.last_rejection = str(exc)
            self.get_logger().error('Command rejected: '+str(exc), throttle_duration_sec=2.0)
            if self.arm.fault:
                self.fault_message = str(exc)
                self.report_stop(self.arm.stop(fault=True))
                self.publish_diagnostics()

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
        if self.arm.fault:
            # Keep reporting the latched fault; never label old values as fresh.
            self.publish_diagnostics()
            return
        try:
            self.arm.verify(enabled=self.arm.enabled)
            samples = read_telemetry(self.arm)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(samples)
            msg.position = [s['position_rad'] for s in samples.values()]
            msg.velocity = [s['velocity_rad_s'] for s in samples.values()]
            # Present Load is not measured N.m: do not fabricate effort feedback.
            self.publisher.publish(msg)
            self.publish_diagnostics(samples)
        except Exception as exc:
            self.fault_message = str(exc)
            self.get_logger().fatal('Protection fault; restart required: '+str(exc))
            self.report_stop(self.arm.stop(fault=True))
            self.publish_diagnostics()

    def close(self):
        self.report_stop(self.arm.stop())
        self.bus.close()


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
