#!/usr/bin/env python3
"""Restrict synthetic JSP targets before RViz; never a hardware control path."""
import time
import signal

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from preview_limits import PreviewLimiter, read_limits


class LimitedJointStatePublisher(Node):
    def __init__(self):
        super().__init__('limited_joint_state_publisher')
        description = self.declare_parameter('robot_description', '').value
        self.limiter = PreviewLimiter(read_limits(description))
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(JointState, 'preview/joint_targets', self.target, 10)
        self.timer = self.create_timer(1/30, self.tick)
        self.get_logger().info('Preview angle limits active; speed limiting disabled. No hardware or torque control.')

    def target(self, msg):
        try:
            clamped = self.limiter.accept(msg.name, msg.position, time.monotonic())
        except ValueError as exc:
            self.get_logger().warning(f'Rejected preview targets: {exc}', throttle_duration_sec=2.0)
            return
        if clamped:
            self.get_logger().warning('Clamped preview targets: '+', '.join(clamped), throttle_duration_sec=2.0)

    def tick(self):
        positions = self.limiter.step(time.monotonic())
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(positions)
        msg.position = list(positions.values())
        # Velocity and effort stay empty: this is an instantaneous pose preview.
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = None
    try:
        node = LimitedJointStatePublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch and its terminal can both send SIGINT during shutdown.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
