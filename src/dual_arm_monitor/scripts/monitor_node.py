#!/usr/bin/env python3
"""Read-only subscriber for /dual_arm/diagnostics; demo/help need no ROS runtime."""
import argparse
from datetime import datetime, timezone
import math
import sys
import time

from monitor_view import CsvRecorder, MonitorView, demo_records, records_from_message


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be a finite positive number')
    return number


def main():
    parser = argparse.ArgumentParser(description='실기 듀얼 팔 상태 모니터 (읽기 전용)')
    parser.add_argument('--topic', default='/dual_arm/diagnostics')
    parser.add_argument('--refresh', type=positive, default=1.0, help='화면 갱신 간격, 초 (기본 1)')
    parser.add_argument('--stale-after', type=positive, default=2.0, help='수신 중단 판정, 초 (기본 2)')
    parser.add_argument('--once', action='store_true', help='첫 프레임 출력 후 종료')
    parser.add_argument('--timeout', type=positive, default=5.0, help='--once 수신 대기 제한, 초')
    parser.add_argument('--csv', nargs='?', const='auto', help='수신 프레임 CSV 기록, 경로 생략 시 log/monitor에 생성')
    parser.add_argument('--demo', action='store_true', help='장치/ROS 접속 없이 예시 화면 한 번 출력')
    args, ros_args = parser.parse_known_args()
    if ros_args and ros_args[0] != '--ros-args':
        parser.error('unknown arguments: ' + ' '.join(ros_args))
    if args.demo and args.csv:
        parser.error('--demo does not record real telemetry; omit --csv')
    view = MonitorView(args.stale_after)
    if args.demo:
        view.update(demo_records(), 0.0, 'DEMO')
        print(view.render(0.0, demo=True))
        return 0

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    recorder = None
    node = None
    try:
        if args.csv:
            path = args.csv
            if path == 'auto':
                path = 'log/monitor/' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '.csv'
            recorder = CsvRecorder(path)
            print('CSV: ' + path, file=sys.stderr)
        rclpy.init(args=ros_args)
        node = rclpy.create_node('dual_arm_monitor')

        def receive(msg):
            records = records_from_message(msg)
            if not records:
                return
            stamp = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
            view.update(records, time.monotonic(), stamp)
            if recorder:
                recorder.write(records, stamp)

        # Volatile, depth 1: do not replay a retained snapshot as a live reading.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        subscription = node.create_subscription(DiagnosticArray, args.topic, receive, qos)
        started = time.monotonic()
        next_render = started
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=min(args.refresh, .1))
            now = time.monotonic()
            if args.once:
                if view.received_at is not None or now - started >= args.timeout:
                    print(view.render(now))
                    return 0 if view.received_at is not None else 2
            elif now >= next_render:
                if sys.stdout.isatty():
                    print('\033[2J\033[H', end='')
                print(view.render(now), flush=True)
                next_render = now + args.refresh
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        if recorder:
            recorder.close()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
