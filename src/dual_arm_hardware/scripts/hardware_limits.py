#!/usr/bin/env python3
"""Plan/check/apply AX-12A limits. This tool never enables torque or sends a goal."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

from ax12_guard import ProtectedArm, SDKBus, load_configuration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['plan', 'check', 'apply'])
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--description-config', type=Path, required=True)
    parser.add_argument('--report-dir', type=Path, default=Path('log/hardware_limits'))
    args = parser.parse_args()
    report = {'action': args.action, 'time_utc': datetime.now(timezone.utc).isoformat(),
              'configuration': str(args.config.resolve()), 'verified_on_device': False,
              'torque_enabled_by_tool': False}
    # Establish a writable audit destination before device I/O.
    args.report_dir.mkdir(parents=True, exist_ok=True)
    filename = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_'+args.action+'.json'
    path = args.report_dir/filename
    with path.open('x') as stream:
        json.dump({**report, 'status': 'started'}, stream, indent=2)
    bus = None
    code = 0
    try:
        config, motors = load_configuration(args.config, args.description_config)
        report['requested_output_percent'] = config['torque_limit_percent']
        report['motors'] = [vars(m) for m in motors]
        if args.action == 'plan':
            report['status'] = 'offline_plan_only'
        else:
            bus = SDKBus(config['port'], config['baudrate'], record=True)
            arm = ProtectedArm(bus, motors)
            report['effective_registers'] = arm.prepare(apply=args.action=='apply')
            report['verified_on_device'] = arm.ready
            report['status'] = 'applied_and_read_back' if arm.ready else 'preflight_only_no_writes'
        print(json.dumps(report, indent=2, ensure_ascii=False))
    except Exception as exc:
        code = 2
        report['status'] = 'failed'
        report['error'] = str(exc)
        print(str(exc), file=sys.stderr)
    finally:
        if bus is not None:
            report['register_transactions'] = bus.events
            try:
                bus.close()
            except Exception as exc:
                code = 2
                report['close_error'] = str(exc)
        path.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
        print('Report:', path)
    return code


if __name__ == '__main__':
    sys.exit(main())
