#!/usr/bin/env python3
"""Deterministic physical target-tracking checks; no GUI or hardware access."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np
from simulation_model import build_scene, Simulation


def validate(urdf, description, config, output):
    scene, manifest = build_scene(urdf, description, config, output / 'model')
    sim = Simulation(scene, manifest)
    poses = {
        'neutral': [0] * 12,
        'shoulder_pitch_90': [90, 0, 0, 0, 0, 0] * 2,
        'shoulder_roll_90': [0, 90, 0, 0, 0, 0] * 2,
        'elbow_90': [0, 0, 90, 0, 0, 0] * 2,
        'wrist_positive': [0, 0, 0, 67.5, 0, 0] * 2,
        'wrist_negative': [0, 0, 0, -67.5, 0, 0] * 2,
        'fingers_open': [0, 0, 0, 0, 30, 30] * 2,
        'combined': [70, 45, 70, 40, 15, 15] * 2,
        'asymmetric': [60, 30, 45, -45, 10, 20, 30, 60, 70, 50, 25, 5],
        'all_upper': np.degrees(sim.upper).tolist(),
        'all_lower': np.degrees(sim.lower).tolist(),
    }
    result = {'created_utc': datetime.now(timezone.utc).isoformat(),
              'urdf_sha256': manifest['urdf_sha256'], 'controller_sha256': manifest['controller_sha256'],
              'scope': 'MuJoCo simulation, not hardware validation', 'engine': manifest['engine'],
              'cases': []}
    for name, target in poses.items():
        sim.reset()
        sim.set_targets(np.radians(target))
        peak = np.zeros(12)
        overshoot = 0.
        for _ in range(600):
            sim.step(10)
            sample = sim.snapshot()
            peak = np.maximum(peak, np.abs(sample['effort']))
            overshoot = max(overshoot, float(max(sim.lower - sample['position'])),
                            float(max(sample['position'] - sim.upper)))
        error = float(max(np.abs(np.degrees(sample['error']))))
        speed = float(max(np.abs(np.degrees(sample['velocity']))))
        case = {'name': name, 'settle_time_s': sample['time'], 'max_error_deg': error,
                'max_final_speed_deg_s': speed, 'max_limit_violation_deg': float(np.degrees(overshoot)),
                'joints': {joint: {'target_deg': target[i], 'position_deg': float(np.degrees(sample['position'][i])),
                    'effort_nm': float(sample['effort'][i]), 'peak_abs_effort_nm': float(peak[i]),
                    'cap_nm': float(sim.caps[i])} for i, joint in enumerate(sim.names)}}
        case['passed'] = bool(error < .5 and speed < 1 and np.degrees(overshoot) < .5
                              and np.all(peak <= sim.caps + 1e-10) and not sim.fault)
        result['cases'].append(case)
        print(f'{name}: error={error:.3f} deg, speed={speed:.3f} deg/s, pass={case["passed"]}', flush=True)
    result['passed'] = all(c['passed'] for c in result['cases'])
    path = output / 'control_validation.json'
    path.write_text(json.dumps(result, indent=2) + '\n')
    print(path)
    return result['passed']


def main():
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, default=root / 'log/simulation/current.urdf')
    parser.add_argument('--description-dir', type=Path, default=root / 'src/dual_arm_description')
    parser.add_argument('--config', type=Path, default=root / 'src/dual_arm_simulation/config/simulation.yaml')
    parser.add_argument('--output', type=Path, default=root / 'log/simulation/control_validation')
    args = parser.parse_args()
    return 0 if validate(args.urdf, args.description_dir, args.config, args.output) else 1


if __name__ == '__main__':
    raise SystemExit(main())
