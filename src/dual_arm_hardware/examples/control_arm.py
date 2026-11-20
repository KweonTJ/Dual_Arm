#!/usr/bin/env python3
"""Run offline by default; --real uses the provided hardware configuration."""
import argparse
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/dual_arm_hardware/scripts'))
from lowlevel_control import DualArmLowLevel
from lowlevel_cli import mock_controller


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--real',action='store_true')
    parser.add_argument('--config',type=Path,default=ROOT/'src/dual_arm_hardware/config/hardware.yaml')
    args=parser.parse_args()
    description=ROOT/'src/dual_arm_description/config'
    arm=(DualArmLowLevel.from_config(args.config,description,ROOT/'src/dual_arm_hardware/config/lowlevel.yaml')
         if args.real else mock_controller(description,None))
    print('REAL AX-12A' if args.real else 'MOCK: no physical device')
    try:
        arm.start();arm.enable()
        # Radians relative to the calibrated 180-degree motor neutral.
        arm.set_positions({'right_wrist_roll_joint':.05})
        # Gripper inner pad spacing, not motor angle, in millimetres.
        arm.set_gripper('right',10)
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            arm.keepalive()
            sample=arm.snapshot()
            if sample['sample_valid']:
                print({n:round(s['position_deg'],2) for n,s in sample['samples'].items()})
            time.sleep(.1)
        arm.hold()
    finally:
        errors=arm.close()
        if errors:raise RuntimeError('Torque OFF unconfirmed: '+'; '.join(errors))


if __name__=='__main__':main()
