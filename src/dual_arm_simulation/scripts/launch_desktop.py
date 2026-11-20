#!/usr/bin/env python3
"""Run the same MuJoCo/Qt app on Windows or Linux without a ROS installation."""
from pathlib import Path
import os
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[3]


def prepare_urdf(description, output):
    import xacro
    output.mkdir(parents=True, exist_ok=True)
    # Forward slashes also avoid backslash escapes inside Xacro expressions on Windows.
    document = xacro.process_file((description / 'urdf/dual_arm.urdf.xacro').as_posix())
    target = output / 'current.urdf'
    target.write_text(document.toxml(), encoding='utf-8')
    return target


def main():
    description = ROOT / 'src/dual_arm_description'
    output = ROOT / 'log/simulation'
    urdf = prepare_urdf(description, output)
    env = os.environ.copy()
    if sys.platform == 'win32':
        env['MUJOCO_GL'] = 'glfw'
        # Use the installed Windows Qt platform plugin, not inherited offscreen/X11 settings.
        env.pop('QT_QPA_PLATFORM', None)
        env.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
    else:
        env.setdefault('MUJOCO_GL', 'egl')
    command = [sys.executable, str(Path(__file__).with_name('simulation_app.py')),
               '--urdf', str(urdf), '--description-dir', str(description),
               '--config', str(ROOT / 'src/dual_arm_simulation/config/simulation.yaml'),
               '--output-dir', str(output), *sys.argv[1:]]
    return subprocess.call(command, cwd=ROOT, env=env)


if __name__ == '__main__':
    raise SystemExit(main())
