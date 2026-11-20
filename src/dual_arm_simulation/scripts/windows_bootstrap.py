#!/usr/bin/env python3
"""Install a project-local Windows runtime on first launch, then open the app."""
import hashlib
from pathlib import Path
import struct
import subprocess
import sys
import venv


ROOT = Path(__file__).resolve().parents[3]


def main():
    if sys.platform != 'win32':
        raise RuntimeError('Use launch_desktop.py on Linux; this installer is for Windows.')
    if not (3, 10) <= sys.version_info[:2] <= (3, 12) or struct.calcsize('P') != 8:
        raise RuntimeError('Install 64-bit Python 3.12, 3.11 or 3.10 from python.org.')
    environment = ROOT / '.venv-windows'
    python = environment / 'Scripts/python.exe'
    if not python.is_file():
        print('Creating the local Python environment...', flush=True)
        venv.EnvBuilder(with_pip=True).create(environment)
    requirements = ROOT / 'src/dual_arm_simulation/requirements-desktop.txt'
    fingerprint = hashlib.sha256(requirements.read_bytes()).hexdigest()
    marker = environment / 'dual_arm_requirements.sha256'
    if not marker.exists() or marker.read_text(encoding='ascii') != fingerprint:
        print('Installing MuJoCo and desktop dependencies (internet needed on first run)...', flush=True)
        subprocess.run([str(python), '-m', 'pip', 'install', '--only-binary=:all:',
                        '-r', str(requirements)], check=True, cwd=ROOT)
        marker.write_text(fingerprint, encoding='ascii')
    print('Starting the Dual Arm digital twin...', flush=True)
    return subprocess.call([str(python), str(Path(__file__).with_name('launch_desktop.py')),
                            *sys.argv[1:]], cwd=ROOT)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        print(f'Windows startup failed: {exc}', file=sys.stderr)
        raise SystemExit(1)
