#!/usr/bin/env python3
"""Bundle only the native desktop app and model; excludes machine-specific state."""
import hashlib
import json
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

ROOT = Path(__file__).resolve().parents[3]


def main():
    paths = [ROOT / 'simulation.bat', ROOT / 'WINDOWS.md']
    for directory in ('src/dual_arm_description/config', 'src/dual_arm_description/meshes',
                      'src/dual_arm_description/urdf', 'src/dual_arm_simulation/config', 'cad/centered_gripper'):
        paths.extend(p for p in (ROOT / directory).rglob('*') if p.is_file())
    paths.append(ROOT / 'src/dual_arm_simulation/requirements-desktop.txt')
    paths.append(ROOT / 'src/dual_arm_description/scripts/gripper_kinematics.py')
    for script in ('import_centered_gripper.py','generate_meshes.py','mesh_geometry.py'):
        paths.append(ROOT/'src/dual_arm_description/scripts'/script)
    for script in ('simulation_app.py', 'simulation_model.py', 'twin_bridge.py', 'launch_desktop.py',
                   'windows_bootstrap.py', 'test_simulation.py', 'test_portable.py'):
        paths.append(ROOT / 'src/dual_arm_simulation/scripts' / script)
    for script in ('ax12_guard.py','hardware_telemetry.py','lowlevel_control.py','mock_bus.py','lowlevel_cli.py'):
        paths.append(ROOT/'src/dual_arm_hardware/scripts'/script)
    for name in ('hardware.yaml','lowlevel.yaml'):
        paths.append(ROOT/'src/dual_arm_hardware/config'/name)
    output = ROOT / 'dist/Dual_Arm_Windows.zip'
    output.parent.mkdir(parents=True, exist_ok=True)
    checksums = {}
    with ZipFile(output, 'w', ZIP_DEFLATED) as archive:
        for path in sorted(paths):
            name = path.relative_to(ROOT).as_posix()
            data = path.read_bytes()
            if path.suffix == '.bat':
                data = data.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
            archive.writestr('Dual_Arm/' + name, data)
            checksums[name] = hashlib.sha256(data).hexdigest()
        archive.writestr('Dual_Arm/package_manifest.json', json.dumps(checksums, indent=2))
    with ZipFile(output) as archive:
        if archive.testzip() is not None:
            raise RuntimeError('Invalid Windows archive')
    print(f'{output} ({output.stat().st_size:,} bytes, {len(paths)} files)')


if __name__ == '__main__':
    main()
