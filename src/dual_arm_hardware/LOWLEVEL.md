# AX-12A low-level control

현재 프로젝트의 U2D2 직접 연결용 실기 제어입니다. Python API와 ROS 어댑터가
동일한 제어 코어를 사용합니다. 먼저 `./lowlevel.sh demo` 또는
`./lowlevel.sh shell --mock`로 장치 없이 확인할 수 있습니다.

실기는 `config/hardware.yaml`에 **포트·baudrate·토크 제한 비율·10개 모터의
회전 방향**을 입력한 뒤 `./lowlevel.sh shell`로 실행합니다. 초기 토크는 OFF이며,
`enable`은 현재 위치를 유지하도록 설정한 후 모터를 켭니다. `set`은 rad,
`grip`은 두 턱 안쪽 간격 mm입니다. 클라이언트 갱신이 기본 1초 동안 끊기거나
통신/보호값 오류가 발생하면 전체 OFF를 시도하고 오류를 유지합니다.
OFF 미확인은 별도로 보고하며, 토크를 끄면 팔의 지지력도 사라집니다.

Linux Python implementation for the confirmed U2D2 / USB-TTL direct connection,
DYNAMIXEL Protocol 1.0. No ROS is required for the API or terminal shell.

The ten active joints use the current URDF limits and motor zero 180 degrees:

| Role | Right ID | Left ID |
|---|---:|---:|
| Shoulder pitch | 5 | 10 |
| Shoulder roll | 4 | 9 |
| Elbow pitch | 3 | 8 |
| Wrist roll | 2 | 7 |
| Centered gripper | 1 | 6 |

## Setup

From the repository root:

```bash
python3 -m venv .venv-hardware
.venv-hardware/bin/python -m pip install -r src/dual_arm_hardware/requirements-lowlevel.txt
```

Edit `config/hardware.yaml` with the actual serial port (prefer
`/dev/serial/by-id/...`), baudrate, torque output percentage, and each motor's
direction. These fields deliberately remain unset until measured. The transport
is confirmed, but the port, baudrate and calibration are not inferred from it.
The gripper IDs are right 1 / left 6. Do not run this shell and another bus
controller at the same time; the driver takes exclusive ownership of the port.

`./hardware.sh plan` validates the configuration without opening a device.
`./hardware.sh check` performs a read-only preflight. `./hardware.sh apply`
applies the configured protection limits without enabling torque.

You can inspect model numbers, raw positions and existing limits before filling
direction/torque calibration, using an explicit port and baudrate:

```bash
./lowlevel.sh probe --port /dev/serial/by-id/ACTUAL_ADAPTER --baudrate 1000000 --ids 1 2 3 4 5 6 7 8 9 10
```

The baudrate above is an example; supply the actual bus setting. `probe` only
reads registers and never changes torque, goals, speed or EEPROM. It reports
unresponsive IDs individually. It does not scan unspecified ports or baudrates.

## Offline demonstration

```bash
./lowlevel.sh demo
./lowlevel.sh shell --mock
.venv-hardware/bin/python src/dual_arm_hardware/examples/control_arm.py
```

`demo` uses an in-memory register emulator, including an intentional client
timeout. It opens no serial device and does not simulate physical dynamics.
Logs explicitly contain `mock: true`. Test values such as a 25% cap and positive
motor directions are emulator settings, not real robot calibration.

## Real terminal control

```bash
./lowlevel.sh shell --config src/dual_arm_hardware/config/hardware.yaml
```

Opening the real shell checks all motors, applies narrower protection settings,
and starts with torque OFF. It refuses already-enabled devices. Once the arm is
supported and the configuration has been checked, enter commands:

```text
status
enable
set right_wrist_roll_joint 0.05
grip right 10
hold
disable
quit
```

`set` uses radians relative to the calibrated neutral. Multiple joint/value pairs
can be supplied in the same command; other joints retain their last targets.
`grip` uses the **inner pad gap in mm**, 0–70, converted using the provided CAD
linkage equations. It remains subject to the motor's actual narrower limits.
`enable` first sets every motor's goal to its measured position, then turns torque
ON; it does not move to neutral. `hold` replaces goals with fresh measured
positions while keeping torque ON. `disable` requests torque OFF.
`estop` requests OFF and latches a fault; restart the session to recover.

The interactive shell refreshes the client lease while it is alive and waiting
for input. EOF, Ctrl-C, SIGTERM and normal exit close the session and attempt OFF
on every motor. A timeout, communication/status error, altered protection value,
or goal readback mismatch also attempts OFF and latches a fault. Failed OFF
confirmation is reported explicitly. Torque OFF releases the arm's holding
force; loss of bus/power or a killed host cannot guarantee a software stop.

Session events and requested commands are saved to `log/lowlevel/*.jsonl`.

## Python API

`scripts/lowlevel_control.py` exports `DualArmLowLevel`. A complete executable
example is in `examples/control_arm.py`; it uses the emulator by default and
requires `--real` to open the configured bus.

```python
from pathlib import Path
import sys
import time

root = Path('/home/ktj/Dual_Arm')
sys.path.insert(0, str(root / 'src/dual_arm_hardware/scripts'))
from lowlevel_control import DualArmLowLevel

arm = DualArmLowLevel.from_config(
    root / 'src/dual_arm_hardware/config/hardware.yaml',
    root / 'src/dual_arm_description/config',
    root / 'src/dual_arm_hardware/config/lowlevel.yaml')
with arm:  # Starts polling, torque OFF.
    arm.enable()
    arm.set_positions({'right_wrist_roll_joint': 0.05})
    arm.set_gripper('right', 10)
    for _ in range(20):
        arm.keepalive()
        state = arm.snapshot()
        if state['sample_valid']:
            position = state['samples']['right_wrist_roll_joint']['position_rad']
        time.sleep(0.1)
    arm.hold()
```

| Method | Behavior |
|---|---|
| `start()` | Verify/apply protection, start bus polling; no torque enable |
| `enable()` | Hold measured positions, then enable all ten motors |
| `set_positions(mapping, require_all=False)` | Atomically validate targets, Sync Write, read every goal back |
| `set_gripper(side, opening_mm)` | Convert 0–70 mm gap to the single gripper motor angle |
| `keepalive()` | Renew a still-valid client lease without a motion command |
| `hold()` | Set fresh measured positions as goals, torque remains ON |
| `snapshot()` | Cached telemetry, sample age/validity, derived passive TF, loop statistics |
| `disable()` | Request OFF; returns any unconfirmed motor IDs |
| `emergency_stop(reason)` | Request OFF and latch fault |
| `close()` | Stop worker, request OFF, close port; returns unconfirmed OFF errors |

The context manager raises if OFF cannot be confirmed. When using `close()`
directly, check its returned errors. Rejected input does not renew the lease.

## Loop and device semantics

`config/lowlevel.yaml` defaults to a 20 Hz **requested** feedback loop, 2 Hz full
protection checks, a 1 s command/keepalive timeout and a 0.5 s feedback validity
budget. A dedicated thread checks the lease independently of the client. Bus
operations are serialized; `snapshot()` exposes scan duration and overruns.
Actual timing depends on USB latency, baudrate and motor response. Python on a
desktop OS is not a hard real time controller. Blocking I/O delays stop delivery.

Each position command validates the entire batch and protection state before
transmission. It writes exactly two bytes at Goal Position(30), so Moving
Speed(32) is preserved. A successful Sync Write only confirms packet
transmission; acceptance here additionally requires every device's goal readback
to match. Acceptance is not confirmation that the joint reached its target.

Current position, velocity, load ratio, voltage, temperature, torque enable/cap
and moving status are read from the motor. Present Load is not a measured N.m
torque/current value. AX-12A uses its onboard position controller; this API does
not expose torque/current commands or copy MuJoCo PID compensation to hardware.
No host velocity interpolation/limit is added. Existing narrower angle limits
and lower output caps are preserved. Faults do not automatically re-enable motors
or restore an alarm-cleared torque cap.

## ROS adapter

`./hardware.sh run` uses the same controller, not a second serial implementation.
It publishes `/joint_states` and `/dual_arm/diagnostics` and accepts:

- `/dual_arm/joint_targets`: `sensor_msgs/msg/JointState`, all ten motor positions
  in radians, no duplicate names. Commands use receipt time for lease renewal.
- `/dual_arm/keepalive`: `std_msgs/msg/Empty`, refreshes an existing enabled lease.
- `/dual_arm/enable`: `std_srvs/srv/SetBool`.

While enabled, send a valid command or keepalive more frequently than the 1 s
timeout. A single one-shot command does not authorize indefinite torque ON.
Do not run the synthetic RViz publisher on `/joint_states` at the same time.
The eight passive jaw/rod positions are derived from measured gripper angles,
not separate encoders. No fabricated effort is published.

## Validation

```bash
source /opt/ros/humble/setup.bash  # Required for ROS adapter message tests only.
PYTHONPATH="$PWD/.venv-hardware/lib/python3.10/site-packages:$PYTHONPATH" \
  .venv-hardware/bin/python -m unittest discover -s src/dual_arm_hardware/scripts -p 'test_*.py'
```

Tests include actual official SDK/pyserial packets against a local pseudo-terminal
responder, checksum/ACK handling, two-byte Sync Write, dropped recipients,
timeout, changed limits, disconnected motors, invalid/stale inputs and OFF
failure reporting. They open no physical adapter and do not establish hardware
calibration, collision-free travel or real motor timing.

References: [AX-12A control table](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/),
[Protocol 1.0 Sync Write](https://emanual.robotis.com/docs/en/dxl/protocol1/#sync-write),
[official Python SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/api_reference/python/python_groupsyncwrite/).
