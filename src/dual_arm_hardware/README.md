# AX-12A 실기 연결 및 구동

실기 모터 조작창: `./manual_control.sh --connect` → 현재각 확인 → **전체 토크 ON** →
슬라이더 조작. [직접 조작 방법](MANUAL_CONTROL.md)을 참고하세요. 모터 절대각을
사용하며 URDF 영점·방향 캘리브레이션과 별도로 조작합니다.

저수준 Python API와 터미널 제어창은 [LOWLEVEL.md](LOWLEVEL.md)에 정리했습니다. `./lowlevel.sh demo`는 장치 없이 동작을 검증하고, 설정 완료 후 `./lowlevel.sh shell`로 실기를 제어합니다. `./hardware.sh run`도 동일한 제어 코어를 사용합니다.

이 패키지는 **DYNAMIXEL Protocol 1.0 TTL 버스에 직접 연결된 AX-12A**를 대상으로 합니다. U2D2의 TTL 포트 같은 어댑터가 필요합니다. CM-530 등 제어 보드를 통하는 경우에는 해당 보드의 펌웨어/통신 방식이 먼저 확인되어야 하며, USB 포트 이름만으로 호환을 가정하지 않습니다.

현재 `config/hardware.yaml`의 포트는 실제 확인한 FTDI 어댑터의 `/dev/serial/by-id/...` 경로입니다(현재 `/dev/ttyUSB1`). 2026-11-21에 baudrate `1000000`에서 ID 1~10 모두 AX-12A 모델 12로 응답했고 토크 OFF를 확인했습니다. 모든 방향 `+1`과 물리 영점은 실측 보정 전입니다. ID·영점·목표각 범위는 현재 모델과 공유합니다. **`configure_protection: false`이므로 연결·apply·구동 시작 시 CW/CCW Angle Limit, Max Torque, Torque Limit, Shutdown을 쓰지 않습니다.** 모터에 저장된 값을 읽어 사용하며, 100%나 초기값으로 되돌리는 쓰기도 하지 않습니다. DYNAMIXEL Wizard 등 다른 프로그램의 연결을 해제해야 포트를 사용할 수 있습니다.

## 필요한 설정

`config/hardware.yaml`의 포트, baudrate, 관절별 회전 방향을 실제 장치와 대조합니다. ID는 사용자 사진 기준으로 입력되어 있습니다(오른팔: 어깨부터 5·4·3·2, 중앙 집게 1 / 왼팔: 어깨부터 10·9·8·7, 중앙 집게 6). ID는 중복 없는 0~253이며 254번 broadcast는 사용하지 않습니다. 방향은 URDF 각도를 늘릴 때 모터 raw 위치가 증가하면 +1, 감소하면 −1입니다. 중립은 `dual_arm_description/config/motor_calibration.yaml`의 180°를 사용합니다. 0~300°/0~1023 변환에서 가장 가까운 중립 raw 값은 614(약 180.06°)이며, 각도 경계는 안쪽으로 반올림합니다. 실제 장착 방향과 영점은 실측해 맞춰야 합니다.

현재 설정에는 `torque_limit_percent`가 필요하지 않습니다. 보호값 쓰기를 사용하는 별도 설정(`configure_protection: true`)에서만 출력 상한을 **% 단위**로 지정합니다. URDF effort나 MuJoCo N·m 상한을 자동 환산하지 않습니다. 그 모드에서는 `floor(1023 × percent / 100)`으로 계산하며 기존 모터의 더 낮은 출력 상한을 유지합니다. 설정 키가 없는 이전 형식의 파일은 기존 보호값 쓰기 동작을 유지합니다.

필요 패키지는 ROS 2 Humble, DYNAMIXEL SDK, Python pyserial과 PyYAML입니다. 배포 패키지 이름은 `ros-humble-dynamixel-sdk`, `python3-serial`, `python3-yaml`입니다. 프로젝트 전용 `.venv-hardware`에 `requirements-lowlevel.txt`의 SDK·pyserial·PyYAML·NumPy를 설치했습니다. SDK 패킷은 가상 직렬 포트로 검증했고, 실제 버스에서는 읽기 140회·쓰기 0회로 10개 모터의 설정을 확인했습니다. 기록은 `log/hardware_limits/20261120T183534_697693Z_apply.json`에 있습니다. 이후 양손목 ID 2·7의 약 2° 왕복 구동도 확인했습니다. [MOTION_TEST.md](MOTION_TEST.md)에 실측값과 재실행 방법이 있습니다. 전체 팔의 영점·방향 보정은 남아 있습니다.

## 실행

프로젝트 루트에서 실행합니다. 기존 실기 제어기와 프리뷰 발행기는 종료하고, 모터 전원이 켜져 있더라도 Torque Enable은 OFF여야 합니다. 토크 OFF 시 팔을 지지해야 합니다.

```bash
# 오프라인 설정 검증/계획. 장치를 열지 않음.
./hardware.sh plan

# 실제 10개 모터를 읽어 적용 가능 여부 확인. 쓰기 없음.
./hardware.sh check

# 현재 설정: 기존 장치 설정을 읽고 재검증. 보호값 쓰기 없음.
./hardware.sh apply

# 기존 장치 설정 확인 후 실기 상태 발행과 RViz 시작. 초기 토크는 OFF.
./hardware.sh run
```

다른 설정 파일을 쓸 때는 두 번째 인자로 전달합니다. `plan/check/apply` 실행 기록과 레지스터 읽기/쓰기 결과는 `log/hardware_limits/`에 고유 시각으로 저장합니다. `display.sh`는 기존 프리뷰 전용으로 유지합니다.

`run`은 `/dual_arm/enable` 서비스를 제공합니다. 실물 위치와 팔 지지를 확인한 뒤 명시적으로 모터를 켭니다. 활성화 직전 현재 위치를 읽어 그 위치를 목표로 설정하므로 URDF 영점으로 자동 이동하지 않습니다.

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 service call /dual_arm/enable std_srvs/srv/SetBool '{data: true}'
# 끌 때:
ros2 service call /dual_arm/enable std_srvs/srv/SetBool '{data: false}'
```

토크 ON 상태에서 **기본 1초 이내**에 유효한 목표 또는 `/dual_arm/keepalive` (`std_msgs/msg/Empty`)가 계속 도착해야 합니다. 갱신이 끊기면 전체 OFF를 시도하고 오류를 유지합니다. ROS 발행 타이머만 살아 있다고 클라이언트 lease를 자동 연장하지 않습니다.

실기 목표는 `/dual_arm/joint_targets` (`sensor_msgs/msg/JointState`)에 10개 관절 이름과 위치를 모두 넣어 보냅니다. 위치 단위는 URDF 기준 rad입니다. 범위를 벗어난 목표, NaN/Inf, 이름 중복·누락은 **모든 목표를 쓰기 전에** 거부합니다. `/joint_states`에는 모터의 측정 위치와 측정 개폐각에서 계산한 수동 턱·링크 TF를 발행합니다. 수동 관절은 별도 엔코더 측정값이 아닙니다. AX-12A의 Present Load는 측정 토크가 아니므로 effort 필드는 비웁니다.

## 적용 순서와 실패 처리

1. 포트를 열기 전에 ID/방향/각도 변환을 검증합니다. 출력 상한은 보호값 쓰기 모드에서만 요구합니다.
2. 10개 모두 모델 번호 12, Torque OFF, Joint Mode, Status Return Level 2, 대기 중 REG_WRITE 없음, 현재 위치가 허용 범위 안인지 검사합니다. 활성화된 모터를 임의로 끄지 않고 적용을 거부합니다.
3. 기본 `configure_protection: false`에서는 모든 보호 레지스터를 읽기만 합니다. `true`를 명시한 설정에서만 EEPROM의 Max Torque(14), Shutdown(18), CW/CCW Angle Limit(6/8), RAM의 Torque Limit(34)을 설정하고 다시 읽습니다. 그 모드는 기존의 더 좁은 각도 범위, 더 낮은 토크 상한과 기존 Shutdown 비트를 유지하고 과열·과부하 비트(0x24)를 포함합니다.
4. 하나라도 검증이 실패하면 활성화를 허용하지 않습니다. 일부 적용 후 실패해도 더 높은 토크값으로 되돌리지 않습니다. 기록을 확인하고 다시 점검해야 합니다.
5. 실제 구동 중 보호 레지스터는 기본 2 Hz, 상태는 요청 주기 20 Hz로 확인하며, 목표를 보내기 전에도 다시 검사합니다. 재부팅/설정 변경/오류가 감지되면 토크 OFF를 시도하고 오류를 유지합니다. 자동 재활성화나 자동 과부하 복구는 하지 않습니다. 통신 불능 등으로 OFF 재확인이 안 되면 **OFF 미확인**으로 보고합니다.

모터의 Moving Speed(32)는 읽어 덮어쓰거나 0으로 설정하지 않습니다. 사용자가 요청한 대로 호스트의 이동 속도 보간/제한도 없습니다. 따라서 모터에 이미 저장된 Moving Speed 설정이 있다면 그것이 실제 이동 속도를 결정합니다.

이 구현은 충돌 회피나 정밀 토크 제어를 하지 않습니다. 임시 URDF 각도 범위는 실측 전입니다. 호스트 전원 차단/강제 종료/단선 시 소프트웨어의 토크 OFF를 보장할 수 없습니다. 기본 설정은 모터에 저장된 Max Torque와 Shutdown을 변경하지 않습니다. 목표 입력의 모델 각도 범위, 기존 장치 범위, 통신 오류 처리와 클라이언트 timeout은 유지합니다.

## 검증과 출처

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 src/dual_arm_hardware/scripts/test_ax12_guard.py
```

가짜 버스의 오류 주입 검사이며 실제 모터 시험이 아닙니다. 잘못된 ID/방향/설정, 단위 변환, 전체 사전 검사, 쓰기 후 불일치, 중간 활성화 실패, 토크 상한 변경, 모터 재부팅, OFF 확인 실패 등을 검사합니다.

- [AX-12A 제어표·Torque Limit·Shutdown](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/)
- [공식 Python SDK Protocol 1.0 예제](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/sample_code/python_read_write_protocol_1_0/)

## 실기 상태 모니터 연결

별도 터미널에서 워크스페이스의 `./monitor.sh`를 실행하면 `/dual_arm/diagnostics`로 발행되는 측정값과 보호 오류를 확인할 수 있습니다. 구동기는 기존 보호 검사 후 현재각·목표각·속도·추정 부하·전압·온도·토크 상태/제한을 읽습니다. `/joint_states`에는 측정 모터 위치와 그리퍼 각도에서 계산한 수동 관절 위치를 포함하며 `velocity`와 `effort`는 비워 둡니다. 측정 속도는 진단 토픽에서 확인합니다. 오류 시 기존 정지 경로를 실행하고 진단 오류를 계속 발행합니다. 자세한 사용법은 [모니터 설명](../dual_arm_monitor/README.md)을 참고하세요.
