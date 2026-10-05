# AX-12A 실기 보호 설정 및 구동

이 패키지는 **DYNAMIXEL Protocol 1.0 TTL 버스에 직접 연결된 AX-12A**를 대상으로 합니다. U2D2의 TTL 포트 같은 어댑터가 필요합니다. CM-530 등 제어 보드를 통하는 경우에는 해당 보드의 펌웨어/통신 방식이 먼저 확인되어야 하며, USB 포트 이름만으로 호환을 가정하지 않습니다.

현재 저장소의 `config/hardware.yaml`은 연결 정보와 출력 상한이 비어 있습니다. **실제 장치에 적용 완료된 상태가 아닙니다.** 미입력 값이 있으면 시리얼 포트를 열기 전에 중단합니다. 테스트의 1~12번 ID와 25%는 메모리상의 가짜 모터용 값이며 실기 설정값이 아닙니다.

## 필요한 설정

`config/hardware.yaml`에 실제 포트, baudrate, `torque_limit_percent`, 관절별 회전 방향을 입력합니다. ID는 사용자 사진 기준으로 입력되어 있습니다(오른팔: 어깨부터 5·4·3·2, 안쪽 집게 12·바깥 집게 1 / 왼팔: 어깨부터 10·9·8·7, 안쪽 집게 13·바깥 집게 6). ID는 중복 없는 0~253이며 254번 broadcast는 사용하지 않습니다. 방향은 URDF 각도를 늘릴 때 모터 raw 위치가 증가하면 +1, 감소하면 −1입니다. 중립은 `dual_arm_description/config/motor_calibration.yaml`의 180°를 사용합니다. 0~300°/0~1023 변환에서 가장 가까운 중립 raw 값은 614(약 180.06°)이며, 각도 경계는 안쪽으로 반올림합니다. 실제 장착 방향과 영점은 실측해 맞춰야 합니다.

출력 상한은 **% 단위**입니다. URDF의 `effort=0.3 N·m`를 자동 환산하지 않습니다. 예를 들어 25%를 지정하면 raw 255를 요청하지만, 이를 정확한 0.375 N·m로 해석하면 안 됩니다. `floor(1023 × percent / 100)`으로 계산하며 기존 모터의 Max Torque/Torque Limit이 더 낮으면 그 낮은 값을 유지합니다.

필요 패키지는 ROS 2 Humble, DYNAMIXEL SDK, Python pyserial과 PyYAML입니다. 배포 패키지 이름은 `ros-humble-dynamixel-sdk`, `python3-serial`, `python3-yaml`입니다. 이 작업 환경에서는 SDK 파일은 있지만 pyserial이 없어 실제 SDK 시리얼 통신을 검증하지 못했습니다.

## 실행

프로젝트 루트에서 실행합니다. 기존 실기 제어기와 프리뷰 발행기는 종료하고, 모터 전원이 켜져 있더라도 Torque Enable은 OFF여야 합니다. 토크 OFF 시 팔을 지지해야 합니다.

```bash
# 오프라인 설정 검증/계획. 장치를 열지 않음.
./hardware.sh plan

# 실제 12개 모터를 읽어 적용 가능 여부 확인. 쓰기 없음.
./hardware.sh check

# 보호값을 쓰고 재조회 검증. 모터를 켜거나 움직이지 않음.
./hardware.sh apply

# 보호값 적용/검증 후 실기 상태 발행과 RViz 시작. 초기 토크는 OFF.
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

실기 목표는 `/dual_arm/joint_targets` (`sensor_msgs/msg/JointState`)에 12개 관절 이름과 위치를 모두 넣어 보냅니다. 위치 단위는 URDF 기준 rad입니다. 범위를 벗어난 목표, NaN/Inf, 이름 중복·누락은 **모든 목표를 쓰기 전에** 거부합니다. `/joint_states`는 모터의 측정 위치이며 프리뷰 목표를 재발행하지 않습니다. AX-12A의 Present Load는 측정 토크가 아니므로 effort 필드는 비웁니다.

## 적용 순서와 실패 처리

1. 포트를 열기 전에 ID/방향/출력 상한과 각도 변환을 검증합니다.
2. 12개 모두 모델 번호 12, Torque OFF, Joint Mode, Status Return Level 2, 대기 중 REG_WRITE 없음, 현재 위치가 허용 범위 안인지 검사합니다. 활성화된 모터를 임의로 끄지 않고 적용을 거부합니다.
3. EEPROM의 Max Torque(14), Shutdown(18), CW/CCW Angle Limit(6/8), RAM의 Torque Limit(34)을 설정하고 다시 읽습니다. 기존의 더 좁은 각도 범위, 더 낮은 토크 상한과 기존 Shutdown 비트는 유지합니다. 과열·과부하 Shutdown 비트(0x24)를 포함합니다. 변경된 EEPROM 값만 씁니다.
4. 하나라도 검증이 실패하면 활성화를 허용하지 않습니다. 일부 적용 후 실패해도 더 높은 토크값으로 되돌리지 않습니다. 기록을 확인하고 다시 점검해야 합니다.
5. 실제 구동 중 보호 레지스터와 상태를 5 Hz로 확인하며, 목표를 보내기 전에도 다시 검사합니다. 재부팅/설정 변경/오류가 감지되면 토크 OFF를 시도하고 오류를 유지합니다. 자동 재활성화나 자동 과부하 복구는 하지 않습니다. 통신 불능 등으로 OFF 재확인이 안 되면 **OFF 미확인**으로 보고합니다.

모터의 Moving Speed(32)는 읽어 덮어쓰거나 0으로 설정하지 않습니다. 사용자가 요청한 대로 호스트의 이동 속도 보간/제한도 없습니다. 따라서 모터에 이미 저장된 Moving Speed 설정이 있다면 그것이 실제 이동 속도를 결정합니다.

이 구현은 충돌 회피나 정밀 토크 제어를 하지 않습니다. 임시 URDF 각도 범위를 기계적으로 충돌 없는 범위로 검증한 것이 아니며, 낮은 출력 상한에서도 하중과 자세에 따라 기어 손상이 생길 수 있습니다. 호스트 전원 차단/강제 종료/단선 시 소프트웨어의 토크 OFF를 보장할 수 없습니다. 다만 Max Torque는 비휘발성으로 설정하므로 정상적인 모터 재부팅 후에도 출력 상한이 복원됩니다. 제조사 초기화나 다른 프로그램의 설정 변경까지 막는 기능은 아닙니다.

## 검증과 출처

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 src/dual_arm_hardware/scripts/test_ax12_guard.py
```

가짜 버스의 오류 주입 검사이며 실제 모터 시험이 아닙니다. 잘못된 ID/방향/설정, 단위 변환, 전체 사전 검사, 쓰기 후 불일치, 중간 활성화 실패, 토크 상한 변경, 모터 재부팅, OFF 확인 실패 등을 검사합니다.

- [AX-12A 제어표·Torque Limit·Shutdown](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/)
- [공식 Python SDK Protocol 1.0 예제](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/sample_code/python_read_write_protocol_1_0/)

## 실기 상태 모니터 연결

별도 터미널에서 워크스페이스의 `./monitor.sh`를 실행하면 `/dual_arm/diagnostics`로 발행되는 측정값과 보호 오류를 확인할 수 있습니다. 구동기는 기존 보호 검사 후 현재각·목표각·속도·추정 부하·전압·온도·토크 상태/제한을 읽습니다. `/joint_states`에는 측정 위치와 속도만 포함하며 `effort`는 비워 둡니다. 오류 시 기존 정지 경로를 실행하고 진단 오류를 계속 발행합니다. 자세한 사용법은 [모니터 설명](../dual_arm_monitor/README.md)을 참고하세요.
