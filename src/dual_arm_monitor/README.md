# 실기 듀얼 팔 모니터

**가상 로봇을 조작하며 물리 상태를 보는 디지털 트윈 화면은 `./monitor.sh --sim` 또는 `./simulation.sh`로 실행합니다.** [시뮬레이션 사용법](../dual_arm_simulation/README.md)을 참고하세요. 아래 내용은 별도의 실기 진단 모니터입니다.

`dual_arm_hardware`가 발행하는 `/dual_arm/diagnostics`를 받아 12개 모터 상태를 터미널에 표시합니다. 모니터는 시리얼 포트를 열거나, 명령을 발행하거나, 토크 서비스를 호출하지 않습니다. 모니터를 종료해도 구동기는 계속 실행됩니다.

워크스페이스에서 별도 터미널로 실행합니다.

```bash
./monitor.sh
./monitor.sh --csv                 # log/monitor/UTC시각.csv에 수신 프레임 기록
./monitor.sh --once                # 첫 프레임 출력, 5초간 수신 없으면 종료 코드 2
./monitor.sh --demo                # 장치/ROS 연결 없이 예시 화면만 출력
```

실기 구동은 기존 `./hardware.sh run`으로 별도로 시작합니다. `src/dual_arm_hardware/config/hardware.yaml`의 실제 포트·baudrate·12개 ID/방향·토크 제한 비율과 SDK/pyserial 준비가 필요합니다. 모니터만 실행하면 `WAITING`이 표시됩니다. `display.sh`의 RViz 프리뷰는 실기 진단 토픽을 발행하지 않습니다. 두 터미널의 `ROS_DOMAIN_ID`가 같아야 합니다.

## 표시 값

| 항목 | 의미 |
|---|---|
| Joint / ID | 좌우 관절 이름과 실제 설정된 모터 ID |
| q(deg) / Goal / Err | URDF 기준 현재각 / 모터에서 읽은 목표각 / 목표각−현재각. 모터 영점 180°가 q=0°이며 encoder 양자화 포함 |
| deg/s | 현재 속도 피드백, 관절 방향 보정 적용. 속도 제한을 설정하지 않음 |
| Load% | 모터 추정 부하 비율. +CCW/−CW는 모터 기준이며 URDF 관절 부호와 별개 |
| V / C | 입력 전압(V) / 내부 온도(°C) |
| EN / Cap% | 읽어 온 Torque Enable ON/OFF / RAM Torque Limit 비율 |
| Status | 정상 수신 OK / 보호 오류 FAULT / 누락 NO DATA / 수신 중단 STALE |

Goal은 모터에 남아 있는 목표 레지스터 값입니다. **토크 OFF 상태의 Goal/Err는 실제 이동 명령이나 추종 상태를 뜻하지 않습니다.** Load는 실제 토크(N·m)나 전류(A) 측정값이 아닙니다. 단위와 부호는 [ROBOTIS AX-12A 제어 테이블](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#present-speed-38)을 따릅니다.

## 수신 및 오류 처리

구동기는 기존 보호 레지스터 검사 후 각 모터의 주소 24–46을 한 패킷으로 읽습니다. `diagnostic_msgs/msg/DiagnosticArray`에 `dual_arm/system` 및 관절별 상태 12개를 발행합니다. 기본 폴링 목표는 5Hz이며 실제 주기는 baudrate·반환 지연·버스 상태에 따라 길어질 수 있습니다. 12개 모터 값은 순차 수집되며 같은 순간의 동시 측정이 아닙니다.

모든 읽기가 성공한 프레임만 유효 데이터로 표시합니다. 통신/상태 오류 또는 보호값 불일치가 발생하면 기존 정지 경로를 실행하고 오류를 계속 발행합니다. 오류 이후 측정값은 비워 두며, 토크 OFF가 확인되지 않은 모터는 별도로 표시합니다. 재시작 전 자동 복구/재활성화는 없습니다. Status는 구동기 통신·검사 결과이며 별도의 실측 토크 판정이 아닙니다.

기본 화면 갱신은 1초, 마지막 수신 후 2초가 지나면 `STALE`입니다. 이때 남아 있는 숫자는 과거 값이라고 표시합니다. `--refresh 0.5 --stale-after 3`처럼 표시 주기를 조정할 수 있습니다. `--topic`과 `--ros-args`도 지원합니다. 상태 신선도는 수신측 monotonic 시계를 사용하고 ROS 발행 시각을 함께 표시합니다.

CSV에는 새로 받은 프레임만 기록하며 시스템 상태도 한 행으로 남깁니다. 단절 중 가짜 샘플을 추가하지 않습니다. 수신 시각·ROS 시각·관절·오류·표시 값과 encoder 원시값, 모터 절대각, Moving 플래그를 저장합니다. 지정 경로에 기존 파일이 있으면 덮어쓰지 않고 실패합니다. 예시 모드는 CSV 기록과 ROS 발행을 하지 않습니다.

## ROS 패키지로 실행 및 검증

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dual_arm_hardware dual_arm_monitor
source install/local_setup.bash
export ROS_LOG_DIR="$PWD/log/monitor_ros"
ros2 run dual_arm_monitor monitor_node.py

/usr/bin/python3 src/dual_arm_hardware/scripts/test_ax12_guard.py
/usr/bin/python3 src/dual_arm_hardware/scripts/test_hardware_telemetry.py
/usr/bin/python3 src/dual_arm_monitor/scripts/test_monitor_view.py
```

단위·부호·수신 중단·오류 데이터 무효화·CSV·구동기의 오류 발행은 가상 버스와 ROS 메시지 직렬화로 검사합니다. 이는 실제 모터 통신/부하 시험을 대신하지 않습니다.

2026-10-04 검증: `dual_arm_hardware`·`dual_arm_monitor` 빌드 성공, 기존 보호 검사 20개와 새 텔레메트리/모니터 검사 14개 통과, 쉘의 예시 화면과 미수신 시 `--once` 종료 코드 2 확인. 현재 실행 환경의 DDS 소켓 제한으로 프로세스 간 토픽 전달은 검증하지 못했으며 실제 모터 통신도 미검증입니다.
