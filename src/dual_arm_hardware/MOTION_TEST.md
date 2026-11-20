# 실기 손목 구동 시험 — 2026-11-21

FTDI by-id 포트(당시 `/dev/ttyUSB1`), 1 Mbps에서 실제 AX-12A 손목 모터를
한 개씩 시험했습니다. 현재 측정 위치를 유지한 뒤 약 +2°의 상대 목표로
1.5초, 원래 위치 목표로 1.5초 구동하고 토크 OFF를 재조회했습니다.
이 결과는 엔코더 응답이며 로봇 정면 기준 회전 방향을 검증한 결과는 아닙니다.

| 관절 | ID | 시작 → 이동 목표 → 복귀 목표(raw) | 실측 이동량 | 이동 목표 오차 | 복귀 오차 |
|---|---:|---|---:|---:|---:|
| 오른쪽 손목 | 2 | 523 → 530 → 523 | +2.053° | 0.000° | 0.587° |
| 왼쪽 손목 | 7 | 500 → 507 → 500 | +1.760° | 0.293° | 0.587° |

두 시험 모두 목표 오차 2 tick 이내 기준으로 통과했습니다. 오른쪽은 12.0V·33°C,
왼쪽은 12.0V·32°C를 읽었습니다. 해당 손목 한 개에만 목표 위치와 토크
ON/OFF를 기록했으며, 나머지 모터는 활성화하지 않았습니다. 보호값·Moving Speed는
변경하지 않았습니다. 시험 후 전체 10개 모터의 토크 OFF와 기존 CW/CCW·Max Torque·
Torque Limit·Shutdown 값이 시험 전과 동일함을 확인했습니다.

기록:

- `log/motion_test/20261120T183922_307549Z_motion.json` — 오른손목 실측·명령·통신
- `log/motion_test/20261120T183937_567513Z_motion.json` — 왼손목 실측·명령·통신
- `log/motion_test/20261121_wrist_test_summary.json` — 두 시험 요약
- `log/hardware_limits/20261120T183949_805462Z_apply.json` — 시험 후 전체 읽기 검증

로그 파일의 시각은 UTC이며 실제 시험은 한국 시각 2026-11-21 03:39경입니다.

재실행은 저장소 루트에서 아래 명령을 사용합니다. 기본 실행은 메모리 에뮬레이터이며,
`--real`을 추가하면 선택한 손목이 실제로 움직입니다.

```bash
PYTHONPATH="$PWD/.venv-hardware/lib/python3.10/site-packages:$PYTHONPATH" \
  .venv-hardware/bin/python src/dual_arm_hardware/examples/motion_test.py \
  --real --joint right_wrist_roll_joint --delta-deg 2
```

`left_wrist_roll_joint`로 왼손목을 선택할 수 있습니다. 모터 영점은 기존 180°를
유지합니다. 일부 다른 관절의 현재 측정 위치가 모델의 목표각 범위 밖이므로,
전체 팔 제어를 사용하려면 실제 기준 자세·영점·방향 대조가 남아 있습니다.
손목 두 개의 소각도 구동을 확인한 것이며 전체 팔 궤적 일치나 하중 시험은 수행하지 않았습니다.
