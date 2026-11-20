# Windows에서 양팔 디지털 트윈 앱 실행

Windows PC에서 현재와 같은 **MuJoCo 3D 창, 10개 관절 조작창, 모터 ID, 현재각·목표각·오차·속도·토크 표시**를 직접 실행합니다. 회전·확대·이동, 모터 ON/OFF, 영점·어깨 90°·팔 굽히기, CSV 기록도 같은 코드로 동작합니다.

## 처음 실행

1. Windows 10/11 x64 PC에 [Python 3.12 64-bit](https://www.python.org/downloads/windows/)를 설치합니다. 설치 시 Python launcher 또는 `Add Python to PATH`를 활성화합니다. Python 3.10/3.11 64-bit도 선택할 수 있습니다.
2. `Dual_Arm_Windows.zip`을 Windows PC로 옮긴 뒤, 쓰기 가능한 폴더(예: `C:\Robots\Dual_Arm`)에 **전체 압축 해제**합니다. 저장소 전체를 복사한 경우에도 동일하게 실행할 수 있습니다.
3. `simulation.bat`를 더블클릭합니다.
4. 첫 실행에는 인터넷으로 MuJoCo·Qt·모델 변환 도구를 설치합니다. 설치가 끝나면 3D 앱이 열립니다. 이후에는 같은 파일을 더블클릭하면 됩니다.

ROS, WSL, Linux PC 연결은 필요하지 않습니다. Python과 의존성을 포함한 단일 EXE 배포본은 아니며, 실행 배치 파일이 프로젝트 전용 `.venv-windows` 환경을 준비합니다. 시스템 Python의 패키지는 변경하지 않습니다.

## 모델과 조작

- 모델·설정은 `src/dual_arm_description`, 제어 설정은 `src/dual_arm_simulation/config/simulation.yaml`을 사용합니다.
- 매 실행 시 현재 Xacro/YAML로 URDF와 MuJoCo 모델을 다시 생성합니다. 변경한 관절 제한도 다음 실행에 반영됩니다.
- 왼팔/오른팔은 로봇 기준입니다. 오른팔 ID는 어깨부터 5·4·3·2, 안쪽 집게 12·바깥 집게 1입니다. 왼팔은 10·9·8·7, 안쪽 13·바깥 6입니다.
- 마우스 왼쪽 드래그는 회전, 오른쪽/Shift 드래그는 이동, 휠은 기존과 같은 방향으로 확대·축소합니다.
- CSV와 생성 모델은 이 폴더의 `log/simulation`에 저장됩니다.
- Windows 앱은 해당 PC에서 시뮬레이션을 계산합니다. Linux에서 열어 둔 앱이나 실제 로봇 상태와 실시간 동기화되는 구성은 아닙니다.

명령 프롬프트에서 다음 실행 옵션도 사용할 수 있습니다.

```bat
simulation.bat --pose shoulders90
simulation.bat --headless --seconds 5
simulation.bat --headless --seconds 2 --screenshot log\simulation\windows_check.png
```

## 실행 문제가 있을 때

- Python을 찾지 못하면 Python 3.12 64-bit와 Python launcher 설치 여부를 확인합니다.
- 설치 오류가 나면 표시된 오류를 확인한 뒤 다시 실행합니다. 성공한 설치만 기록하므로 다음 실행에서 재시도합니다.
- 검은 화면이나 OpenGL 오류가 나면 그래픽 드라이버를 확인합니다. Windows에서는 GLFW 기반 OpenGL을 사용하며 Linux의 EGL 설정은 적용하지 않습니다.
- 오류가 발생하면 배치 창이 열려 있으므로 오류 내용을 확인할 수 있습니다. 압축 파일 내부에서 바로 실행하지 마세요.

검증 범위: Linux에서 ROS 없는 별도 Python 환경으로 동일 실행 경로·물리 모델·GUI를 검사하고 Windows용 의존성 배포 파일을 확인합니다. 실제 Windows 그래픽 드라이버와 배치 실행은 Windows PC에서 추가 확인이 필요합니다.

공식 참고: [MuJoCo Python 설치](https://mujoco.readthedocs.io/en/stable/python.html), [Xacro Python 패키지](https://pypi.org/project/xacro/).

## 모의 실기 연동 시연

명령 프롬프트에서 프로젝트 폴더로 이동한 뒤 실행합니다.

```bat
simulation.bat --twin mock --twin-demo
```

동일 목표를 MuJoCo와 모의 모터 버스에 보내고 돌아온 각도를 확인하는 모드입니다. 실제 모터와 연결하지 않습니다. U2D2 실기 연동은 현재 Linux의 `./simulation.sh --twin real` 경로를 사용합니다.
