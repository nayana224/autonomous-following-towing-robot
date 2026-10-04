# 개발 및 유지보수

이 문서는 일반 사용자가 아닌 개발자·유지보수 담당자를 위한 참고 사항을 정리합니다.

설치와 실행은 [설치 및 실행](setup.md), 실제 운용은 [운용 안내](operation.md)를 먼저 참고하세요.

## 기본 Workspace 경로

문서의 예시는 다음 Workspace 구조를 기준으로 합니다.

```text
~/ros2_ws/
└── src/
    └── autonomous-following-towing-robot/
```

Host에서 repository로 이동할 때:

```bash
cd ~/ros2_ws/src/autonomous-following-towing-robot
```

Container에서는 Workspace가 `/workspace`에 mount됩니다.

## Workspace Build

Laptop:

```bash
./src/autonomous-following-towing-robot/scripts/build_ws.sh
source /workspace/install_laptop/setup.bash
```

Jetson:

```bash
./src/autonomous-following-towing-robot/scripts/build_ws.sh
source /workspace/install_jetson/setup.bash
```

`AFTR_BUILD_TARGET`에 따라 Laptop과 Jetson build/install/log 디렉터리가 분리됩니다.

## Test 실행

일반 배포 절차에는 필수가 아니며 코드 변경 후 회귀 검증 시 사용합니다.

### Laptop

```bash
colcon   --log-base /workspace/log_laptop   test   --build-base /workspace/build_laptop   --install-base /workspace/install_laptop

colcon   test-result   --test-result-base /workspace/build_laptop   --verbose
```

### Jetson

```bash
colcon   --log-base /workspace/log_jetson   test   --build-base /workspace/build_jetson   --install-base /workspace/install_jetson

colcon   test-result   --test-result-base /workspace/build_jetson   --verbose
```

테스트 개수나 과거 pass/fail 수치는 코드 변경에 따라 달라지므로 이 문서에 고정하지 않습니다.

## 알려진 정적 검사 항목

현재 일부 팀원 담당 package에는 기존 lint/docstring 정리 항목이 남아 있을 수 있습니다. Build 또는 runtime 오류와 구분해서 다룹니다.

단순 formatting만을 이유로 전체 package를 대규모 수정하지 말고, 실제 기능 변경과 별도로 처리하는 것을 권장합니다.

## compile_commands.json

C++ 개발 편의를 위해 `aftr_hardware`와 `aftr_teleop`은 CMake compile database를 생성합니다.

현재 repository root의 `compile_commands.json`은 build script가 `aftr_hardware`의 compile database를 가리키도록 생성합니다.

VSCode 예:

```json
{
  "C_Cpp.default.compileCommands": "${workspaceFolder}/compile_commands.json"
}
```

## Headless 실행

GUI가 필요 없는 개발 환경:

```bash
./scripts/docker_run_laptop.sh --headless
./scripts/docker_run_jetson.sh --headless
```

## rosdep 확인

필요한 경우 Container에서:

```bash
rosdep update
rosdep check   --from-paths /workspace/src   --ignore-src   --rosdistro humble   --skip-keys=ament_python
```

## 자주 확인할 문제

### Overlay를 찾지 못하는 경우

첫 build 직후 같은 shell에서는 직접 source합니다.

```bash
source /workspace/install_laptop/setup.bash
# 또는
source /workspace/install_jetson/setup.bash
```

### Laptop / Jetson build가 섞인 경우

아키텍처가 다르므로 build 산출물을 공유하지 않습니다.

```text
Laptop: build_laptop / install_laptop / log_laptop
Jetson: build_jetson / install_jetson / log_jetson
```

### GUI가 표시되지 않는 경우

Host의 `DISPLAY`, X11/XWayland socket 및 `XAUTHORITY` 상태를 확인합니다. GUI가 필요하지 않으면 `--headless`를 사용합니다.

### Serial alias가 없는 경우

Host의 udev rule과 장치 연결 상태를 확인합니다.

```bash
ls -l /dev/ttyMotor /dev/ttyLidar
```

### Model을 찾지 못하는 경우

Host의 다음 경로를 확인합니다.

```text
models/fall_detection/yolov8n-pose.pt
```

Container에서는:

```text
/models/fall_detection/yolov8n-pose.pt
```

## 변경 시 원칙

- ROS topic/service/action 이름을 임의로 변경하지 않습니다.
- Runtime data 경로를 임의로 변경하지 않습니다.
- Hardware packet, checksum, byte order 및 motor safety sequence 변경은 실제 hardware 영향 검토가 필요합니다.
- 기능 변경과 style/lint 정리를 한 commit에 섞지 않는 것을 권장합니다.
- 외부 package `laser_filters`, `serial-ros2`, `sllidar_ros2`는 별도 repository로 관리합니다.
