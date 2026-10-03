# Laptop Docker Development Environment

## 1. 목적

Laptop 개발과 CI에 사용하는 CPU-only 환경입니다. Linux amd64 Host와 Docker가 설계 대상이며, Phase 1 검증은 Ubuntu 26 LTS amd64 Host에서 수행했습니다.

```text
Linux amd64 Host → Docker → Ubuntu 22.04 Jammy → ROS 2 Humble → CPU-only PyTorch
```

이 환경에서 Workspace Build, Test, lint, Launch/YAML 정적 검사, Python import와 CPU 장치 선택 경로를 검증합니다. 실제 motor, LiDAR, RealSense, GPIO 및 GPU inference 검증은 Jetson 단계에서 수행합니다. [Dockerfile](../../docker/laptop/Dockerfile)은 `ros:humble-ros-base-jammy`를 사용하며 소스는 image에 COPY하지 않습니다.

## 2. 사전 요구사항

- Linux amd64 Host와 실행 가능한 Docker Engine (검증된 Host: Ubuntu 26 LTS amd64)
- `~/autonomous_following_towing_robot_ws/src/autonomous-following-towing-robot`
- 같은 `src/` 아래의 external sibling `laser_filters`, `serial-ros2`, `sllidar_ros2`

세 sibling은 image build 시 COPY되지 않습니다. Container 실행 script가 존재 여부를 확인하고 Workspace 전체를 bind mount합니다. Host에 설치된 ROS 배포판은 Container의 ROS Humble에 영향을 주지 않습니다.

## 3. Workspace 구조

```text
~/autonomous_following_towing_robot_ws/
├── src/
│   ├── autonomous-following-towing-robot/
│   ├── laser_filters/
│   ├── serial-ros2/
│   └── sllidar_ros2/
├── models/fall_detection/yolov8n-pose.pt
├── data/
│   ├── paths/
│   ├── maps/
│   └── poses/
├── build_laptop/
├── install_laptop/
└── log_laptop/
```

Host Workspace는 Container의 `/workspace`에 연결됩니다. `build_laptop/install_laptop/log_laptop`은 amd64 Humble 전용입니다. 기존 `build/install/log`에는 다른 ROS 배포판의 산출물이 있으므로 재사용하지 않습니다. Jetson arm64 산출물과도 공유하지 않습니다.

## 4. Docker image build

Repository root에서 실행합니다.

```bash
cd ~/autonomous_following_towing_robot_ws/src/autonomous-following-towing-robot
./scripts/docker_build_laptop.sh
```

[빌드 script](../../scripts/docker_build_laptop.sh)는 `docker/laptop/Dockerfile`을 `linux/amd64`로 빌드하고 `aftr-dev:humble-cpu` 태그를 지정합니다. [Ubuntu apt 목록](../../docker/common/apt-packages.txt)과 [ROS apt 목록](../../docker/common/ros-packages.txt)을 설치합니다. PyTorch `2.2.2+cpu`, torchvision `0.17.2+cpu`, Ultralytics `8.2.103`, NumPy `1.26.4`, OpenCV Python `4.8.1.78`을 직접 지정합니다. Jammy의 apt NumPy보다 새 버전이 Ultralytics에 필요해 NumPy 1.x ABI를 유지한 채 pip 버전을 지정했습니다. 한글 GUI glyph를 위해 `fonts-noto-cjk`를 설치하고 image build 중 font cache를 갱신합니다. apt repository와 전이 의존성의 버전까지 고정된 lockfile은 아직 없습니다.

## 5. Container 실행

```bash
./scripts/docker_run_laptop.sh
```

[실행 script](../../scripts/docker_run_laptop.sh)는 기본적으로 interactive `bash`를 열고 Workspace를 `/workspace`에 bind mount합니다. Host에 사용 가능한 로컬 X11/XWayland display가 있으면 GUI forwarding을 자동으로 활성화합니다. Display가 없으면 headless로 전환합니다. CI에서는 `--headless`를 명시할 수 있습니다.

```bash
./scripts/docker_run_laptop.sh --headless
./scripts/docker_run_laptop.sh --headless colcon list --names-only
```

명령을 전달하면 shell 대신 해당 명령을 실행합니다. `--headless`는 명령보다 앞에 둡니다.

### Model과 runtime data mount

실행 script는 기본적으로 Workspace `models/`를 `/models:ro`로, `data/`를 `/data:rw`로 bind mount합니다. `models/`와 `data/{paths,maps,poses}/`가 없으면 Host 사용자 권한으로 만듭니다. 모델 파일 자체는 제공하지 않습니다. 모델의 기본 위치는 Host `models/fall_detection/yolov8n-pose.pt` → Container `/models/fall_detection/yolov8n-pose.pt`입니다. 해당 파일이 없으면 실제 Fall Detection inference는 실행할 수 없습니다.

별도 Host 저장소를 사용할 때는 **절대경로**를 지정합니다.

```bash
AFTR_MODELS_DIR=/absolute/path/to/models AFTR_DATA_DIR=/absolute/path/to/data ./scripts/docker_run_laptop.sh
```

`/data/paths`에는 `recorded_path.csv`, `safe_path.csv`, `corner_turn_path.csv`가, `/data/maps`에는 `mdbot_map.yaml`과 `mdbot_map.pgm`이, `/data/poses`에는 `last_pose.yaml`이 저장됩니다. Map saver의 prefix는 `/data/maps/mdbot_map`이며 YAML의 `image: mdbot_map.pgm` 상대 참조를 유지합니다. 이 이름은 runtime data contract입니다. `/data`와 하위 디렉터리는 Host UID/GID로 쓰기 가능해야 합니다. Script는 시작 전에 권한을 확인합니다. Docker 내에서도 같은 UID/GID로 실행되어 파일 소유권이 Host 사용자에게 남습니다.

```bash
./scripts/docker_run_laptop.sh --headless bash -lc 'id; test -r /models; test -w /data/paths; test -w /data/maps; test -w /data/poses'
```


### Container 사용자와 파일 소유권

Container 시작 시 [Entrypoint](../../docker/common/entrypoint.sh)가 Host UID/GID에 대응하는 passwd/group entry를 확인하고, 필요하면 생성합니다. 설정이 끝나면 해당 UID/GID로 권한을 낮춰 명령을 실행합니다. 사용자 이름은 일반적으로 `aftr`이며, 기존 UID entry가 있으면 그 이름을 사용합니다. `HOME`은 쓰기 가능한 `/home/aftr-<UID>`입니다. Workspace bind mount에 쓰는 파일은 Host 사용자 소유로 생성됩니다. 초기 사용자 설정을 위해 Container가 잠시 root로 시작하지만, 애플리케이션은 Host UID/GID로 실행됩니다. `--privileged`는 사용하지 않습니다.

```bash
./scripts/docker_run_laptop.sh id
./scripts/docker_run_laptop.sh whoami
./scripts/docker_run_laptop.sh groups
./scripts/docker_run_laptop.sh bash -c 'echo "$HOME"; test -w "$HOME"'
```

### GUI와 headless

GUI 자동 연결에는 Host의 `DISPLAY`, 해당 `/tmp/.X11-unix/X<N>` socket, 읽을 수 있는 `XAUTHORITY` 파일(미설정 시 `~/.Xauthority`)이 필요합니다. Linux Wayland session의 XWayland display도 지원합니다. Script는 X11 socket과 인증 파일을 Container에 read-only로 mount하고 `DISPLAY`와 `XAUTHORITY`를 전달합니다. Qt는 `xcb`를 사용하며 `LIBGL_ALWAYS_SOFTWARE=1`로 software OpenGL을 요청합니다. `XDG_RUNTIME_DIR`은 Container 사용자 소유의 별도 디렉터리로 준비됩니다.

GUI의 한글 label은 Container 안의 CJK font가 필요합니다. Font fallback은 `fonts-noto-cjk`에 맡기며 GUI source/UI에서 특정 운영체제 font family를 강제하지 않습니다. 설치 상태는 GUI mode Container에서 `fc-list :lang=ko | head`와 `fc-match "Noto Sans CJK KR"`로 확인할 수 있습니다. 재빌드 후 `fc-match "Noto Sans CJK KR"`가 `NotoSansCJK-Regular.ttc`를 선택했고, 실제 Operator UI 파일의 한글·영문·숫자가 GUI mode에서 렌더링되는 것을 확인했습니다. `fc-match sans-serif`는 `DejaVu Sans`를 선택하지만 한글 glyph에는 CJK fallback이 적용됩니다.

GUI가 있어도 PyTorch는 CPU-only입니다. GPU device, CUDA, TensorRT, NVIDIA Container Runtime 및 `--gpus`는 사용하지 않습니다. `SDL_AUDIODRIVER=dummy`는 양쪽 모드에 적용됩니다. 명시적인 headless 모드나 display 자동 감지 실패 시 `QT_QPA_PLATFORM=offscreen`을 사용합니다.

Host에서 robot visualization을 시작하는 예:

```bash
./scripts/docker_run_laptop.sh ros2 launch aftr_description view_robot.launch.py
```

현재 개발 Host는 Wayland/XWayland, `DISPLAY=:0`에서 테스트했습니다. `robot_state_publisher`가 URDF를 읽었고, `joint_state_publisher_gui`와 RViz2 창이 Host display에 나타났습니다. RViz2는 OpenGL 4.5를 보고했고 X display 오류가 없었습니다. 화면의 robot model 픽셀까지는 별도로 확인하지 않았습니다. Ctrl-C로 종료할 때 RViz2가 exit code -11로 끝난 현상은 아래 제한사항에 기록합니다.

## 6. Workspace Build

Container shell은 `/workspace`에서 시작합니다.

```bash
./src/autonomous-following-towing-robot/scripts/build_ws.sh
```

[Build script](../../scripts/build_ws.sh)는 `ROS_DISTRO=humble`을 확인한 뒤 Workspace root에서 다음을 실행합니다.

```bash
colcon --log-base log_laptop build --build-base build_laptop --install-base install_laptop --symlink-install
```

Build script의 기본 target은 계속 Laptop이며 Jetson 실행 script가 별도 target을 설정합니다. 추가 인자는 Build script가 colcon에 전달합니다. 예: `./src/autonomous-following-towing-robot/scripts/build_ws.sh --parallel-workers 2`. Phase 1 전체 Build는 외부 3개와 `aftr_*` 13개, 합계 16개 Package에서 성공했습니다. 외부 저장소 소스는 수정하지 않습니다.

## 7. Overlay source

[Entrypoint](../../docker/common/entrypoint.sh)는 Container 시작 시 `/opt/ros/humble/setup.bash`를 source하고, 이미 있으면 `/workspace/install_laptop/setup.bash`도 source합니다. 첫 Build를 마친 **같은 shell**에서는 새 overlay를 직접 읽습니다.

```bash
source /workspace/install_laptop/setup.bash
```

새 Container를 열면 기존 Laptop overlay는 자동으로 읽힙니다. 공유 Entrypoint의 기본값은 계속 `install_laptop`입니다. Jetson 실행 script만 `install_jetson`을 선택합니다. Package import가 안 되면 `ROS_DISTRO`와 overlay 경로를 먼저 확인합니다.

## 8. Package 확인

```bash
colcon list --names-only
```

예상 결과는 `aftr_*` 13개와 `laser_filters`, `serial`, `sllidar_ros2`입니다. `mdbot_*` Package identity는 없어야 합니다. 디렉터리 이름 `serial-ros2`의 실제 ROS Package 이름은 `serial`입니다.

## 9. Test

Container의 `/workspace`에서 다음 명령을 사용합니다.

```bash
colcon --log-base log_laptop test --build-base build_laptop --install-base install_laptop --parallel-workers 2
colcon test-result --test-result-base build_laptop --verbose
```

`--parallel-workers 2`는 Phase 1 검증 때 사용한 값이며 필수는 아닙니다. 이 Test는 hardware launch를 시작하지 않습니다. 기본 `aftr_gui/operator_system.launch.py`는 camera와 robot base를 시작하므로 Laptop에서 실행하지 않습니다.

## 10. CPU-only PyTorch 검증

```bash
python3 - <<'PYTORCH'
import torch

print("torch:", torch.__version__)
print("torch.version.cuda:", torch.version.cuda)
print("cuda available:", torch.cuda.is_available())
assert torch.version.cuda is None
assert torch.cuda.is_available() is False
PYTORCH
```

Phase 1 결과는 `2.2.2+cpu`, `None`, `False`입니다. CPU wheel 설치, GPU device 미전달, 빈 `CUDA_VISIBLE_DEVICES` 값을 함께 사용합니다.

## 11. aftr_fall_detection 검증

Build와 overlay source 이후 import를 검사합니다.

```bash
python3 -c 'import cv2, numpy, torch, cv_bridge, ultralytics; from aftr_fall_detection import fall_detection_node; print("fall detection import: PASS")'
```

Phase 1에서 import와 CUDA 부재 시 `cpu` 선택 경로가 통과했습니다. 현재 기본 `model_path`는 `/models/fall_detection/yolov8n-pose.pt`이며 ROS parameter로 override할 수 있습니다. 실제 model file이 없으면 영상 추론을 실행하지 않습니다.

## 12. rosdep

Image에는 `python3-rosdep`과 확인된 apt 의존성을 설치했습니다. Source를 복사하지 않으므로 Build 때 `rosdep install`을 자동 실행하지 않습니다. Container에서 의존성 상태를 확인하려면 다음을 실행합니다.

```bash
rosdep update
rosdep check --from-paths /workspace/src --ignore-src --rosdistro humble --skip-keys=ament_python
```

현재 네 Python Package manifest가 `ament_python`을 build tool로 선언하지만 Humble rosdep에는 해당 키의 매핑이 없습니다. `--skip-keys` 없이 검사하면 exit code 2가 발생합니다. 해당 키만 제외한 `rosdep check`와 `rosdep install`은 Phase 1에서 모두 성공했고, 필요한 시스템 의존성은 이미 충족되었습니다. 이는 Docker Build 실패가 아니라 추후 정리할 manifest/rosdep metadata 문제입니다.

새 의존성이 생기면 [apt 목록](../../docker/common/apt-packages.txt), [ROS apt 목록](../../docker/common/ros-packages.txt) 또는 Dockerfile을 수정해 image를 다시 빌드합니다. 일회성 Container에서 apt를 설치해도 다음 실행에 유지되지 않습니다.

## 13. Validation Baseline

Phase 1 검증 결과와 기존 lint 실패 내역은 [Testing](../testing.md)에 기록합니다. 현재 상태는 위 명령으로 다시 확인하세요.

## 14. Known Limitations

- 실제 Fall Detection model 파일이 제공되지 않아 inference는 미검증입니다. 기본 model path는 `/models/fall_detection/yolov8n-pose.pt`입니다.
- Robot에서 `/models`와 `/data` mount 및 기존 데이터 이관은 별도로 확인해야 합니다.
- 기존 lint/docstring 실패 4개가 남아 있습니다.
- 실제 motor, LiDAR, RealSense, GPIO, GPU inference 및 전체 operator hardware Launch는 이 환경에서 검증하지 않았습니다.
- GUI launch의 Ctrl-C 종료 후 RViz2가 exit code -11로 끝났습니다. 실행 중 GUI 창과 OpenGL 초기화는 확인했지만 종료 시 segfault 원인은 아직 조사하지 않았습니다.
- Jetson 전용 실행 방법과 완료된 CUDA·센서·GUI 검증은 [Jetson Docker 안내](jetson-docker.md)와 [검증 현황](../testing.md)을 참고하세요.

## 15. Troubleshooting

- **파일 소유권:** Entrypoint가 Host UID/GID에 맞는 passwd/group entry와 writable HOME을 준비한 뒤 권한을 낮춥니다. 수동 `docker run`에서 root로 Build했다면 생성 파일의 소유권을 확인합니다.
- **한글 네모 표시:** Image를 다시 빌드한 뒤 `fc-list :lang=ko | head`와 `fc-match "Noto Sans CJK KR"`를 확인합니다. 한글 font가 없으면 새 image tag로 Container를 실행 중인지 확인합니다.
- **GUI 연결:** Host의 `DISPLAY`, X11/XWayland socket, `XAUTHORITY` 파일을 확인합니다. 자동 headless 전환 메시지가 나오면 세 조건 중 하나가 없습니다. Display 없이 검증할 때는 `--headless`를 사용합니다.
- **ROS 산출물 혼동:** Laptop에서는 `build_laptop/install_laptop/log_laptop`만 사용합니다. 기존 `build/install/log`를 Humble overlay로 source하지 않습니다.
- **Import 실패:** 첫 Build 후 같은 shell에서는 `source /workspace/install_laptop/setup.bash`가 필요합니다.
- **CUDA 판별:** `torch.version.cuda`가 `None`이고 `torch.cuda.is_available()`가 `False`여야 합니다. 다른 결과면 image tag와 PyTorch 설치를 확인합니다.
- **rosdep exit code 2:** `ament_python` 매핑 문제입니다. 위의 `--skip-keys=ament_python`을 사용해 나머지 의존성을 검사합니다.
