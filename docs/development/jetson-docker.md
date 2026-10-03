# Jetson Docker Development Environment

## 대상과 사전 조건

NVIDIA Jetson Orin Nano Super (arm64), Ubuntu 22.04.5, L4T R36.5.2, CUDA 12.6, Docker와 NVIDIA Container Runtime이 대상입니다. 기반 image는 NVIDIA의 Jetson 전용 `nvcr.io/nvidia/l4t-cuda:12.6.11-runtime`입니다. CUDA 12.6과 Ubuntu 22.04를 제공하며 Jetson NVIDIA runtime을 사용합니다. JetPack 6 / CUDA 12.6 / Python 3.10용 arm64 PyTorch 2.8.0 및 torchvision 0.23.0 wheel을 SHA-256으로 고정해 설치합니다. 이 wheel은 [Jetson AI Lab 패키지 저장소](https://pypi.jetson-ai-lab.io/jp6/cu126/)에서 제공됩니다. NVIDIA의 공식 PyTorch wheel 목록에는 JetPack 6.2 전용 wheel이 없으므로 이 wheel의 배포 출처를 공식 NVIDIA wheel로 표기하지 않습니다. [NVIDIA의 24.10 PyTorch container](https://docs.nvidia.com/deeplearning/frameworks/pytorch-release-notes/rel-24-10.html)는 드라이버 560 이상이 필요해 Host 540.5.0에 적합하지 않습니다. 24.12 container는 Ubuntu 24.04/Python 3.12여서 ROS Humble Jammy binary와 맞지 않습니다.

Host 사용자가 Docker socket에 접근할 수 있어야 합니다. `docker info`가 permission denied를 반환하면 Host 관리자가 Docker 권한을 준비해야 합니다. 실행 script는 Docker daemon 설정을 변경하지 않습니다.

Workspace는 `~/260929_ws/src/` 아래에 AFTR 및 robot-tested `laser_filters`, `serial-ros2`, `sllidar_ros2` 사본이 있는 구조입니다. Image build는 sibling 소스를 복사하지 않으며, 실행 시 Workspace 전체를 `/workspace`에 연결합니다. 외부 저장소를 갱신하거나 초기화하지 마세요.

## Quick Start

Host에서:

```bash
cd ~/260929_ws/src/autonomous-following-towing-robot
./scripts/docker_build_jetson.sh
./scripts/docker_run_jetson.sh
```

Container 안에서:

```bash
source /opt/ros/humble/setup.bash
./src/autonomous-following-towing-robot/scripts/build_ws.sh
source /workspace/install_jetson/setup.bash
```

Build script는 Jetson run script가 설정하는 `AFTR_BUILD_TARGET=jetson`에 따라 `build_jetson/`, `install_jetson/`, `log_jetson/`을 사용합니다. Laptop의 `build_laptop/`, `install_laptop/`, `log_laptop/` 또는 수동 Host build 산출물을 Jetson에서 재사용하지 마세요. Entrypoint는 ROS Humble을 읽고 기존 Jetson overlay가 있으면 자동으로 읽습니다. 첫 Build 직후 같은 shell에서는 위처럼 직접 source합니다.

`docker/jetson/Dockerfile`은 Jammy/Python 3.10을 확인한 뒤 ROS 공식 `ros2-apt-source` 1.3.0 package와 NVIDIA Jetson r36.5 apt 저장소를 명시적으로 설정합니다. JetPack 6.2 계열의 cuDNN 9.3 runtime 및 CUDA CUPTI 12.6, 공통 apt/ROS package 목록, `ros-humble-ros-base`, `ros-humble-realsense2-camera` 및 GUI 의존성을 설치합니다. Ultralytics 8.2.103과 Jetson.GPIO 2.1.9는 Jetson image에만 설치합니다. NumPy 1.26.4는 ROS `cv_bridge`의 NumPy 1.x ABI에 맞춥니다. 기반 image digest와 wheel SHA-256은 고정했습니다. apt 전이 의존성 버전까지 고정한 lockfile은 없습니다.

## Model 및 runtime data

기본 Host 경로는 Workspace의 `models/`, `data/`입니다. Container에서는 각각 `/models:ro`, `/data:rw`입니다. Script는 `data/{paths,maps,poses}`를 Host 사용자로 준비하고 권한을 확인합니다. 실제 모델은 Host의 `models/fall_detection/yolov8n-pose.pt`에 준비해야 Container의 `/models/fall_detection/yolov8n-pose.pt`에서 읽힙니다. 모델 파일은 image에 포함하지 않습니다.

```bash
AFTR_MODELS_DIR=/absolute/models AFTR_DATA_DIR=/absolute/data ./scripts/docker_run_jetson.sh
```

`/data/paths/recorded_path.csv`, `safe_path.csv`, `corner_turn_path.csv`, `/data/maps/mdbot_map.yaml`, `/data/poses/last_pose.yaml`은 기존 runtime 경로입니다.

## Device, GPIO, audio, display

Run script는 NVIDIA runtime과 GPU, Host network, RealSense용 `/dev/bus/usb`, 현재 존재하는 `/dev/video*`, `/dev/gpiochip*`, `/dev/snd/*`를 연결합니다. USB bus에는 hotplug를 위한 USB character-device cgroup rule을 사용합니다. GPIO와 serial device는 실행 시 존재해야 합니다. `--privileged`나 `/dev` 전체 mount는 사용하지 않습니다. Host UID/GID와 필요한 장치 group ID를 Container 사용자에게 반영합니다. NVIDIA runtime이 연결하는 GPU 장치의 `video`/`render` group ID도 포함하고 root 보조 그룹은 부여하지 않습니다. 초기 계정 준비는 root로 진행하고 실제 명령은 Host UID/GID로 실행합니다. 장치 파일 자체의 Host 권한도 사용자에게 허용돼 있어야 합니다.

Host udev rule이 만든 `/dev/ttyMotor`와 `/dev/ttyLidar` alias를 Container 안에서도 같은 이름으로 제공합니다. Alias가 없으면 개발용 shell은 열리지만 hardware launch 전에 Host udev 설정과 장치 연결을 확인하세요. `ttyUSB0`과 `ttyUSB1` 번호를 ROS 설정에 직접 쓰지 마세요. RealSense ROS driver는 Container의 `ros-humble-realsense2-camera`를 사용합니다. `pyrealsense2` Python module은 별도 요구사항이 아니며 포함 여부는 image 검증 시 확인하세요.

NVIDIA [Jetson.GPIO Container 안내](https://github.com/NVIDIA/jetson-gpio#using-the-jetson-gpio-library-from-a-docker-container)에 맞춰 `JETSON_MODEL_NAME=JETSON_ORIN_NANO`와 GPIO chip device를 전달합니다. 실제 LED GPIO 사용에는 Host의 chip 접근 권한과 올바른 pinmux 설정이 필요합니다. Container 안의 import 성공만으로 물리 LED 동작이 입증되지는 않습니다.

사용 가능한 local `DISPLAY`, X11/XWayland socket, 읽을 수 있는 `XAUTHORITY`가 있으면 Qt `xcb`와 Host display를 사용합니다. 없으면 자동으로 headless로 전환합니다. `--headless`는 Qt offscreen 및 SDL dummy audio를 명시합니다. GUI mode에서는 Host PulseAudio socket과 읽기 전용 인증 cookie를 Container에 전달하고 SDL `pulseaudio` 드라이버를 사용합니다. PulseAudio socket이 없으면 기존 ALSA 장치 접근을 시도합니다. 2026-10-03 X11에서 operator GUI 창을 확인했고 HDMI speaker에서 `system_ready.wav` 시험 재생을 들었습니다. 화면 조작 전체와 운영 중 모든 audio event는 아직 검증되지 않았습니다.

```bash
./scripts/docker_run_jetson.sh --headless
./scripts/docker_run_jetson.sh --headless id
```

## 안전한 검증 명령

다음은 motor 동작이나 autonomous driving을 시작하지 않습니다. Image 재빌드와 Container 재검증 시 사용하세요.

```bash
./scripts/docker_run_jetson.sh --headless bash -lc 'uname -m; python3 --version; ros2 pkg list | head'
./scripts/docker_run_jetson.sh --headless python3 -c 'import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available(), torch.cuda.get_device_name(0)); a=torch.randn(256,256,device="cuda:0"); print((a@a).sum().item())'
./scripts/docker_run_jetson.sh --headless python3 -c 'import torch, ultralytics, cv2, numpy, yaml, PyQt5, pygame, Jetson.GPIO; print("imports: PASS")'
./scripts/docker_run_jetson.sh --headless bash -lc 'ls -l /dev/ttyMotor /dev/ttyLidar /dev/video* /dev/gpiochip* /dev/bus/usb 2>/dev/null'
```

`Jetson.GPIO` import는 `/dev/gpiochip0` 접근 권한을 요구할 수 있습니다. `pyrealsense2`는 별도로 `python3 -c "import pyrealsense2"`로 확인하세요. ROS camera node 자체는 `ros-humble-realsense2-camera`로 제공됩니다.

Workspace build 후 다음을 확인합니다.

```bash
source /workspace/install_jetson/setup.bash
colcon list --names-only
ros2 pkg list | grep '^aftr_'
colcon --log-base log_jetson test --build-base build_jetson --install-base install_jetson --parallel-workers 2
colcon test-result --test-result-base build_jetson --verbose
```

2026-09-30 검증에서 AFTR 13개와 `laser_filters`, `serial`, `sllidar_ros2` 등 총 16개 Build가 성공했습니다. `laser_filters`와 `sllidar_ros2`가 stderr 경고를 출력했지만 Build는 완료됐습니다. 2026-10-03 Jetson Test는 16개 package에서 총 97개 중 0 errors, 4 failures, 4 skipped로 끝났습니다. 실패는 모두 AFTR lint/docstring 검사로, `aftr_path_manager` 2개 (flake8, pep257), `aftr_status_led` 1개 (flake8), `aftr_tracking` 1개 (flake8)입니다. 외부 package의 실패는 보고되지 않았습니다. Laptop의 최신 기록은 97개 중 5 failures, 4 skipped이며 `aftr_mode_manager`의 추가 lint/docstring 실패가 포함되어 있습니다. `aftr_description view_robot.launch.py`의 실제 GUI 실행은 별도 확인해야 합니다. Hardware에 영향을 줄 수 있는 bringup 및 주행 Launch는 operator가 통제할 때만 실행하세요.

## Operator GUI 실행

실제 로봇의 기본 실행 명령입니다. `auto_start_base`, `enable_fall_camera`, `enable_fall_detection`의 기본값은 모두 `true`입니다. mode manager가 base bringup (`mdbot.launch.py`)을 시작하고 LiDAR, motor controller, RealSense, CUDA fall detector, GUI, audio, status LED가 함께 기동합니다. GUI의 기본 시스템 준비 표시는 `/scan` 메시지와 필수 controller의 활성화까지 확인한 뒤 바뀝니다. 주행 버튼을 누르기 전에 주변을 비우고 현장에서 로봇을 감독하세요.

```bash
cd ~/260929_ws/src/autonomous-following-towing-robot
./scripts/docker_run_jetson.sh bash -lc 'ros2 launch aftr_gui operator_system.launch.py'
```

화면과 CUDA detector만 확인할 때에는 `auto_start_base:=false`를 명시합니다. 이 옵션은 자동 base 시작만 막으므로 GUI의 수동 명령까지 차단하지는 않습니다.

```bash
./scripts/docker_run_jetson.sh bash -lc 'ros2 launch aftr_gui operator_system.launch.py auto_start_base:=false show_fall_image:=false'
```

2026-10-03 실제 Jetson X11에서 `MDBOT 운영 화면` 창 표시와 mode manager의 IDLE 기동, RealSense, CUDA fall detector의 모델 로드 및 `cuda:0` 선택을 확인했습니다. 검증 중 `mdbot.launch.py`와 `ros2_control_node` 프로세스는 시작되지 않았고 종료 후 검증용 프로세스도 남지 않았습니다. RealSense가 간헐적인 frame timeout/IR stream 경고를 출력했고, 당시 audio node는 ALSA 장치를 열지 못해 SDL dummy 드라이버로 전환했습니다. 이후 Host PulseAudio 연결을 추가해 `aftr_audio`의 `audio_backend_ready=True`와 HDMI speaker의 실제 `system_ready.wav` 재생을 확인했습니다. 추론 노드는 `NO_PERSON`, `NORMAL`, 일시적인 `STALE_IMAGE`를 보고했습니다. 따라서 지속적인 카메라 수신, 전체 operator 흐름의 안내음, 수동 GUI 조작과 robot workflow는 별도 현장 기록이 필요합니다.

## 현재 제한

2026-09-30 실제 Jetson에서 완성된 AFTR image가 드라이버 560 요구 경고 없이 시작했습니다. Python 3.10.12, `ROS_DISTRO=humble`, PyTorch 2.8.0, `torch.version.cuda=12.6`, `torch.cuda.is_available()=True`, GPU 이름 `Orin`, 실제 CUDA tensor matmul을 확인했습니다. 주요 Python import (`torch`, `torchvision`, `ultralytics`, `cv2`, `numpy`, `yaml`, `PyQt5`, `pygame`, `Jetson.GPIO`, `cv_bridge`)와 AFTR package 13개 discovery가 통과했습니다. `/dev/ttyMotor`, `/dev/ttyLidar`, `/dev/video0`, `/dev/gpiochip0`, `/dev/nvmap`, `/dev/nvhost-gpu`의 Container 내부 표시를 확인했습니다. 장치 표시와 import만으로 기능을 입증할 수 없으므로 실제 영상·LiDAR·CUDA 추론·X11·HDMI 안내음을 별도로 확인했습니다. 운영자는 기본 operator 실행 후 전체 동작이 정상이라고 확인했지만 Motor controller, GPIO 출력과 전체 주행 흐름의 제출용 로그는 별도로 정리해야 합니다. Jetson Test의 lint/docstring 실패 4개는 [검증 현황](../testing.md)에 기록했습니다.

## 다음 작업

- [x] Jetson Container에서 `colcon test`와 `colcon test-result`를 실행했습니다 (2026-10-03, 97개 중 4 failures, 4 skipped). 실패는 모두 AFTR lint/docstring 검사이며, 외부 저장소 세 곳의 기존 Git 상태는 유지됐습니다.
- [ ] AFTR의 기존 lint/docstring 실패 4개를 별도 정리할지 결정하고, 수정한다면 해당 package를 다시 Test합니다. `aftr_mode_manager`는 이번 Jetson 실행에서 통과했습니다.
- [x] 2026-10-03 RealSense D435i에서 ROS color 1280×720 `rgb8`와 depth 848×480 `16UC1` 이미지 메시지를 수신했습니다. SLLidar의 `/scan`에서 `laser` frame, 1,800개 거리값을 가진 메시지 3개를 수신했고 각 메시지에 유효 거리값이 1,689~1,767개 있었습니다. 두 센서 노드는 검증 후 종료했습니다. `/dev/ttyMotor`, `/dev/ttyLidar` alias도 확인했습니다. 모터 명령이나 자동 주행은 operator가 통제하는 별도 시험에서만 실행합니다.
- [x] 2026-10-03 기존 local 모델을 Host의 `models/fall_detection/yolov8n-pose.pt`에 복사하고 원본과 SHA-256 (`c6fa93dd1ee4a2c18c900a45c1d864a1c6f7aba75d84f91648a30b7fb641d212`) 일치를 확인했습니다. Container의 `/models`에서 모델을 읽어 실제 RealSense color 프레임 (1280×720)에 YOLO pose 추론을 실행했습니다. PyTorch CUDA 12.6, Orin, predictor `cuda:0`, pose 결과를 확인했습니다. 해당 프레임의 사람 검출은 0명이므로 낙상 판정 정확도는 아직 검증되지 않았습니다. 모델 파일은 Git에 추가하지 않습니다.
- [ ] 현장 감독하에 기본 operator 명령을 실행하고 `/scan` 수신, `joint_state_broadcaster`와 `diff_drive_controller` 활성화, GUI의 기본 시스템 준비 상태를 확인합니다. 자동 주행은 별도로 검증합니다.
- [x] X11의 operator GUI 창 표시와 Host HDMI speaker의 `system_ready.wav` 시험 재생을 확인했습니다.
- [ ] 실제 GPIO LED 출력, GUI 버튼 조작, 전체 operator 실행의 audio event를 각각 확인하고 결과를 이 문서에 기록합니다.
- [ ] 마무리 전에 AFTR의 Git 상태와 외부 저장소 세 곳의 HEAD 및 작업 트리 상태를 확인합니다.
