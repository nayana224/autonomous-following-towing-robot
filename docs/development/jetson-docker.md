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

사용 가능한 local `DISPLAY`, X11/XWayland socket, 읽을 수 있는 `XAUTHORITY`가 있으면 Qt `xcb`와 Host display를 사용합니다. 없으면 자동으로 headless로 전환합니다. `--headless`는 Qt offscreen 및 SDL dummy audio를 명시합니다. 기본 hardware mode에서는 SDL audio를 dummy로 강제하지 않으며, 존재하는 ALSA 장치를 전달합니다. 실제 GUI rendering 및 speaker playback은 아직 검증되지 않았습니다.

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

2026-09-30 검증에서 AFTR 13개와 `laser_filters`, `serial`, `sllidar_ros2` 등 총 16개 Build가 성공했습니다. `laser_filters`와 `sllidar_ros2`가 stderr 경고를 출력했지만 Build는 완료됐습니다. Laptop에서 과거 97 Test 중 4개 실패, 4개 skip이 있었으며 실패는 보호된 팀 package에 있었습니다. 이번 Jetson Test는 실행하지 않았습니다. `aftr_description view_robot.launch.py`의 실제 GUI 실행도 별도 확인해야 합니다. Hardware에 영향을 줄 수 있는 bringup 및 주행 Launch는 operator가 통제할 때만 실행하세요.

## 현재 제한

2026-09-30 실제 Jetson에서 완성된 AFTR image가 드라이버 560 요구 경고 없이 시작했습니다. Python 3.10.12, `ROS_DISTRO=humble`, PyTorch 2.8.0, `torch.version.cuda=12.6`, `torch.cuda.is_available()=True`, GPU 이름 `Orin`, 실제 CUDA tensor matmul을 확인했습니다. 주요 Python import (`torch`, `torchvision`, `ultralytics`, `cv2`, `numpy`, `yaml`, `PyQt5`, `pygame`, `Jetson.GPIO`, `cv_bridge`)와 AFTR package 13개 discovery가 통과했습니다. `/dev/ttyMotor`, `/dev/ttyLidar`, `/dev/video0`, `/dev/gpiochip0`, `/dev/nvmap`, `/dev/nvhost-gpu`의 Container 내부 표시를 확인했습니다. Device 표시와 Python import는 RealSense 촬영, serial 통신, 실제 GPIO LED 출력, audio 재생 또는 GUI rendering의 동작 검증을 뜻하지 않습니다. Jetson colcon Test는 실행하지 않았습니다.

## 다음 작업 (2026-10-01 이어서)

- [ ] Jetson Container에서 위 `colcon test`와 `colcon test-result`를 실행하고 실패 package 및 로그를 기록합니다. 기존 Laptop 기준의 보호된 외부 package 실패 4건과 비교하되 외부 저장소 세 곳의 파일이나 Git 상태는 변경하지 않습니다.
- [ ] 실제 RealSense 영상과 LiDAR scan 수신을 각각 확인합니다. `/dev/ttyMotor`, `/dev/ttyLidar` alias와 접근 권한을 확인하고, 모터 명령이나 자동 주행은 operator가 통제하는 별도 시험에서만 실행합니다.
- [ ] Host의 `models/fall_detection/yolov8n-pose.pt`를 준비한 뒤 Container의 읽기 권한과 `cuda:0` 추론을 확인합니다. 모델 파일은 Git에 추가하지 않습니다.
- [ ] 현장 장치에서 GPIO LED, GUI 표시, speaker 재생을 각각 확인하고 결과를 이 문서에 기록합니다.
- [ ] 마무리 전에 AFTR의 Git 상태와 외부 저장소 세 곳의 HEAD 및 작업 트리 상태를 확인합니다.
