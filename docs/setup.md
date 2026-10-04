# 설치 및 실행

이 문서는 Autonomous Following and Towing Robot을 Laptop 개발 환경 또는 Jetson Orin Nano 배포 환경에서 준비하고 실행하는 방법을 설명합니다.

## 1. Workspace 구성

저장소는 ROS 2 Workspace의 `src/` 아래에 배치합니다. 다음 외부 패키지도 같은 `src/` 아래에 있어야 합니다.

```text
<workspace>/
├── src/
│   ├── autonomous-following-towing-robot/
│   ├── laser_filters/
│   ├── serial-ros2/
│   └── sllidar_ros2/
├── models/
│   └── fall_detection/
│       └── yolov8n-pose.pt
└── data/
    ├── paths/
    ├── maps/
    └── poses/
```

`serial-ros2` 저장소의 ROS package 이름은 `serial`입니다.

## 2. 모델 및 Runtime Data

Fall Detection 모델 기본 경로:

```text
Host:      <workspace>/models/fall_detection/yolov8n-pose.pt
Container: /models/fall_detection/yolov8n-pose.pt
```

Runtime data는 Host의 `data/`를 Container의 `/data`에 연결합니다.

주요 경로:

```text
/data/paths/recorded_path.csv
/data/paths/safe_path.csv
/data/maps/mdbot_map.yaml
/data/maps/mdbot_map.pgm
/data/poses/last_pose.yaml
```

별도 경로를 사용할 경우:

```bash
AFTR_MODELS_DIR=/absolute/models AFTR_DATA_DIR=/absolute/data ./scripts/docker_run_laptop.sh
```

Jetson도 동일한 환경 변수를 사용할 수 있습니다.

## 3. Laptop 개발 환경

대상은 Linux amd64 Host입니다. Container는 Ubuntu 22.04 Jammy와 ROS 2 Humble을 사용하며 PyTorch는 CPU-only 구성입니다.

### Docker Image 생성

```bash
cd ~/autonomous_following_towing_robot_ws/src/autonomous-following-towing-robot
./scripts/docker_build_laptop.sh
```

### Container 실행

```bash
./scripts/docker_run_laptop.sh
```

GUI가 필요 없으면:

```bash
./scripts/docker_run_laptop.sh --headless
```

### Workspace Build

Container 안에서:

```bash
./src/autonomous-following-towing-robot/scripts/build_ws.sh
source /workspace/install_laptop/setup.bash
```

Laptop build 산출물은 `build_laptop/`, `install_laptop/`, `log_laptop/`에 저장됩니다.

## 4. Jetson Orin Nano 배포 환경

대상은 NVIDIA Jetson Orin Nano 계열입니다. Jetson에서는 NVIDIA Container Runtime을 사용하며 CUDA 추론을 지원합니다.

### Docker Image 생성

```bash
cd ~/260929_ws/src/autonomous-following-towing-robot
./scripts/docker_build_jetson.sh
```

### Container 실행

```bash
./scripts/docker_run_jetson.sh
```

### Workspace Build

Container 안에서:

```bash
./src/autonomous-following-towing-robot/scripts/build_ws.sh
source /workspace/install_jetson/setup.bash
```

Jetson build 산출물은 `build_jetson/`, `install_jetson/`, `log_jetson/`에 저장됩니다.

Laptop의 amd64 build 산출물과 Jetson arm64 build 산출물을 공유하지 마세요.

## 5. Jetson 장치 준비

실제 로봇 실행 전 Host에서 다음 장치를 확인합니다.

- `/dev/ttyMotor`: MD400T Motor Driver
- `/dev/ttyLidar`: LiDAR
- RealSense D435i USB 장치
- 필요 시 `/dev/gpiochip*`
- GUI/Audio 사용 시 Display 및 Audio 장치

Serial alias는 Host udev rule을 사용합니다.

```bash
cd ~/260929_ws/src/autonomous-following-towing-robot
sudo ./scripts/create_udev_rules.sh
```

장치를 다시 연결한 뒤 alias가 생성되었는지 확인합니다.

```bash
ls -l /dev/ttyMotor /dev/ttyLidar
```

## 6. 전체 시스템 실행

실제 로봇에서는 Jetson Host에서 다음 명령을 사용합니다.

```bash
cd ~/260929_ws/src/autonomous-following-towing-robot
./scripts/docker_run_jetson.sh bash -lc   'ros2 launch aftr_gui operator_system.launch.py'
```

주행 전에는 반드시 로봇 주변을 비우고 즉시 정지할 수 있는 운영자가 현장에 있어야 합니다.

GUI와 Fall Detection만 확인하고 base를 자동 기동하지 않으려면:

```bash
./scripts/docker_run_jetson.sh bash -lc   'ros2 launch aftr_gui operator_system.launch.py auto_start_base:=false show_fall_image:=false'
```

## 7. 로봇 모델 확인

Laptop에서 URDF/RViz만 확인하려면:

```bash
./scripts/docker_run_laptop.sh   ros2 launch aftr_description view_robot.launch.py
```

## 8. 다음 단계

설치가 끝났다면 [운용 안내](operation.md)에 따라 작업자 추종, 경로 기록, 위치 추정 및 경로 재생을 진행합니다.

테스트, lint, compile database 및 개발자용 문제 해결 방법은 [개발 및 유지보수](maintenance.md)에 별도로 정리되어 있습니다.
