# Autonomous Following and Towing Robot

ROS 2 Humble 기반의 작업자 추종 및 견인 로봇 프로젝트입니다. 작업자 추종, 경로 기록·재생, SLAM, Nav2 자율주행, Fall Detection 및 Safety Monitoring을 하나의 운용 흐름으로 통합합니다.

<p align="center">
  <img src="images/robot_prototype.png" alt="Autonomous Following and Towing Robot prototype" width="900">
</p>

## Overview

Autonomous Following and Towing Robot은 작업자를 추종하면서 이동 경로를 기록하고, 저장된 경로와 지도를 기반으로 위치 추정 및 자율 경로 재생을 수행하는 이동형 견인 로봇입니다.

Operator GUI에서 추종·경로 저장·자율주행 모드를 제어하며, LiDAR와 Depth Camera를 이용한 주변 인식, ros2_control 기반 구동계 제어, 낙상 감지 및 safety stop 절차를 함께 제공합니다.

## Project Information

| Item | Description |
| --- | --- |
| Project | Autonomous Following and Towing Robot |
| Program | 한이음 드림업 |
| Platform | ROS 2 Humble |
| Target Hardware | NVIDIA Jetson Orin Nano |
| Development | Linux amd64 Laptop / Jetson arm64 + Docker |

## System Architecture

로봇의 주요 센서, 연산 장치, 사용자 인터페이스 및 구동계는 다음과 같이 구성됩니다.

<p align="center">
  <img src="images/system_architecture.png" alt="AFTR system architecture" width="950">
</p>

- **LiDAR**: RPLIDAR S2 기반 거리 측정 및 작업자 추종·SLAM 입력
- **Depth Camera**: Intel RealSense D435i 기반 RGB-D 및 IMU 입력
- **Compute**: ROS 2 기반 perception, mode management, navigation 및 hardware control 실행
- **Operator GUI**: 추종·경로 저장·자율주행 모드 선택 및 상태 모니터링
- **Motor Driver**: MD400T 2채널 BLDC motor driver
- **Drive System**: BLDC in-wheel motor 2개 기반 differential drive

## Operation Workflow

전체 운용 과정은 **초기 설정 → 작업자 추종 및 경로 저장 → 저장 경로 재현 및 자율주행**의 세 단계로 구성됩니다.

<p align="center">
  <img src="images/operation_workflow.png" alt="AFTR operation workflow" width="1200">
</p>

### 1. 초기 설정

로봇과 Operator PC의 연결 상태, 센서 및 카메라 동작 상태를 확인한 뒤 사용할 운용 모드를 선택합니다.

### 2. 추종 및 경로 저장

LiDAR 기반 작업자 탐지 및 추종을 수행하면서 로봇의 위치와 이동 경로를 실시간으로 기록합니다. 작업 종료 시 이후 재현에 사용할 경로 데이터를 저장합니다.

### 3. 경로 재현 및 자율주행

저장된 경로와 지도를 기반으로 localization을 수행하고, Nav2와 path replay workflow를 이용해 이전 경로를 재현합니다.

## Operator GUI

Operator GUI는 로봇의 상태 확인과 작업 흐름 제어를 담당합니다.

<p align="center">
  <img src="images/operator_gui.png" alt="AFTR operator GUI" width="1200">
</p>

주요 기능은 다음과 같습니다.

- 작업자 추종 / 자율주행 모드 선택
- 경로 저장 여부 설정
- Robot mode 및 속도 상태 표시
- 저장 경로 기반 정방향·복귀 주행 제어
- Navigation map 및 현재 위치 모니터링
- 운행 중단 및 workflow 복귀

## Mechanical Design

로봇은 작업 카트와 연결하여 견인할 수 있도록 프레임과 연결 구조를 구성했으며, 운용 편의성을 위해 조절식 GUI mount와 status LED를 적용했습니다.

<p align="center">
  <img src="images/robot_components.png" alt="AFTR mechanical components" width="850">
</p>

### Steering / Latch Mechanism

카트 연결부에는 기계적 결합을 위한 latch 구조를 적용했습니다.

<p align="center">
  <img src="images/steering_latch_mechanism.png" alt="AFTR steering latch mechanism" width="850">
</p>

## Features

- Worker Following 및 Path Recording / Replay
- Nav2 기반 Navigation 및 Localization
- SLAM Toolbox 기반 Mapping
- Fall Detection 및 Safety Monitoring
- Operator GUI, audio 및 status LED feedback
- ros2_control 기반 BLDC motor control
- LiDAR 및 RealSense sensor integration
- Laptop / Jetson 분리 Docker development environment

## Team

| 이름 | 담당 분야 | 담당 영역 |
| --- | --- | --- |
| 박재범 | Path Planning 및 Autonomous Navigation Support | `aftr_status_led`, `aftr_path_manager` |
| 이인표 | Firmware 및 System Integration | `aftr_audio`, `aftr_bringup`, `aftr_description`, `aftr_gui`, `aftr_hardware`, `aftr_mode_manager`, `aftr_navigation`, `aftr_slam`, `aftr_teleop` |
| 이재빈 | Mechanical Design | 기구부 설계 및 CAD |
| 이하나 (팀장) | Leg Tracking 및 Fall Detection | `aftr_tracking`, `aftr_fall_detection` |

## Packages

| Package | 역할 |
| --- | --- |
| `aftr_audio` | Operator audio feedback |
| `aftr_bringup` | Robot base, controller, LiDAR bringup |
| `aftr_description` | URDF 및 robot model |
| `aftr_fall_detection` | 낙상 관찰 및 heartbeat |
| `aftr_gui` | Operator GUI |
| `aftr_hardware` | ros2_control hardware interface |
| `aftr_mode_manager` | 작업 흐름 및 안전 상태 관리 |
| `aftr_navigation` | Nav2 및 localization 설정 |
| `aftr_path_manager` | 경로 기록·재생 및 pose 저장 |
| `aftr_slam` | SLAM Toolbox 설정 |
| `aftr_status_led` | Status LED feedback |
| `aftr_teleop` | 수동 `/cmd_vel` 입력 |
| `aftr_tracking` | 작업자 추종 |

## Quick Start

Laptop (Linux amd64) Host에서 Docker image를 빌드하고 Container shell을 엽니다.

```bash
cd ~/autonomous_following_towing_robot_ws/src/autonomous-following-towing-robot
./scripts/docker_build_laptop.sh
./scripts/docker_run_laptop.sh
```

Container의 `/workspace`에서 Workspace를 빌드합니다. 첫 빌드 직후 같은 shell에서 overlay를 source합니다.

```bash
./src/autonomous-following-towing-robot/scripts/build_ws.sh
source /workspace/install_laptop/setup.bash
```

실행 script는 Host Workspace의 `models/`를 Container `/models`에 읽기 전용으로, `data/`를 `/data`에 읽기·쓰기용으로 연결합니다. 모델은 `models/fall_detection/yolov8n-pose.pt`에 준비하세요. 상세 경로와 override는 [Laptop Docker Development](docs/development/laptop-docker.md)를 참고하세요.

Jetson Orin Nano Super (arm64)에서는 다음 명령을 사용합니다.

```bash
cd ~/260929_ws/src/autonomous-following-towing-robot
./scripts/docker_build_jetson.sh
./scripts/docker_run_jetson.sh

# Container 안에서
./src/autonomous-following-towing-robot/scripts/build_ws.sh
source /workspace/install_jetson/setup.bash
```

Jetson image는 CUDA PyTorch와 ROS 2 Humble을 사용합니다. 동일한 `/models:ro`, `/data:rw` 경로를 사용하며, `build_jetson/install_jetson/log_jetson`은 Laptop 산출물과 분리합니다. 실제 로봇 시연은 [시연·운영 안내](docs/operation.md), 개발 환경과 장치 전달 방법은 [Jetson Docker 안내](docs/development/jetson-docker.md)를 참고하세요.

## Development Environment

### Laptop / Development PC

개발 환경은 Linux amd64 Host에서 Docker를 사용합니다. Container 내부는 Ubuntu 22.04 Jammy, ROS 2 Humble 및 CPU-only PyTorch로 구성됩니다. Host display가 있으면 GUI가 기본으로 연결되고, display가 없거나 `--headless` 옵션을 사용하면 headless로 실행됩니다. **Ubuntu 26 LTS amd64 Host에서** 전체 Workspace Build와 Test를 검증했습니다. 세부 결과는 [Testing](docs/testing.md)에 있습니다.

### Jetson Orin Nano

NVIDIA Jetson Orin Nano Super는 실제 Robot deployment 대상입니다. Ubuntu 22.04, L4T 36.5.2, CUDA 12.6 Host에서 NVIDIA Container Runtime을 사용합니다. Jetson에서 CUDA 추론, Workspace package Build, RealSense 영상, LiDAR scan, X11 GUI 표시와 HDMI 안내음 재생을 확인했습니다. 세부 검증 상태는 [Testing](docs/testing.md)을 참고하세요.

## Workspace Layout

```text
~/autonomous_following_towing_robot_ws/
├── src/
│   ├── autonomous-following-towing-robot/
│   ├── laser_filters/
│   ├── serial-ros2/
│   └── sllidar_ros2/
├── build_laptop/             # amd64 전용
├── install_laptop/
├── log_laptop/
├── build_jetson/             # arm64 전용
├── install_jetson/
└── log_jetson/
```

세 external sibling은 독립 Git repository입니다. `serial-ros2`의 ROS Package 이름은 `serial`이며, Laptop 산출물은 Jetson arm64 산출물과 공유하지 않습니다.

## Documentation

- 대회 시연: [시연·운영 안내](docs/operation.md), [검증 현황](docs/testing.md)
- 시스템 설계: [아키텍처](docs/architecture.md), [ROS 인터페이스](docs/interfaces.md), [안전 정책](docs/safety.md)
- 개발 환경: [Jetson Docker](docs/development/jetson-docker.md), [Laptop Docker](docs/development/laptop-docker.md)
- 근거 자료: [하드웨어 매뉴얼과 발표 자료](docs/references/README.md)
- 협업 규칙: [AGENTS.md](AGENTS.md)
