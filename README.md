# Autonomous Following and Towing Robot

ROS 2 Humble 기반의 작업자 추종 및 견인 로봇 프로젝트입니다. 경로 기록·재생, Navigation, Fall Detection과 Safety Monitoring을 통합합니다.

<p align="center">
  <img src="images/README/robot.png" alt="Autonomous Following and Towing Robot" width="379">
</p>

## Overview

작업자 추종부터 경로 기록, 정렬, 위치 추정, 자율 경로 재생까지 이어지는 작업 흐름을 제공합니다. Operator GUI에서 작업을 제어하고 낙상 감지나 detector 연결 상실 시 안전 정지 절차를 수행합니다.

## Project Information

| Item | Description |
| --- | --- |
| Project | Autonomous Following and Towing Robot |
| Program | 한이음 드림업 |
| Platform | ROS 2 Humble |
| Target Hardware | NVIDIA Jetson Orin Nano |
| Development | Linux amd64 Laptop / Jetson arm64 + Docker |

## Team

| 이름 | 담당 분야 | 담당 영역 |
| --- | --- | --- |
| 박재범 | Path Planning 및 Autonomous Navigation Support | `aftr_status_led`, `aftr_path_manager` |
| 이인표 | Firmware 및 System Integration | `aftr_audio`, `aftr_bringup`, `aftr_description`, `aftr_gui`, `aftr_hardware`, `aftr_mode_manager`, `aftr_navigation`, `aftr_slam`, `aftr_teleop` |
| 이재빈 | Mechanical Design | 기구부 설계 및 CAD |
| 이하나 (팀장) | Leg Tracking 및 Fall Detection | `aftr_tracking`, `aftr_fall_detection` |

## Features

- Worker Following 및 Path Recording/Replay
- Nav2 Navigation과 SLAM
- Fall Detection 및 Safety Monitoring
- Operator GUI, audio 및 status LED
- ros2_control 기반 motor, LiDAR, RealSense 통합

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

NVIDIA Jetson Orin Nano Super는 실제 Robot deployment 대상입니다. Ubuntu 22.04, L4T 36.5.2, CUDA 12.6 Host에서 NVIDIA Container Runtime을 사용합니다. Jetson에서 CUDA 추론, 16개 Workspace package Build, RealSense 영상, LiDAR scan, X11 GUI 표시와 HDMI 안내음 재생을 확인했습니다. 운영자는 전체 시스템이 현장에서 정상 동작한다고 확인했습니다. 제출용 구동 로그와 장시간 안정성 기록은 [검증 현황](docs/testing.md)의 남은 항목으로 관리합니다.

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
