# 시스템 구조

Autonomous Following and Towing Robot은 ROS 2 Humble 기반으로 작업자 추종, 경로 기록, Mapping, Localization 및 저장 경로 재생을 수행합니다.

## 전체 구조

```text
Operator GUI
     │
     ▼
Mode Manager
     │
     ├── Tracking
     ├── Path Manager
     ├── SLAM
     ├── Navigation / Nav2
     ├── Fall Detection
     └── Audio / Status LED
     │
     ▼
Bringup / ros2_control
     │
     ▼
MD400T Motor Driver
     │
     ▼
BLDC In-Wheel Motors
```

## 패키지 역할

| Package | 역할 |
| --- | --- |
| `aftr_gui` | 운영자 화면 및 사용자 입력 |
| `aftr_mode_manager` | 전체 작업 흐름, 상태 전이, 프로세스 시작·종료 및 안전 제어 |
| `aftr_tracking` | 작업자 탐지 및 추종 |
| `aftr_fall_detection` | 낙상 상태와 detector heartbeat 제공 |
| `aftr_path_manager` | 경로 기록·저장·재생 및 pose 관리 |
| `aftr_slam` | SLAM Toolbox 기반 Mapping |
| `aftr_navigation` | Nav2 및 Localization |
| `aftr_bringup` | Controller, LiDAR 및 base bringup |
| `aftr_hardware` | MD400T ros2_control hardware interface |
| `aftr_audio` | Operator audio feedback |
| `aftr_status_led` | 로봇 상태 LED |
| `aftr_teleop` | 수동 속도 명령 |
| `aftr_description` | URDF/Xacro 및 robot model |

## 기본 Workflow

```text
HOME
  ↓
FOLLOW / RECORDING_FOLLOW
  ↓
ALIGNMENT_DECISION
  ↓
ALIGNMENT 또는 LOCALIZING
  ↓
AUTONOMOUS_READY
  ↓
AUTONOMOUS_DRIVING
```

### 경로 저장 추종

```text
Base 준비
→ SLAM 시작
→ 새 /map 확인
→ Path Manager 준비
→ 경로 기록 시작
→ 작업자 추종 시작
```

### 위치 추정 준비

```text
기록 종료
→ 경로 / 지도 / pose 저장
→ SLAM 종료 확인
→ Nav2 시작
→ Initial Pose 전달
→ AMCL Pose 확인
```

## GUI와 Mode Manager 역할

GUI는 현재 상태를 표시하고 허용된 command를 전송합니다. Workflow 정책과 안전 판단은 `aftr_mode_manager`가 담당합니다.

GUI는 `/mode_manager/status`의 `allowed_commands`를 기준으로 버튼을 활성화합니다.

## 구동 명령 소유권

`/cmd_vel`은 한 시점에 하나의 제어 주체만 사용하도록 운용합니다.

- 작업자 추종: `aftr_tracking`
- 수동 정렬: GUI / teleop 계열
- 자율주행: Nav2

## 외부 패키지

다음 패키지는 Workspace의 별도 Git repository로 관리합니다.

- `laser_filters`
- `serial-ros2` (`serial`)
- `sllidar_ros2`
