# ROS 인터페이스

이 문서는 AFTR package 간에 사용하는 주요 ROS 2 인터페이스를 정리합니다.

## 주요 Topic

| Topic | 설명 |
| --- | --- |
| `/cmd_vel` | 로봇 속도 명령 |
| `/odom` | Odometry |
| `/scan` | Filtered LiDAR scan |
| `/map` | SLAM / Localization map |
| `/amcl_pose` | AMCL 위치 추정 결과 |
| `/planned_path` | 경로 시각화 |
| `/follow_state` | 작업자 추종 상태 |
| `/tracking_detail` | Tracking 상세 상태 |
| `/mode_manager/status` | GUI용 Workflow 및 Safety 상태 |
| `/mode_manager/audio_event` | Audio event |
| `/mode_manager/led_event` | LED event |
| `/fall_detection/status` | Fall detector 상태 |
| `/fall_detection/detected` | 낙상 감지 상태 |
| `/fall_detection/heartbeat` | Detector heartbeat |

`/follow_state`의 대표 상태는 `SEARCH`, `FOLLOW`, `OBSTACLE`입니다.

## Mode Manager Command

주요 service는 다음과 같습니다.

```text
/mode_manager/command/start_follow
/mode_manager/command/start_recording_follow
/mode_manager/command/finish_recording_for_alignment
/mode_manager/command/start_alignment
/mode_manager/command/skip_alignment
/mode_manager/command/finish_alignment
/mode_manager/command/prepare_autonomous
/mode_manager/command/path_forward_auto
/mode_manager/command/path_reverse_auto
/mode_manager/command/stop
/mode_manager/command/clear_error
/mode_manager/command/clear_safety_stop
```

GUI는 일반적으로 현재 `/mode_manager/status`의 `allowed_commands`에 포함된 command만 요청합니다.

## Fall Detection Service

```text
/fall_detection/reset
```

안전 정지 해제 조건이 충족된 뒤 mode manager가 detector latch를 초기화할 때 사용합니다.

## Mode Manager Status

`/mode_manager/status`는 JSON text payload를 사용합니다.

주요 field:

- `mode`
- `busy`
- `last_command`
- `last_error`
- `operator_step`
- `operator_message`
- `allowed_commands`
- `manual_control_allowed`
- `tracking_message`
- `recording_message`
- `path_message`
- `driving_message`
- `safety_stop_active`
- `safety_stop_in_progress`
- `fall_detector_alive`
- `fall_detection_status`

## Path Manager

Path Manager의 경로 기록·재생 service는 일반 운용에서 GUI가 직접 호출하지 않고 `aftr_mode_manager`가 순서를 관리합니다.

대표 interface:

```text
/path_manager/start_record
/path_manager/stop_record
/path_manager/save_pose
/path_manager/publish_initial_pose
```

## 주요 TF Frame

```text
map
└── odom
    └── base_footprint
        └── base_link
            └── laser
```

실제 TF 연결은 SLAM, Nav2 및 robot_state_publisher의 실행 상태에 따라 구성됩니다.
