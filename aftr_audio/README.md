# aftr_audio

`aftr_audio`는 Mode Manager가 전달하는 이벤트를 안내음과 BGM으로 출력하는 ROS 2 package입니다.

## 역할

- `/mode_manager/audio_event` 구독
- 반복 BGM 및 일회성 SFX 재생
- SFX queue 및 cooldown 관리
- 동일 이벤트의 과도한 반복 재생 방지

이 package는 Robot Workflow를 직접 결정하지 않습니다. 작업 흐름과 안전 상태는 `aftr_mode_manager`가 관리합니다.

## Topic

```text
/mode_manager/audio_event
```

Message type:

```text
std_msgs/msg/String
```

## 주요 Event

- `system_ready`
- `follow_start`
- `recording_follow_start`
- `worker_detected`
- `worker_lost`
- `alignment_request`
- `alignment_completed`
- `autonomous_ready`
- `autonomous_bgm_start`
- `autonomous_bgm_stop`
- `arrived`
- `blocked`
- `failed`
- `return_to_home`

실제 event-to-audio mapping은 `config/audio_events.yaml`에서 관리합니다.

## 구조

```text
aftr_audio/
├── aftr_audio/
│   ├── audio_node.py
│   ├── audio_policy.py
│   ├── bgm_player.py
│   └── sfx_player.py
├── config/
│   └── audio_events.yaml
├── sounds/
└── launch/
    └── audio.launch.py
```

## 주요 Parameter

- `enabled`
- `config_file`
- `sounds_dir`
- `bgm_volume_scale`
- `sfx_volume_scale`

## 실행

전체 시스템에서는 Operator launch와 함께 사용합니다.

개별 package를 실행해야 하는 경우 Workspace build와 overlay source 이후:

```bash
ros2 launch aftr_audio audio.launch.py
```

설치 환경은 [설치 및 실행](../docs/setup.md), 테스트 및 개발자용 명령은 [개발 및 유지보수](../docs/maintenance.md)를 참고하세요.

## 동작 원칙

- `autonomous_moving_bgm.wav`는 반복 BGM으로 사용합니다.
- `arrived`, `failed`와 같은 종료 이벤트는 필요한 경우 현재 BGM을 정지한 뒤 재생합니다.
- `blocked` 등 반복될 수 있는 이벤트에는 cooldown을 적용합니다.
- Audio backend 오류가 Robot Workflow 자체를 막지 않도록 구성합니다.
