# aftr_mode_manager

`aftr_mode_manager`는 AFTR의 전체 운용 Workflow와 안전 상태를 관리하는 핵심 ROS 2 package입니다.

## 역할

- Base, SLAM, Path Manager, Tracking, Nav2 등 runtime process의 시작·종료 순서 관리
- Operator command service 제공
- `/mode_manager/status` 발행
- 동시에 여러 command가 실행되지 않도록 직렬화
- 부분적으로 시작된 Workflow 실패 시 rollback
- 낙상 및 detector 장애 시 Safety Stop 처리

## 기본 Workflow

```text
IDLE
→ FOLLOW / RECORDING_FOLLOW
→ ALIGNMENT
→ LOCALIZING
→ AUTONOMOUS_READY
→ AUTONOMOUS_DRIVING
```

### 경로 저장 추종 시작

```text
Base 준비
→ SLAM 준비
→ 새 /map 확인
→ Path Manager 준비
→ 경로 기록 시작
→ Follower 시작
```

### Localization 시작

```text
SLAM 종료
→ 종료 상태 확인
→ Nav2 시작
→ Lifecycle 준비 확인
→ Initial Pose 전달
→ AMCL Pose 확인
```

## Public Command

일반 운용 command는 `/mode_manager/command/*` 아래에 있습니다.

대표 command:

- 작업자 추종
- 경로 저장 추종
- 정렬 시작/완료
- 자율주행 준비
- 저장 경로 정방향·복귀 재생
- 작업 중지
- Error 해제
- Safety Stop 해제

`/mode_manager/internal/*` service는 내부 process 관리용이므로 GUI의 일반 Workflow에서는 직접 호출하지 않습니다.

## Safety

낙상 감지 또는 detector heartbeat 손실 시 Safety Stop이 latch될 수 있습니다.

중단된 작업은 자동으로 재개하지 않으며, 운영자의 안전 확인 후 명시적인 해제 절차를 거쳐 IDLE로 복귀합니다.

## 관련 문서

- [시스템 구조](../docs/architecture.md)
- [ROS 인터페이스](../docs/interfaces.md)
- [안전 정책](../docs/safety.md)
- [설치 및 실행](../docs/setup.md)
- [개발 및 유지보수](../docs/maintenance.md)
