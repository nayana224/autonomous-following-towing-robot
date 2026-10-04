# 안전 정책

이 문서는 실제 로봇 운용 시 적용되는 안전 정지 정책을 설명합니다.

## 기본 원칙

- 낙상이 확인되면 Safety Stop이 latch됩니다.
- detector가 이후 `False`를 보내더라도 자동으로 latch를 해제하지 않습니다.
- 안전 정지는 운영자의 명시적인 해제 요청이 필요합니다.
- 해제 후 이전 작업을 자동으로 재개하지 않습니다.
- detector heartbeat가 유실된 상태에서는 새로운 이동 작업을 시작하지 않습니다.

## Safety Stop 발생 조건

다음 상황에서 안전 정지 또는 이동 제한이 발생할 수 있습니다.

- 낙상 감지
- Fall Detector heartbeat 손실
- Camera frame timeout으로 인한 detector 상태 상실
- 안전에 필요한 runtime process 종료 실패

## Safety Stop 처리

```text
낙상 / Detector 장애
        ↓
Safety Latch
        ↓
현재 이동 Workflow 정지
        ↓
Operator Safety 화면
        ↓
현장 안전 확인
        ↓
clear_safety_stop
        ↓
Detector reset
        ↓
HOME / IDLE
```

## 안전 정지 해제 조건

다음 조건에서는 해제가 거부될 수 있습니다.

- Safety cleanup이 아직 진행 중인 경우
- Detector heartbeat가 없는 경우
- Detector가 계속 `FALL_DETECTED` 상태인 경우

GUI의 버튼 활성 여부는 사용자 안내용이며 최종 검증은 `aftr_mode_manager`가 수행합니다.

## Cleanup 우선순위

Safety cleanup에서는 로봇 이동을 발생시킬 수 있는 process를 우선 정지합니다.

대표 대상:

- Tracking
- Path Manager
- Nav2
- SLAM

필수 process를 안전하게 종료하지 못하면 Safety Stop 상태를 유지합니다.

## GUI 표시

Safety 화면에는 다음 정보를 표시할 수 있습니다.

- Camera image
- Cleanup 진행 상태
- 중단된 mode
- Detector 연결 상태
- 현재 detector observation

안전 정지 해제는 확인 대화상자를 거쳐 수행합니다.

## 운용 주의사항

Safety 기능은 소프트웨어 안전 계층입니다. 실제 로봇 운용 시에는 다음 사항을 함께 지켜야 합니다.

- 로봇 주변 작업 공간 확보
- 즉시 정지 가능한 운영자 배치
- 견인물과 연결부 상태 확인
- 초기 저속 시험 후 정상 운용
- 예상하지 못한 이동 시 즉시 운행 중단
