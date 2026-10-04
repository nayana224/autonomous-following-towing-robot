# aftr_tracking

`aftr_tracking`은 LiDAR `/scan`에서 작업자 후보를 탐지하고 선택된 작업자를 추종하는 ROS 2 package입니다.

## 기본 동작

기본 tracker는 `aftr_tracking/tracker_node.py`를 사용합니다.

- 초기 탐색 영역: 전방 약 ±50°
- 추종 중 탐색 영역: 약 ±80°
- 가까운 작업자 후보를 추종 대상으로 선택
- 일시적인 가림 또는 후보 손실 시 짧은 시간 동안 이전 target 유지
- 작업자와의 목표 거리를 유지하도록 `/cmd_vel` 생성
- 작업자와 로봇 사이 장애물 조건을 함께 확인

현재 field-tested 기본 tracker의 동작을 기준으로 운용합니다.

## Public Interface

### 입력

```text
/scan
```

### 주요 출력

```text
/cmd_vel
/follow_state
```

`/follow_state`의 대표 상태:

- `SEARCH`
- `FOLLOW`
- `OBSTACLE`

Optional experimental tracker는 추가 상태 정보를 `/tracking_detail`로 제공할 수 있지만, 일반 Workflow는 `/follow_state`를 기준으로 동작합니다.

## 전체 시스템에서 실행

Tracking process는 Robot이 IDLE일 때 항상 실행되는 것이 아니라 Operator가 추종 Workflow를 선택할 때 Mode Manager가 시작합니다.

전체 시스템 실행 방법은 [설치 및 실행](../docs/setup.md)과 [운용 안내](../docs/operation.md)를 참고하세요.

## Experimental Tracker

비교용 experimental tracker를 사용할 경우 Operator launch의 `robust_tracking` option을 사용할 수 있습니다.

```bash
ros2 launch aftr_gui operator_system.launch.py robust_tracking:=true
```

일반 현장 운용에서는 기본 tracker 사용을 권장합니다.

## 운용 주의사항

실제 추종 시험 전에는 다음을 확인합니다.

- 작업자와 로봇 사이에 충분한 공간 확보
- 비상 정지가 가능한 운영자 배치
- 초기 저속 환경에서 추종 방향 확인
- 작업자 손실 및 장애물 발생 시 정지 동작 확인

테스트, lint 및 개발자용 명령은 [개발 및 유지보수](../docs/maintenance.md)를 참고하세요.
