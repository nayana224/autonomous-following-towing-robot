# 한이음 드림업 시연·운영 안내

이 문서는 Jetson Orin Nano Super에서 작업자 추종, 경로 기록, 위치 추정, 경로 재생, 낙상 안전 정지를 시연하는 순서입니다. 시스템 구조는 [아키텍처](architecture.md), 안전 동작의 상세 조건은 [안전 정책](safety.md)을 참고하세요.

## 시작 전 확인

- 로봇 주변을 비우고 정지·전원 차단을 담당할 운영자를 현장에 둡니다.
- Host에 `/dev/ttyMotor`, `/dev/ttyLidar`, RealSense D435i, HDMI 출력 장치가 연결돼 있는지 확인합니다.
- Workspace의 `models/fall_detection/yolov8n-pose.pt`를 준비합니다. 컨테이너에서는 `/models/fall_detection/yolov8n-pose.pt`로 읽습니다.
- Jetson Docker image와 16개 package가 빌드돼 있어야 합니다. 처음 준비할 때는 [Jetson Docker 안내](development/jetson-docker.md)를 따릅니다.

## 전체 시스템 실행

Host에서 다음 명령을 실행합니다. `auto_start_base`, `enable_fall_camera`, `enable_fall_detection`의 기본값은 모두 `true`입니다. `mode_manager`가 base bringup을 시작하므로 motor controller와 LiDAR도 기동합니다.

```bash
cd ~/260929_ws/src/autonomous-following-towing-robot
./scripts/docker_run_jetson.sh bash -lc 'ros2 launch aftr_gui operator_system.launch.py'
```

GUI에서 기본 시스템 준비 표시를 확인합니다. 준비 판정에는 `/scan` 메시지와 필수 controller의 활성화가 포함됩니다. 준비되지 않으면 launch 로그의 `missing topics`와 controller 오류를 먼저 확인합니다. 창을 닫으면 operator launch가 종료됩니다.

## 시연 흐름

1. **작업자 추종:** `경로 저장`을 끈 상태에서 `작업자 추종`을 누르고, 정지 버튼으로 종료합니다.
2. **경로 기록:** `경로 저장`을 켜고 추종을 시작합니다. SLAM의 새 `/map`과 경로 기록 상태를 확인한 뒤 기록을 종료합니다. 이때 경로·지도·pose 저장이 완료돼야 합니다.
3. **정렬과 위치 추정:** 화면의 조이스틱으로 정렬하고 손을 떼면 입력이 중립으로 돌아오는지 확인합니다. `정렬 완료` 후 SLAM이 멈추고 Nav2/AMCL 준비가 완료되는지 확인합니다.
4. **경로 재생:** `자율주행 준비 완료` 화면에서 허용된 이동 방향을 선택합니다. 주행 중 정지 버튼이 동작하는지 현장 감독하에 확인합니다.
5. **안전 정지:** 낙상 또는 detector 장애 시 활성 작업이 정지하고 안전 화면이 표시되는지 통제된 시험에서 확인합니다. 현장을 확인한 뒤 `안전 정지 해제`와 확인 대화상자를 사용합니다. 이전 작업은 자동으로 재개되지 않습니다.

상태 전이, ROS topic/service 이름과 데이터 경로는 [아키텍처](architecture.md), [ROS 인터페이스](interfaces.md)에 기록돼 있습니다. 시연 결과와 미검증 항목은 [검증 현황](testing.md)에 따로 남깁니다.

## 시연 기록

날짜, Jetson image/commit, 실행 명령, GUI 화면, `/mode_manager/status`, `/scan`, controller 상태와 실패 로그를 함께 보관합니다. 구동이 예상과 다르면 주행을 중단하고 원인을 확인한 뒤 다시 시작합니다.
