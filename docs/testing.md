# 검증 현황

대회 시연에 필요한 재현 가능한 결과와 아직 확인할 항목을 구분해 기록합니다. 수치는 기록된 실행 시점의 결과이며 새 변경 후에는 다시 검증해야 합니다.

## 확인된 결과

| 환경 | 확인 내용 | 결과 |
| --- | --- | --- |
| Jetson Orin Nano Super, L4T R36.5.2 | Docker 시작, ROS 2 Humble, Python 3.10, CUDA 12.6, Orin GPU, CUDA tensor 연산 | 통과. Driver 560 요구 경고 없음 |
| Jetson Docker | 13개 `aftr_*`와 외부 package 3개 Build | 16개 완료 |
| Jetson Docker | RealSense D435i color/depth 영상과 SLLidar `/scan` | 실제 메시지 수신 |
| Jetson Docker | `yolov8n-pose.pt`와 실제 카메라 프레임의 YOLO pose 추론 | predictor `cuda:0`에서 통과. 해당 프레임의 사람 검출은 0명 |
| Jetson X11·HDMI | Operator GUI 창, `aftr_audio` PulseAudio 초기화, `system_ready.wav` | 화면 표시 및 실제 안내음 청취 확인 |
| Jetson `colcon test` (2026-10-03) | 전체 97개 | 89 passed, 4 failed, 4 skipped, 0 errors |
| Laptop CPU Docker | 16개 Build, 전체 97개 Test | Build 통과; Test는 5 failed, 4 skipped, 0 errors |

Jetson Test의 실패 4개는 `aftr_path_manager`의 flake8·pep257, `aftr_status_led`의 flake8, `aftr_tracking`의 flake8입니다. 모두 AFTR lint/docstring 검사이며 외부 package의 Test 실패는 보고되지 않았습니다. Laptop의 최신 5개 실패에는 위 항목에 더해 `aftr_mode_manager` 검사 1개가 포함됩니다. Test 전체를 통과했다고 표기하지 않습니다.

운영자는 2026-10-03 기본 operator 실행 후 전체 동작이 정상이라고 확인했습니다. 본 문서의 표에는 별도로 재현 가능한 출력과 직접 확인된 항목만 통과로 적었습니다. 주행·GPIO·장시간 안정성의 제출용 로그는 아래 현장 항목으로 수집합니다.

Jetson 컨테이너 안의 `/workspace`에서 재실행:

```bash
source /opt/ros/humble/setup.bash
source /workspace/install_jetson/setup.bash
colcon --log-base log_jetson test --build-base build_jetson --install-base install_jetson --parallel-workers 2
colcon test-result --test-result-base build_jetson --verbose
```

Laptop에서는 `jetson`을 `laptop`으로 바꾼 별도 Build/Test 디렉터리를 사용합니다. 상세 실행 방법은 [Jetson Docker](development/jetson-docker.md)와 [Laptop Docker](development/laptop-docker.md)에 있습니다.

## 현장 시연에서 확인할 항목

- Base bringup의 `/scan`, `/odom`, `joint_state_broadcaster`, `diff_drive_controller` 준비와 비상 정지 절차
- GUI의 추종·기록·정렬·위치 추정·경로 재생 흐름, 중간 정지와 재시작
- 낙상 감지, detector heartbeat 상실, 안전 래치와 해제 조건
- 실제 GPIO LED 변화와 전체 작업의 안내음 재생
- RealSense 장시간 수신 안정성: 짧은 operator 실행에서 IR stream/frame timeout 경고가 관찰됨

작업 중 `ERROR` 또는 비정상 프로세스 종료가 발생하면 ROS 로그와 `/mode_manager/status`를 함께 기록합니다. 현장 검증 방법은 [시연·운영 안내](operation.md)와 [안전 정책](safety.md)을 참고하세요.
