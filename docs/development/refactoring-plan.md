# 코드 계층화 검토 및 리팩토링 계획

이번 문서는 계획만 기록합니다. 현재 작업에서는 동작 코드의 구조 변경을 진행하지 않습니다.

## 목표와 기준

- ROS package의 책임, 의존 방향, GUI와 mode manager의 역할 경계를 먼저 확인합니다.
- Topic, service, action, node, frame, parameter, plugin ID, 저장 CSV/map/pose 형식과 안전 정지 동작은 유지합니다.
- 외부 저장소 `laser_filters`, `serial-ros2`, `sllidar_ros2`는 수정하지 않습니다.
- 한 번에 한 package 또는 한 책임만 분리하고, 기존 Test와 필요한 Jetson 현장 확인을 통과한 뒤 다음 단계로 넘어갑니다.

## 순서

1. **기준선 고정:** 16개 package 의존 그래프와 공개 ROS 인터페이스를 [아키텍처](../architecture.md), [인터페이스](../interfaces.md)에 대조합니다. 현 Jetson Test 97개 중 lint/docstring 실패 4개와 실제 동작 검증 결과를 별도 기준선으로 보존합니다.
2. **mode manager 경계 검토:** `workflow/manager.py` (약 1,411줄), `mode_manager_node.py` (약 913줄), `runtime/bringup_runtime.py` (약 900줄)의 상태 전이, ROS 입출력, 프로세스 제어, 안전 복구 책임을 표로 분리합니다. 깊은 상속과 mixin 호출 경로를 먼저 추적하고, 순수 상태 계산부터 작은 단위로 추출합니다.
3. **path manager 중복 검토:** `path_manager_node.py` (약 1,806줄)와 두 corner-turn generator (각 약 1,720줄)의 중복·입출력 부작용을 비교합니다. 좌표·경로 계산을 순수 함수로 분리할 후보를 찾되 CSV 경로와 형식은 유지합니다.
4. **GUI와 detector 경계 검토:** `operator_gui.py` (약 631줄)는 화면 표시와 사용자 입력만 담당하는지, ROS bridge 및 안전 상태의 소유권이 mode manager와 겹치지 않는지 확인합니다. `aftr_tracking`과 `aftr_fall_detection`은 하드웨어·안전 영향 때문에 계약 테스트를 갖춘 후 다룹니다.
5. **작은 변경으로 적용:** 우선순위별 별도 PR/commit으로 분리하고 Laptop/Jetson Build, package Test, GUI·센서·안전 시나리오를 다시 실행합니다. 서식 검사 실패는 구조 변경과 섞지 않고 별도 수정합니다.

## 시작 조건

각 단계 전에 현재 ROS 인터페이스 목록, 대표 입력·출력, 실패 시 정지 동작, 현장 재현 방법을 적습니다. 이 조건이 정리되기 전에는 대규모 파일 이동이나 package 이름 변경을 하지 않습니다.
