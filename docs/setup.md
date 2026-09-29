# Setup and Build

프로젝트의 개발 및 실행 환경을 선택하는 출발점입니다. 현재 Laptop Docker 환경과 기존 Jetson 수동 설정은 서로 다른 Build 경로를 사용합니다.

## 환경 선택

- **Laptop CPU Docker:** [Laptop Docker Development Environment](development/laptop-docker.md)를 따릅니다. Linux amd64 Host에서 ROS 2 Humble Container를 실행하며 `build_laptop/install_laptop/log_laptop`을 사용합니다. Phase 1은 Ubuntu 26 LTS amd64 Host에서 검증했습니다.
- **기존 Jetson 수동 설정:** 아래 명령은 GPU virtualenv를 사용하는 기존 절차입니다. Jetson Docker는 아직 구현되지 않았고 JetPack/L4T 조합도 미확정입니다. 아래의 `build/install/log`와 Laptop Docker 산출물을 혼용하지 마세요.

## 기존 Jetson 수동 설정

이 절차는 기존 수동 환경을 설명하며 새 Jetson Docker의 검증 결과가 아닙니다. GPU virtualenv 위치는 해당 장치에서 확인해 `AFTR_FALL_GPU_VENV`로 지정하세요.

### Workspace

Use a ROS 2 workspace that contains this repository under `src/autonomous-following-towing-robot`.

```bash
export AFTR_WS=~/autonomous_following_towing_robot_ws
cd "$AFTR_WS"
```

### Base environment

```bash
source /opt/ros/humble/setup.bash
```

### Standard package build

Most packages use the system Python environment. Build them without the GPU
package so their console scripts keep the system Python interpreter.

```bash
cd "$AFTR_WS"
colcon build --symlink-install --packages-skip aftr_fall_detection
```

### Fall-detection package

`aftr_fall_detection` uses the Jetson GPU virtual environment.

```bash
source /opt/ros/humble/setup.bash
: "${AFTR_FALL_GPU_VENV:?Set AFTR_FALL_GPU_VENV to the Jetson GPU virtualenv path}"
source "$AFTR_FALL_GPU_VENV/bin/activate"
cd "$AFTR_WS"
python3 -m colcon build \
  --symlink-install \
  --packages-select aftr_fall_detection
```

### GUI-only rebuild

```bash
source /opt/ros/humble/setup.bash
cd "$AFTR_WS"
colcon build --symlink-install --packages-select aftr_gui
```

### Mode-manager-only rebuild

```bash
source /opt/ros/humble/setup.bash
cd "$AFTR_WS"
colcon build --symlink-install --packages-select aftr_mode_manager
```

### Run the operator system

```bash
source /opt/ros/humble/setup.bash
: "${AFTR_FALL_GPU_VENV:?Set AFTR_FALL_GPU_VENV to the Jetson GPU virtualenv path}"
source "$AFTR_FALL_GPU_VENV/bin/activate"
source "$AFTR_WS/install/setup.bash"
ros2 launch aftr_gui operator_system.launch.py
```

### Build rule

When rebuilding the workspace, first build normal packages with system Python
while skipping `aftr_fall_detection`. Then activate the GPU virtual
environment and invoke Colcon as `python3 -m colcon` for
`aftr_fall_detection`. Calling bare `/usr/bin/colcon` after activating the
virtual environment still generates a `/usr/bin/python3` entry point and
causes `torch` imports to fail at runtime.
