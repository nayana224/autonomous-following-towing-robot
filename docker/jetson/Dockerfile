# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
FROM nvcr.io/nvidia/l4t-cuda:12.6.11-runtime@sha256:49853e9fe2a0305efa51fcbb3426eea1bca658085f2d26bb40347711434849b1

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1

# ROS Humble and the Jetson wheels both require Jammy and Python 3.10.
RUN test "$(. /etc/os-release && echo "$VERSION_CODENAME")" = jammy && \
    apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg python3 python3-pip && \
    python3 -c 'import sys; assert sys.version_info[:2] == (3, 10)' && \
    curl -fsSL \
      https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.jammy_all.deb \
      -o /tmp/ros2-apt-source.deb && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb && \
    curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
      -o /usr/share/keyrings/jetson-ota-public.asc && \
    echo "deb [signed-by=/usr/share/keyrings/jetson-ota-public.asc] https://repo.download.nvidia.com/jetson/common r36.5 main" \
      > /etc/apt/sources.list.d/jetson-r36.5.list

COPY docker/common/apt-packages.txt /tmp/apt-packages.txt
COPY docker/common/ros-packages.txt /tmp/ros-packages.txt

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-ros-base libcudnn9-cuda-12=9.3.0.75-1 \
      $(grep -vE '^[[:space:]]*(#|$)' /tmp/apt-packages.txt) \
      $(grep -vE '^[[:space:]]*(#|$)' /tmp/ros-packages.txt) && \
    fc-cache -f && \
    rm -rf /var/lib/apt/lists/* /tmp/apt-packages.txt /tmp/ros-packages.txt

# The Jetson PyTorch wheel links CUPTI even for ordinary imports.
RUN apt-get update && \
    apt-get install -y --no-install-recommends cuda-cupti-12-6=12.6.68-1 && \
    rm -rf /var/lib/apt/lists/*

# JetPack 6 / CUDA 12.6 arm64 wheels. SHA-256 pins prevent silent wheel changes.
RUN curl -fsSL \
      https://pypi.jetson-ai-lab.io/jp6/cu126/+f/62a/1beee9f2f1470/torch-2.8.0-cp310-cp310-linux_aarch64.whl \
      -o /tmp/torch-2.8.0-cp310-cp310-linux_aarch64.whl && \
    echo "62a1beee9f2f147076a974d2942c90060c12771c94740830327cae705b2595fc  /tmp/torch-2.8.0-cp310-cp310-linux_aarch64.whl" | sha256sum -c - && \
    curl -fsSL \
      https://pypi.jetson-ai-lab.io/jp6/cu126/+f/907/c4c1933789645/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl \
      -o /tmp/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl && \
    echo "907c4c1933789645ebb20dd9181d40f8647978e6bd30086ae7b01febb937d2d1  /tmp/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl" | sha256sum -c - && \
    python3 -m pip install --no-cache-dir \
      numpy==1.26.4 opencv-python==4.8.1.78 \
      /tmp/torch-2.8.0-cp310-cp310-linux_aarch64.whl \
      /tmp/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl && \
    python3 -m pip install --no-cache-dir --no-deps \
      ultralytics==8.2.103 ultralytics-thop==2.0.9 \
      py-cpuinfo==9.0.0 Jetson.GPIO==2.1.9 && \
    python3 -c 'import torch; assert torch.version.cuda.startswith("12.6")' && \
    rm /tmp/torch-2.8.0-cp310-cp310-linux_aarch64.whl \
       /tmp/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl

COPY docker/common/entrypoint.sh /usr/local/bin/aftr-entrypoint
RUN chmod +x /usr/local/bin/aftr-entrypoint

WORKDIR /workspace
ENTRYPOINT ["/opt/nvidia/nvidia_entrypoint.sh", "/usr/local/bin/aftr-entrypoint"]
CMD ["bash"]
