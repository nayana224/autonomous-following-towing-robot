# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive \
    CUDA_VISIBLE_DEVICES="" \
    QT_QPA_PLATFORM=offscreen \
    SDL_AUDIODRIVER=dummy \
    PYTHONUNBUFFERED=1

COPY docker/common/apt-packages.txt /tmp/apt-packages.txt
COPY docker/common/ros-packages.txt /tmp/ros-packages.txt

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      $(grep -vE '^[[:space:]]*(#|$)' /tmp/apt-packages.txt) \
      $(grep -vE '^[[:space:]]*(#|$)' /tmp/ros-packages.txt) && \
    fc-cache -f && \
    rm -rf /var/lib/apt/lists/* /tmp/apt-packages.txt /tmp/ros-packages.txt

# Jammy's NumPy is older than Ultralytics requires. Keep NumPy on its 1.x ABI
# for the ROS cv_bridge extension and pin OpenCV before installing Ultralytics.
RUN python3 -m pip install --no-cache-dir \
      numpy==1.26.4 opencv-python==4.8.1.78 && \
    python3 -m pip install --no-cache-dir \
      torch==2.2.2+cpu torchvision==0.17.2+cpu \
      --index-url https://download.pytorch.org/whl/cpu && \
    python3 -m pip install --no-cache-dir \
      numpy==1.26.4 opencv-python==4.8.1.78 ultralytics==8.2.103 && \
    python3 -c 'import torch; assert torch.version.cuda is None and not torch.cuda.is_available()'

COPY docker/common/entrypoint.sh /usr/local/bin/aftr-entrypoint
RUN chmod +x /usr/local/bin/aftr-entrypoint

WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/aftr-entrypoint"]
CMD ["bash"]
