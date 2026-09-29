#!/bin/bash
# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors

echo "MDBot 장치(Motor & Lidar) udev 설정을 시작합니다..."

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
FILENAME="99-mdbot-devices.rules"
FILE_PATH="$SCRIPT_DIR/$FILENAME"
TARGET_DIR="/etc/udev/rules.d/"

if [ ! -f "$FILE_PATH" ]; then
    echo "오류: $FILE_PATH 파일을 찾을 수 없습니다."
    exit 1
fi

sudo cp "$FILE_PATH" "$TARGET_DIR"

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "===================================================="
echo "설정 완료! 장치들을 다시 연결해 주세요."
echo "확인: ls -l /dev/ttyMotor  및  ls -l /dev/ttyLidar"
echo "===================================================="