#!/bin/bash
# ROS2 환경 소싱 후 전달된 명령어 실행
source /opt/ros/jazzy/setup.bash

# 워크스페이스 빌드 결과물이 있으면 같이 소싱
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
