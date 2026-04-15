#!/bin/bash
# gyusama-project Isaac Sim 실행 스크립트

BRIDGE_DIR=/home/linux/isaac_env/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$BRIDGE_DIR/jazzy/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$BRIDGE_DIR/jazzy/rclpy:$PYTHONPATH

# 단일 GPU 강제 (RTX 5070 Ti만 사용, Intel 내장그래픽 제외)
export __NV_PRIME_RENDER_OFFLOAD=0
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

echo "[INFO] Isaac Sim Kit 데이터 캐시 삭제 중..."
rm -rf /home/linux/isaac_env/lib/python3.11/site-packages/isaacsim/kit/data/Kit/
echo "[INFO] 캐시 삭제 완료"

echo "[INFO] Isaac Sim 시작 중..."
# run_ros2_sim.py  : simple_room USD 원본 씬
# run_track_sim.py : 직사각형 루프 트랙 씬 (차선 마킹 + 벽)
/home/linux/isaac_env/bin/python "$(dirname "$0")/run_track_sim.py"
