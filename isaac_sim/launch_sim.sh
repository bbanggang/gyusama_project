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
# 카메라 토픽 활성화: ENABLE_CAMERA=1 bash launch_sim.sh
# (RTX 5070 Ti 에서 충돌 발생 시 기본값(0) 유지)
export ENABLE_CAMERA=${ENABLE_CAMERA:-0}
echo "[INFO] 카메라 토픽: $([ "$ENABLE_CAMERA" = "1" ] && echo '활성화 (/camera/image_raw)' || echo '비활성화 (LiDAR 전용)')"

/home/linux/isaac_env/bin/python "$(dirname "$0")/run_track_sim.py"
