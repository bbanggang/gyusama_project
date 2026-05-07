#!/bin/bash
# gyusama-project Isaac Sim 실행 스크립트

BRIDGE_DIR=/home/linux/isaac_env/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}   # .bashrc 와 동일한 도메인 사용
export LD_LIBRARY_PATH=$BRIDGE_DIR/jazzy/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$BRIDGE_DIR/jazzy/rclpy:$PYTHONPATH

# 단일 GPU 강제 (RTX 5070 Ti만 사용, Intel 내장그래픽 제외)
export __NV_PRIME_RENDER_OFFLOAD=0
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

echo "[INFO] Isaac Sim Kit 데이터 캐시 삭제 중..."
rm -rf /home/linux/isaac_env/lib/python3.11/site-packages/isaacsim/kit/data/Kit/
echo "[INFO] 캐시 삭제 완료"

echo "[INFO] Isaac Sim 시작 중..."
# omni.graph.image.core 는 항상 제외 (RTX 5070 Ti Blackwell cold-start segfault 우회)
# 카메라는 Replicator annotator + rclpy 직접 발행 방식으로 획득 (5주차 구현)
#   ENABLE_CAMERA=1 → /camera/image_raw 발행 (Replicator → rclpy)
#   ENABLE_CAMERA=0 → LiDAR(/scan) + 제어 토픽만 활성화 (기본값)
export ENABLE_CAMERA=${ENABLE_CAMERA:-0}
echo "[INFO] 카메라 토픽: $([ "$ENABLE_CAMERA" = "1" ] && echo '활성화 (Replicator→rclpy→/camera/image_raw)' || echo '비활성화 (LiDAR 전용)')"

/home/linux/isaac_env/bin/python "$(dirname "$0")/run_track_sim.py"
