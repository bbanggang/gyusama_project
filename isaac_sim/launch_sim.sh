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

# ── RTX 5070 Ti (Blackwell) segfault 우회 ──────────────────────────────────
# librtx.scenedb.plugin.so 가 carbOnPluginStartup 에서 segfault 발생.
# .so → .so.bak 으로 이름 변경하여 Kit 엔진이 로드하지 못하게 차단.
# USE_RTX=1 로 실행하면 원래 이름으로 복원 (Isaac Sim 업데이트 후 테스트용).
RTX_SCENEDB=/home/linux/isaac_env/lib/python3.11/site-packages/isaacsim/extscache/omni.hydra.rtx-1.0.0+69cbf6ad.lx64.r/bin/deps/librtx.scenedb.plugin.so
export USE_RTX=${USE_RTX:-0}

if [ "$USE_RTX" = "1" ]; then
    # RTX 복원 모드
    if [ -f "${RTX_SCENEDB}.bak" ] && [ ! -f "$RTX_SCENEDB" ]; then
        mv "${RTX_SCENEDB}.bak" "$RTX_SCENEDB"
        echo "[INFO] RTX scenedb 플러그인 복원 완료"
    fi
else
    # Blackwell 우회 모드: scenedb 플러그인 비활성화
    if [ -f "$RTX_SCENEDB" ]; then
        mv "$RTX_SCENEDB" "${RTX_SCENEDB}.bak"
        echo "[INFO] RTX scenedb 플러그인 비활성화 (Blackwell segfault 우회)"
    else
        echo "[INFO] RTX scenedb 플러그인 이미 비활성화 상태"
    fi
fi

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
echo "[INFO] 렌더러: $([ "$USE_RTX" = "1" ] && echo 'RTX (레이트레이싱)' || echo 'Storm/pxr (래스터화)')"

/home/linux/isaac_env/bin/python "$(dirname "$0")/run_track_sim.py"
