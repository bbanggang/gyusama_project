#!/bin/bash
# ros2_terminal.sh — Isaac Sim 과 통신하는 ROS2 터미널 환경 설정
# 사용법: source ros2_terminal.sh
#
# 핵심 원칙:
#   - Isaac Sim의 bridge PYTHONPATH 는 Isaac Sim 내부 전용 → 외부 노드에 주입 금지
#   - 외부 ROS2 노드는 시스템 rclpy(Python 3.12) 만 사용해야 함
#   - RMW_IMPLEMENTATION 만 맞춰주면 DDS 통신 가능

# ── isaac_env 완전 비활성화 (Python 3.11 → 3.12 충돌 방지) ─────────────
if [ -n "$VIRTUAL_ENV" ]; then
    deactivate 2>/dev/null || true
    echo "[INFO] isaac_env 비활성화 완료"
fi

# ── PATH 에서 isaac_env/bin 제거 (deactivate 가 누락한 경우 대비) ────────
ISAAC_ENV_BIN="/home/linux/isaac_env/bin"
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "^${ISAAC_ENV_BIN}$" | tr '\n' ':' | sed 's/:$//')

# ── PYTHONPATH 에서 Isaac Sim bridge rclpy 제거 ─────────────────────────
BRIDGE_RCLPY="/home/linux/isaac_env/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/rclpy"
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v "$BRIDGE_RCLPY" | tr '\n' ':' | sed 's/:$//')

# ── PYTHONPATH 에서 Python 3.11 site-packages 제거 ──────────────────────
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v "python3\.11" | tr '\n' ':' | sed 's/:$//')

# ── unset PYTHONHOME (가상환경 잔여) ───────────────────────────────────
unset PYTHONHOME

# ── ROS2 Jazzy 환경 로드 (Python 3.12 rclpy) ────────────────────────────
source /opt/ros/jazzy/setup.bash
echo "[INFO] ROS2 Jazzy 환경 로드 완료 (Python $(python3 --version 2>&1 | awk '{print $2}'))"

# ── DDS 구현체 통일 (Isaac Sim 과 동일하게) ──────────────────────────────
# PYTHONPATH, LD_LIBRARY_PATH 는 건드리지 않음 — 시스템 ROS2 경로만 사용
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}   # launch_sim.sh 와 동일한 도메인

echo "[INFO] RMW          : rmw_fastrtps_cpp"
echo "[INFO] ROS_DOMAIN_ID: $ROS_DOMAIN_ID  (Isaac Sim 과 일치 필수)"
echo ""
echo "사용 가능한 명령:"
echo "  teleop  : ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "  자율주행: python3 /home/linux/gyusama-project/isaac_sim/lane_follow_nav.py"
echo "  토픽확인: ros2 topic list"
