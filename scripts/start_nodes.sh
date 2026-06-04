#!/usr/bin/env bash
# start_nodes.sh — RPi5 전체 노드 기동
#
# 사용법:
#   ./scripts/start_nodes.sh              # 노드 기동 후 수동으로 's' 키 입력해 주행
#   ./scripts/start_nodes.sh --autostart  # 노드 기동 후 자동으로 주행 시작
#                                         # 로봇을 트랙 위에 올려놓은 뒤 실행할 것
#
# 환경변수로 RPi5 접속 정보를 변경할 수 있습니다:
#   RPI5_HOST=192.168.0.155 ./scripts/start_nodes.sh

set -euo pipefail

RPI5_HOST="${RPI5_HOST:-192.168.0.155}"
RPI5_USER="${RPI5_USER:-rapi5}"
RPI5_DIR="${RPI5_DIR:-~/gyusama-project}"

AUTOSTART=0
for arg in "$@"; do
    case $arg in
        --autostart) AUTOSTART=1 ;;
    esac
done

echo "=== RPi5 노드 기동 ==="
echo "  대상: ${RPI5_USER}@${RPI5_HOST}"
if [[ $AUTOSTART -eq 1 ]]; then
    echo "  모드: AUTOSTART (기동 후 자동 주행)"
else
    echo "  모드: 수동 주행 시작 ('s' 키 필요)"
fi
echo ""

# ── 1. Docker 컨테이너 기동 ────────────────────────────────────────────────────
echo "[1/3] Docker 컨테이너 기동 중..."
ssh "${RPI5_USER}@${RPI5_HOST}" \
    "cd ${RPI5_DIR}/docker && AUTOSTART=${AUTOSTART} docker compose up -d"
echo "  컨테이너 기동 완료"

# ── 2. camera_node 기동 (호스트 직접 실행 — libcamera RPi fork 필요) ────────────
echo ""
echo "[2/3] camera_node 기동 중 (RPi5 호스트)..."
ssh "${RPI5_USER}@${RPI5_HOST}" bash <<'ENDSSH'
    if pgrep -f camera_node > /dev/null 2>&1; then
        echo "  camera_node 이미 실행 중 — 스킵"
    else
        source /opt/ros/jazzy/setup.bash
        export ROS_DOMAIN_ID=1
        export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
        nohup ros2 run camera_ros camera_node \
            > /tmp/camera_node.log 2>&1 &
        echo $! > /tmp/camera_node.pid
        echo "  camera_node 기동 완료 (PID: $(cat /tmp/camera_node.pid))"
    fi
ENDSSH

# ── 3. 노드 준비 대기 및 상태 확인 ────────────────────────────────────────────
echo ""
echo "[3/3] 노드 준비 대기 중 (5초)..."
sleep 5

echo ""
echo "  [컨테이너 상태]"
ssh "${RPI5_USER}@${RPI5_HOST}" \
    "docker ps --format '  {{.Names}}\t{{.Status}}' \
     | grep -E 'inference|obstacle|behavior|control' || echo '  실행 중인 컨테이너 없음'"

echo ""
echo "  [camera_node]"
ssh "${RPI5_USER}@${RPI5_HOST}" \
    "pgrep -a -f camera_node | grep -v grep \
     && echo '  → 실행 중' || echo '  → 미실행 (로그: /tmp/camera_node.log)'"

# ── 4. 완료 메시지 ─────────────────────────────────────────────────────────────
echo ""
echo "=== 기동 완료 ==="
echo ""

if [[ $AUTOSTART -eq 1 ]]; then
    echo "  로봇이 트랙 위에 있으면 차선 추종 + 장애물 회피 주행이 시작됩니다."
    echo ""
    echo "  주행 중지:"
    echo "    docker attach inference-node  →  'q' 키"
    echo "    또는: ./scripts/stop_nodes.sh"
else
    echo "  주행 시작 방법:"
    echo "    ssh ${RPI5_USER}@${RPI5_HOST}"
    echo "    docker attach inference-node"
    echo "    → 's' 키 입력"
fi

echo ""
echo "  노드 모니터링:"
echo "    ssh ${RPI5_USER}@${RPI5_HOST} 'python3 ${RPI5_DIR}/scripts/monitor_nodes.py'"
