#!/usr/bin/env bash
# stop_nodes.sh — RPi5 실행 중인 모든 노드 종료
#
# 사용법:
#   ./scripts/stop_nodes.sh

set -euo pipefail

RPI5_HOST="${RPI5_HOST:-192.168.0.155}"
RPI5_USER="${RPI5_USER:-rapi5}"
RPI5_DIR="${RPI5_DIR:-~/gyusama-project}"

echo "=== RPi5 노드 종료 ==="
echo "  대상: ${RPI5_USER}@${RPI5_HOST}"
echo ""

echo "[1/2] Docker 컨테이너 종료 중..."
ssh "${RPI5_USER}@${RPI5_HOST}" \
    "cd ${RPI5_DIR}/docker && docker compose down" && echo "  컨테이너 종료 완료"

echo ""
echo "[2/2] camera_node 종료 중..."
ssh "${RPI5_USER}@${RPI5_HOST}" \
    "pkill -f camera_node 2>/dev/null && echo '  camera_node 종료 완료' || echo '  camera_node 실행 중 아님'"

echo ""
echo "=== 종료 완료 ==="
