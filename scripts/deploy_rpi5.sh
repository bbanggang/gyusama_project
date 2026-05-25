#!/usr/bin/env bash
# deploy_rpi5.sh — PC → RPi5 코드 동기화 및 Docker 컨테이너 재시작 스크립트
#
# 사용법:
#   ./scripts/deploy_rpi5.sh              # 코드 동기화만
#   ./scripts/deploy_rpi5.sh --restart    # 동기화 후 컨테이너 재시작
#   ./scripts/deploy_rpi5.sh --build      # 동기화 + 이미지 빌드 + 재시작
#
# 환경변수로 RPi5 접속 정보를 변경할 수 있습니다:
#   RPI5_HOST=192.168.0.153 RPI5_USER=rapi5 ./scripts/deploy_rpi5.sh

set -euo pipefail

RPI5_HOST="${RPI5_HOST:-192.168.0.153}"
RPI5_USER="${RPI5_USER:-rapi5}"
RPI5_DIR="${RPI5_DIR:-~/gyusama-project}"
LOCAL_DIR="$(cd "$(dirname "$0")/.." && pwd)"

RESTART=0
BUILD=0
for arg in "$@"; do
    case $arg in
        --restart) RESTART=1 ;;
        --build)   BUILD=1; RESTART=1 ;;
    esac
done

echo "=== RPi5 배포 시작 ==="
echo "  대상: ${RPI5_USER}@${RPI5_HOST}:${RPI5_DIR}"
echo "  소스: ${LOCAL_DIR}"
echo ""

# ── 1. 코드 동기화 ─────────────────────────────────────────────────────────────
echo "[1/3] 코드 동기화 중..."
rsync -avz --progress \
    --exclude='.git/' \
    --exclude='.claude/' \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    --exclude='.venv_train/' \
    --exclude='data/' \
    --exclude='/runs/' \
    --include='models/runs/' \
    --include='models/runs/*/' \
    --include='models/runs/*/weights/' \
    --include='models/runs/*/weights/best_int8.onnx' \
    --include='models/runs/*/weights/best.onnx' \
    --exclude='models/runs/*/weights/*.pt' \
    --exclude='models/runs/*/weights/*.onnx' \
    "${LOCAL_DIR}/" \
    "${RPI5_USER}@${RPI5_HOST}:${RPI5_DIR}/"

echo "  동기화 완료"

# ── 2. Docker 이미지 빌드 (선택) ───────────────────────────────────────────────
if [[ $BUILD -eq 1 ]]; then
    echo ""
    echo "[2/3] Docker 이미지 빌드 중 (RPi5)..."
    ssh "${RPI5_USER}@${RPI5_HOST}" \
        "cd ${RPI5_DIR}/docker && docker compose build"
    echo "  빌드 완료"
else
    echo ""
    echo "[2/3] 빌드 스킵 (--build 플래그 없음)"
fi

# ── 3. 컨테이너 재시작 (선택) ──────────────────────────────────────────────────
if [[ $RESTART -eq 1 ]]; then
    echo ""
    echo "[3/3] 컨테이너 재시작 중..."
    ssh "${RPI5_USER}@${RPI5_HOST}" \
        "cd ${RPI5_DIR}/docker && docker compose down && docker compose up -d"
    echo "  재시작 완료"
    echo ""
    echo "  로그 확인:"
    echo "    ssh ${RPI5_USER}@${RPI5_HOST} 'cd ${RPI5_DIR}/docker && docker compose logs -f'"
else
    echo ""
    echo "[3/3] 재시작 스킵 (--restart 플래그 없음)"
fi

echo ""
echo "=== 배포 완료 ==="
