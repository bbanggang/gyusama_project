#!/usr/bin/env bash
# build_hub.sh — ARM64 이미지 빌드 후 Docker Hub push
#
# 사용법:
#   ./scripts/build_hub.sh              # 빌드 + push
#   ./scripts/build_hub.sh --no-push    # 빌드만 (로컬 테스트용)
#
# 사전 조건:
#   docker login  (Docker Hub 로그인 필요)
#   docker buildx inspect --bootstrap  (멀티아키텍처 빌더 활성화 필요)

set -euo pipefail

IMAGE="${IMAGE:-bbanggang/gyusama-rpi5:latest}"
PUSH=1
for arg in "$@"; do
    case $arg in
        --no-push) PUSH=0 ;;
    esac
done

# 프로젝트 루트 기준으로 실행
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "=== Docker Hub 이미지 빌드 ==="
echo "  이미지: ${IMAGE}"
echo "  플랫폼: linux/arm64"
echo "  컨텍스트: ${PROJECT_ROOT}"
echo ""

# buildx 빌더 확인 (없으면 생성)
if ! docker buildx ls | grep -q "gyusama-builder"; then
    echo "[준비] buildx 빌더 생성 중..."
    docker buildx create --name gyusama-builder --use
    docker buildx inspect --bootstrap
    echo ""
fi

# ── ARM64 빌드 ─────────────────────────────────────────────────────────────────
echo "[1/2] ARM64 이미지 빌드 중..."
echo "  (QEMU 에뮬레이션으로 크로스빌드, 5~15분 소요)"
echo ""

if [[ $PUSH -eq 1 ]]; then
    docker buildx build \
        --platform linux/arm64 \
        -t "${IMAGE}" \
        --push \
        -f "${PROJECT_ROOT}/docker/Dockerfile.rpi5" \
        "${PROJECT_ROOT}"
    echo ""
    echo "[2/2] Docker Hub push 완료"
    echo "  이미지: ${IMAGE}"
    echo ""
    echo "=== 완료 ==="
    echo ""
    echo "  RPi5에서 실행:"
    echo "    docker compose pull && docker compose up -d"
else
    docker buildx build \
        --platform linux/arm64 \
        -t "${IMAGE}" \
        --load \
        -f "${PROJECT_ROOT}/docker/Dockerfile.rpi5" \
        "${PROJECT_ROOT}"
    echo ""
    echo "[2/2] 로컬 저장 완료 (push 스킵)"
    echo ""
    echo "=== 완료 ==="
fi
