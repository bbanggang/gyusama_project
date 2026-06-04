#!/usr/bin/env bash
# compile_hef.sh — best.onnx → HEF 컴파일
# Hailo AI Software Suite (DFC) Docker 컨테이너 내부에서 실행하는 스크립트
#
# === 사전 준비 ===
#   1. PC에 hailo8_ai_sw_suite_*_docker.zip 다운로드 + unzip
#   2. ./hailo_ai_sw_suite_docker_run.sh 로 컨테이너 진입
#      (이 스크립트가 자동으로 gyusama-project를 마운트하도록 옵션 조정 필요)
#   3. 컨테이너 내부에서: bash /workspace/scripts/compile_hef.sh
#
# === 출력 ===
#   models/runs/lane_det-4/weights/best.hef  (Hailo Executable Format)
#
# === 중요 ===
#   --hw-arch hailo8l  ← 칩(Hailo-8)이 아닌 동작 모드(HAILO8L) 기준 (Phase 1-5에서 확인)

set -euo pipefail

# ── 경로 설정 (컨테이너 내부 기준) ────────────────────────────────────
# DFC docker는 보통 호스트 디렉토리를 /local/shared_with_docker 에 마운트
# 또는 docker run 시 -v ~/gyusama-project:/workspace 로 직접 마운트
PROJ="${PROJ:-/workspace}"
if [ ! -d "$PROJ" ]; then
    PROJ="${PROJ:-/local/shared_with_docker}"
fi
if [ ! -d "$PROJ" ]; then
    echo "[ERROR] 프로젝트 경로 확인 필요. PROJ 환경변수로 지정:"
    echo "  PROJ=/path/to/gyusama-project bash compile_hef.sh"
    exit 1
fi

ONNX="$PROJ/models/runs/lane_det-4/weights/best.onnx"
CALIB="$PROJ/data/calibration_set"
OUT_DIR="$PROJ/models/runs/lane_det-4/weights"
HEF="$OUT_DIR/best.hef"
HAR="$OUT_DIR/best.har"
HW_ARCH=hailo8l

# ── 검증 ─────────────────────────────────────────────────────────────
echo "=== Hailo HEF 컴파일 시작 ==="
echo "  PROJ:    $PROJ"
echo "  ONNX:    $ONNX"
echo "  CALIB:   $CALIB ($(ls $CALIB 2>/dev/null | wc -l)장)"
echo "  HW_ARCH: $HW_ARCH"
echo "  OUT:     $HEF"
echo ""

[ -f "$ONNX" ] || { echo "[ERROR] ONNX 없음: $ONNX"; exit 1; }
[ -d "$CALIB" ] || { echo "[ERROR] 캘리브셋 없음: $CALIB"; exit 1; }

# ── 옵션 A: Hailo Model Zoo (yolov8s 예제 활용, 가장 간편) ──────────
# yolov8s는 Model Zoo에 등록되어 있어 --ckpt로 커스텀 모델 + --calib-path로 캘리브 가능
if command -v hailomz &>/dev/null; then
    echo "[A] hailomz CLI 사용 (Model Zoo yolov8s 베이스)"
    hailomz compile yolov8s \
        --ckpt "$ONNX" \
        --calib-path "$CALIB" \
        --hw-arch "$HW_ARCH" \
        --classes 1 \
        --results-dir "$OUT_DIR"
    # 결과 .hef를 표준 위치로 이동
    GEN_HEF=$(find "$OUT_DIR" -name "yolov8s*.hef" -newer "$ONNX" | head -1)
    [ -n "$GEN_HEF" ] && mv "$GEN_HEF" "$HEF"
else
    # ── 옵션 B: 3단계 수동 (parser → optimize → compile) ─────────────
    echo "[B] hailo CLI 3단계 (Model Zoo 미사용)"

    echo "  [1/3] hailo parser onnx → HAR"
    hailo parser onnx "$ONNX" \
        --hw-arch "$HW_ARCH" \
        --har-path "$HAR"

    echo "  [2/3] hailo optimize (INT8 양자화, 캘리브셋 사용)"
    hailo optimize "$HAR" \
        --calib-set-path "$CALIB" \
        --hw-arch "$HW_ARCH" \
        --output-har-path "${HAR%.har}_optimized.har"

    echo "  [3/3] hailo compiler → HEF"
    hailo compiler "${HAR%.har}_optimized.har" \
        --hw-arch "$HW_ARCH" \
        --output-dir "$OUT_DIR"
fi

# ── 결과 확인 ────────────────────────────────────────────────────────
echo ""
echo "=== 컴파일 결과 ==="
if [ -f "$HEF" ]; then
    SZ=$(du -h "$HEF" | cut -f1)
    echo "  ✅ HEF 생성: $HEF ($SZ)"
    echo ""
    echo "  RPi5 전송:"
    echo "    scp -O $HEF rapi5@192.168.0.155:~/gyusama-project/models/runs/lane_det-4/weights/"
else
    echo "  ❌ HEF 생성 실패. 로그 확인 필요"
    exit 1
fi
