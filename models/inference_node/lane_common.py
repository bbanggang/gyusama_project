"""
lane_common.py — lane_detect_{ncnn|hailo|onnx}.py 가 공유하는 순수 함수

추출 이유:
  · plan.md Phase 4 선행 리팩토링 — 동일 로직 중복 정의 제거
  · 향후 lane_detect_hailo.py 추가로 3중 복제 방지
  · 카메라 오프셋 보정(`center_bias_px`) 같은 변경을 한 곳에서 처리

엔진별 파일은 다음만 별도 구현:
  · YoloDetector 클래스 (NCNN / HailoRT / ONNXRuntime API 차이)
  · ROS2 노드 클래스 (모델 로드/추론 부분만 다름)

본 파일이 제공하는 함수:
  · load_lane_params()    YAML 파라미터 로드 (config/apf_params.yaml 의 lane_detect 섹션)
  · compute_lane_offset() bbox → 차선 중심 오프셋 (−1 ~ +1)
  · fallback_lane_offset() 모델 미가용 시 흰색 픽셀 기반 폴백
  · nms()                 클래스-내 IoU NMS

참고:
  · 본 파일은 ROS2, ncnn, hailort 등 무거운 import 를 하지 않음 (순수 numpy/cv2).
  · 호출하는 쪽이 필요한 IMG_W, IMG_H, LANE_ROI_TOP_RATIO 등을 인자로 넘김.
"""

from __future__ import annotations

import os
from pathlib import Path
import numpy as np


# ─── 프로젝트 루트 ─────────────────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parent.parent.parent  # gyusama-project/


# ═══════════════════════════════════════════════════════════════════════════════
# ── YAML 파라미터 로드 ────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

LANE_PARAM_DEFAULTS = dict(
    max_speed=0.14,
    min_speed=0.08,
    max_angular=12.0,
    kp_angular=11.0,
    conf_thresh=0.12,
    steer_exp=1.4,
    lane_roi_top=0.50,
    lane_track_thresh=80,
)


def load_lane_params(yaml_path: Path | None = None) -> dict:
    """`config/apf_params.yaml` 의 `lane_detect:` 섹션을 dict 로 로드한다.

    Parameters
    ----------
    yaml_path : Path | None
        명시 안 하면 `ROOT/config/apf_params.yaml` 사용.

    Returns
    -------
    dict
        키는 LANE_PARAM_DEFAULTS 동일. 파일 없거나 파싱 실패 시 기본값.
    """
    if yaml_path is None:
        yaml_path = ROOT / 'config' / 'apf_params.yaml'

    if not yaml_path.exists():
        return dict(LANE_PARAM_DEFAULTS)

    try:
        import yaml
        data = yaml.safe_load(yaml_path.read_text()) or {}
        p = data.get('lane_detect', {})
        return {k: p.get(k, v) for k, v in LANE_PARAM_DEFAULTS.items()}
    except Exception:
        return dict(LANE_PARAM_DEFAULTS)


# ═══════════════════════════════════════════════════════════════════════════════
# ── NMS (클래스-내) ────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

def nms(cx: np.ndarray, cy: np.ndarray, w: np.ndarray, h: np.ndarray,
        scores: np.ndarray, iou_thr: float = 0.45) -> np.ndarray:
    """단순 클래스-내 NMS. 입력 모두 같은 길이의 1D ndarray.

    Returns
    -------
    np.ndarray  유지된 인덱스 (int64)
    """
    x1 = cx - w / 2;  y1 = cy - h / 2
    x2 = cx + w / 2;  y2 = cy + h / 2
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep: list[int] = []
    while order.size:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
        iou   = inter / (areas[i] + areas[order[1:]] - inter + 1e-7)
        order = order[1:][iou < iou_thr]
    return np.array(keep, dtype=np.int64)


# ═══════════════════════════════════════════════════════════════════════════════
# ── 차선 중심 오프셋 계산 ────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

def compute_lane_offset(boxes: list[dict], img_w: int) -> tuple[float | None, dict]:
    """검출된 bbox 에서 목표 중심 오프셋을 계산한다.

    Parameters
    ----------
    boxes : list[dict]
        각 원소: {"cx": float, "cy": float, "w": float, "h": float, "conf": float}
        좌표는 원본 이미지(BGR) 픽셀 단위.
    img_w : int
        원본 이미지 너비.

    Returns
    -------
    offset : float in [-1, +1] | None
        −1 = 화면 가장 왼쪽, +1 = 가장 오른쪽. 차선 미검출 시 None.
    info : dict
        {'left_cx', 'right_cx', 'mid_cx'} (시각화용)
    """
    if not boxes:
        return None, {}

    sorted_boxes = sorted(boxes, key=lambda b: b["cx"])

    if len(sorted_boxes) == 1:
        # 한쪽 차선만 검출 — 반대 차선을 추정(±0.25·img_w) 후 중심 계산
        b  = sorted_boxes[0]
        cx = b["cx"]
        if cx < img_w / 2:
            # 왼쪽 차선만 보임 → 로봇이 오른쪽으로 치우침 → 왼쪽으로 조향
            mid_cx = cx + img_w * 0.25
        else:
            # 오른쪽 차선만 보임 → 로봇이 왼쪽으로 치우침 → 오른쪽으로 조향
            mid_cx = cx - img_w * 0.25
        offset = (mid_cx - img_w / 2.0) / (img_w / 2.0)
        return float(np.clip(offset, -1.0, 1.0)), {
            "left_cx":  cx if cx < img_w / 2 else None,
            "right_cx": cx if cx >= img_w / 2 else None,
            "mid_cx":   mid_cx,
        }

    # 좌/우 차선 모두 검출
    left_cx  = sorted_boxes[0]["cx"]
    right_cx = sorted_boxes[-1]["cx"]
    mid_cx   = (left_cx + right_cx) / 2.0
    offset   = (mid_cx - img_w / 2.0) / (img_w / 2.0)
    return float(np.clip(offset, -1.0, 1.0)), {
        "left_cx":  left_cx,
        "right_cx": right_cx,
        "mid_cx":   mid_cx,
    }


# ═══════════════════════════════════════════════════════════════════════════════
# ── 흰색 픽셀 기반 폴백 ──────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

def fallback_lane_offset(bgr: np.ndarray, lane_roi_top: float,
                         img_w: int, img_h: int) -> tuple[float | None, dict]:
    """모델 미가용 시 ROI 내 흰색 픽셀로 좌/우 차선 x 중심을 추정.

    Parameters
    ----------
    bgr : np.ndarray (h, w, 3)
    lane_roi_top : float [0,1]   상단 컷 비율 (이 비율 위는 무시)
    img_w, img_h : int            원본 이미지 크기 (BGR shape 가 다르면 그쪽이 우선)
    """
    import cv2  # 호출 시점 import (테스트 환경에서 cv2 없어도 import 가능)

    gray   = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    h_in, w_in = bgr.shape[:2]
    roi_y  = int(h_in * lane_roi_top)
    roi    = gray[roi_y:]

    WHITE_THRESH = 180   # 200→180: 조명 변화 강건
    mask   = roi > WHITE_THRESH
    half   = w_in // 2
    left   = mask.copy(); left[:,  half:] = False
    right  = mask.copy(); right[:, :half] = False

    def _col_center(m):
        _, xs = np.where(m)
        return float(xs.mean()) if xs.size > 20 else None

    lx = _col_center(left)
    rx = _col_center(right)

    boxes = []
    if lx is not None:
        boxes.append({"cx": lx})
    if rx is not None:
        # rx 는 np.where 결과 → 이미 절대 x 좌표 (half ~ w_in)
        boxes.append({"cx": rx})

    return compute_lane_offset(boxes, w_in)
