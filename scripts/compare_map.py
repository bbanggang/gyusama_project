"""
FP32 vs INT8 mAP 비교 스크립트
실행: .venv_train/bin/python scripts/compare_map.py
"""
import os
import glob
import numpy as np
import cv2
from pathlib import Path
from ultralytics import YOLO

DATA_YAML  = "data/synthetic/dataset.yaml"
FP32_MODEL = "models/runs/lane_det-3/weights/best.pt"
FP32_ONNX  = "models/runs/lane_det-3/weights/best.onnx"
INT8_MODEL = "models/runs/lane_det-3/weights/best_int8.onnx"

CONF_THRESH = 0.12
NMS_IOU_THR = 0.45
IMG_SZ      = 640

# ─── 1) ultralytics val() — FP32 PT 모델 ──────────────────────────────────────
print("=" * 55)
print("1) FP32 (.pt) 모델 검증 중...")
fp32 = YOLO(FP32_MODEL)
r = fp32.val(data=DATA_YAML, imgsz=IMG_SZ, verbose=False)
map50_fp32   = r.box.map50
map5095_fp32 = r.box.map

# ─── 2) ultralytics val() — FP32 ONNX 모델 ───────────────────────────────────
print("\n2) FP32 (.onnx) 모델 검증 중...")
fp32_onnx = YOLO(FP32_ONNX)
r2 = fp32_onnx.val(data=DATA_YAML, imgsz=IMG_SZ, verbose=False, device='cpu')
map50_fp32_onnx   = r2.box.map50
map5095_fp32_onnx = r2.box.map

# ─── 3) INT8 ONNX — ONNX Runtime 직접 평가 ───────────────────────────────────
print("\n3) INT8 (.onnx) 모델 직접 평가 중...")

import onnxruntime as ort

sess = ort.InferenceSession(INT8_MODEL, providers=["CPUExecutionProvider"])
input_name  = sess.get_inputs()[0].name
output_name = sess.get_outputs()[0].name

# 출력 값 범위 진단 (임의 입력)
_dummy = np.random.randn(1, 3, IMG_SZ, IMG_SZ).astype(np.float32)
_out   = sess.run([output_name], {input_name: _dummy})[0]
print(f"   INT8 출력 dtype : {_out.dtype}")
print(f"   INT8 출력 shape : {_out.shape}")
print(f"   INT8 출력 범위  : {_out.min():.4f} ~ {_out.max():.4f}")


def preprocess(img_path):
    img = cv2.imread(img_path)
    h0, w0 = img.shape[:2]
    img_rgb = cv2.cvtColor(cv2.resize(img, (IMG_SZ, IMG_SZ)), cv2.COLOR_BGR2RGB)
    inp = img_rgb.astype(np.float32) / 255.0
    inp = inp.transpose(2, 0, 1)[np.newaxis]
    return inp, w0, h0


def nms(boxes, scores, iou_thr):
    """단순 NMS — boxes: (N,4) xyxy, scores: (N,)"""
    order = scores.argsort()[::-1]
    keep  = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
        yy1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
        xx2 = np.minimum(boxes[i, 2], boxes[order[1:], 2])
        yy2 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
        w   = np.maximum(0, xx2 - xx1)
        h   = np.maximum(0, yy2 - yy1)
        inter = w * h
        area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        area_j = (boxes[order[1:], 2] - boxes[order[1:], 0]) * \
                 (boxes[order[1:], 3] - boxes[order[1:], 1])
        iou   = inter / (area_i + area_j - inter + 1e-6)
        order = order[1:][iou < iou_thr]
    return keep


def infer_int8(img_path):
    """INT8 모델로 추론 → [[cx,cy,w,h,conf], ...] (정규화 좌표)"""
    inp, w0, h0 = preprocess(img_path)
    raw = sess.run([output_name], {input_name: inp})[0]  # (1, 5, 8400)
    pred = raw[0].T   # (8400, 5)
    cx, cy, bw, bh, conf = pred[:, 0], pred[:, 1], pred[:, 2], pred[:, 3], pred[:, 4]
    mask  = conf > CONF_THRESH
    if not mask.any():
        return np.zeros((0, 5))
    cx, cy, bw, bh, conf = cx[mask], cy[mask], bw[mask], bh[mask], conf[mask]
    x1 = (cx - bw / 2) / IMG_SZ
    y1 = (cy - bh / 2) / IMG_SZ
    x2 = (cx + bw / 2) / IMG_SZ
    y2 = (cy + bh / 2) / IMG_SZ
    boxes_xyxy = np.stack([x1, y1, x2, y2], axis=1)
    keep = nms(boxes_xyxy, conf, NMS_IOU_THR)
    boxes_xyxy = boxes_xyxy[keep]
    conf = conf[keep]
    return np.concatenate([boxes_xyxy, conf[:, None]], axis=1)


def compute_iou(b1, b2):
    """b1, b2: (4,) xyxy"""
    xx1 = max(b1[0], b2[0]); yy1 = max(b1[1], b2[1])
    xx2 = min(b1[2], b2[2]); yy2 = min(b1[3], b2[3])
    inter = max(0, xx2 - xx1) * max(0, yy2 - yy1)
    a1 = (b1[2] - b1[0]) * (b1[3] - b1[1])
    a2 = (b2[2] - b2[0]) * (b2[3] - b2[1])
    return inter / (a1 + a2 - inter + 1e-6)


def compute_ap(recalls, precisions):
    """COCO-style AP (101-point interpolation)"""
    ap = 0.0
    for t in np.linspace(0, 1, 101):
        prec = precisions[recalls >= t]
        ap  += (prec.max() if prec.size else 0.0)
    return ap / 101


def evaluate_int8(val_img_dir, label_dir, iou_thresholds):
    img_files = sorted(glob.glob(os.path.join(val_img_dir, "*.jpg")) +
                       glob.glob(os.path.join(val_img_dir, "*.png")))
    all_scores, all_tp_fp = {t: ([], []) for t in iou_thresholds}, {}
    all_scores  = {t: [] for t in iou_thresholds}
    all_tp      = {t: [] for t in iou_thresholds}
    all_fp      = {t: [] for t in iou_thresholds}
    n_gt = 0

    for img_path in img_files:
        stem = Path(img_path).stem
        label_path = os.path.join(label_dir, stem + ".txt")

        # ground truth (YOLO 형식: cls cx cy w h, 정규화)
        gt_boxes = []
        if os.path.exists(label_path):
            with open(label_path) as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) == 5:
                        _, cx, cy, bw, bh = map(float, parts)
                        gt_boxes.append([cx - bw/2, cy - bh/2, cx + bw/2, cy + bh/2])
        n_gt += len(gt_boxes)

        dets = infer_int8(img_path)  # (N, 5) xyxy + conf

        for iou_thr in iou_thresholds:
            matched = [False] * len(gt_boxes)
            # 신뢰도 내림차순 정렬
            if len(dets):
                order = dets[:, 4].argsort()[::-1]
                for det in dets[order]:
                    box, score = det[:4], det[4]
                    best_iou, best_j = 0, -1
                    for j, gt in enumerate(gt_boxes):
                        if matched[j]:
                            continue
                        iou = compute_iou(box, gt)
                        if iou > best_iou:
                            best_iou, best_j = iou, j
                    if best_iou >= iou_thr and best_j >= 0:
                        matched[best_j] = True
                        all_tp[iou_thr].append(1)
                        all_fp[iou_thr].append(0)
                    else:
                        all_tp[iou_thr].append(0)
                        all_fp[iou_thr].append(1)
                    all_scores[iou_thr].append(score)

    # AP 계산
    aps = {}
    for iou_thr in iou_thresholds:
        scores = np.array(all_scores[iou_thr])
        tp     = np.array(all_tp[iou_thr])
        fp     = np.array(all_fp[iou_thr])
        if len(scores) == 0:
            aps[iou_thr] = 0.0
            continue
        order   = scores.argsort()[::-1]
        tp_cum  = np.cumsum(tp[order])
        fp_cum  = np.cumsum(fp[order])
        recalls    = tp_cum / (n_gt + 1e-6)
        precisions = tp_cum / (tp_cum + fp_cum + 1e-6)
        aps[iou_thr] = compute_ap(recalls, precisions)
    return aps


# 검증 이미지/라벨 경로
val_img_dir = "data/synthetic/images/val"
val_lbl_dir = "data/synthetic/labels/val"

iou_thresholds = np.linspace(0.5, 0.95, 10).tolist()
aps = evaluate_int8(val_img_dir, val_lbl_dir, iou_thresholds)

map50_int8   = aps[0.5]
map5095_int8 = float(np.mean(list(aps.values())))

# ─── 결과 출력 ────────────────────────────────────────────────────────────────
print("\n" + "=" * 55)
print(f"{'모델':<18} {'mAP50':>10} {'mAP50-95':>10}")
print("-" * 55)
print(f"{'FP32 (.pt)':<18} {map50_fp32:>10.4f} {map5095_fp32:>10.4f}")
print(f"{'FP32 (.onnx)':<18} {map50_fp32_onnx:>10.4f} {map5095_fp32_onnx:>10.4f}")
print(f"{'INT8 (.onnx)':<18} {map50_int8:>10.4f} {map5095_int8:>10.4f}")
print("-" * 55)
drop50   = map50_fp32   - map50_int8
drop5095 = map5095_fp32 - map5095_int8
print(f"{'PT→INT8 하락폭':<18} {drop50:>+10.4f} {drop5095:>+10.4f}")
print(f"{'PT→INT8 하락률':<18} {drop50/map50_fp32*100:>9.2f}% {drop5095/map5095_fp32*100:>9.2f}%")
print("=" * 55)
