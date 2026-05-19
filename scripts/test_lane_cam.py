"""
차선 검출 시각화 테스트 (ROS2 불필요)
──────────────────────────────────────
  # 합성 val 이미지 슬라이드쇼 (기본)
  .venv_train/bin/python scripts/test_lane_cam.py

  # 특정 이미지 폴더
  .venv_train/bin/python scripts/test_lane_cam.py --imgdir data/synthetic/images/val

  # 비디오 파일
  .venv_train/bin/python scripts/test_lane_cam.py --video path/to/video.mp4

  # USB 웹캠
  .venv_train/bin/python scripts/test_lane_cam.py --cam 0

키 조작:
  Space / Right  다음 이미지 (이미지 모드)
  Left           이전 이미지 (이미지 모드)
  p              일시정지 / 재개
  q / ESC        종료
"""

import argparse
import glob
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# ── 프로젝트 루트 기준 ONNX 자동 탐색 ─────────────────────────────────────────
ROOT = Path(__file__).resolve().parent.parent

# ── 파라미터 (lane_detect.py 와 동일) ─────────────────────────────────────────
IMG_W, IMG_H        = 640, 480
YOLO_SZ             = 640
CONF_THRESH         = 0.12
NMS_IOU_THR         = 0.45
LANE_ROI_TOP_RATIO  = 0.50


# ─────────────────────────────────────────────────────────────────────────────
def find_onnx() -> str | None:
    runs_dir = ROOT / "models" / "runs"
    candidates = sorted(
        list(runs_dir.glob("lane_det*/weights/best.onnx")) +
        list(runs_dir.glob("lane_seg*/weights/best.onnx")),
        key=lambda p: p.stat().st_mtime,
    )
    return str(candidates[-1]) if candidates else None


def nms(cx, cy, w, h, scores, iou_thr=NMS_IOU_THR):
    x1 = cx - w / 2;  y1 = cy - h / 2
    x2 = cx + w / 2;  y2 = cy + h / 2
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep  = []
    while order.size:
        i = order[0];  keep.append(i)
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


def infer(sess, input_name, out_names, bgr):
    img = cv2.resize(bgr, (YOLO_SZ, YOLO_SZ))
    inp = img[:, :, ::-1].astype(np.float32) / 255.0
    inp = np.ascontiguousarray(inp.transpose(2, 0, 1)[None])

    t0  = time.perf_counter()
    out = sess.run(out_names, {input_name: inp})[0][0]   # (5, 8400)
    ms  = (time.perf_counter() - t0) * 1000.0

    scores  = out[4]
    roi_top = YOLO_SZ * LANE_ROI_TOP_RATIO
    mask    = (scores > CONF_THRESH) & (out[1] > roi_top)
    idx     = np.where(mask)[0]

    boxes = []
    if idx.size:
        keep = nms(out[0][idx], out[1][idx], out[2][idx], out[3][idx], scores[idx])
        h0, w0 = bgr.shape[:2]
        sx, sy = w0 / YOLO_SZ, h0 / YOLO_SZ
        zone_best: dict[int, dict] = {}
        for i in idx[keep]:
            b = {"cx": float(out[0][i]*sx), "cy": float(out[1][i]*sy),
                 "w":  float(out[2][i]*sx), "h":  float(out[3][i]*sy),
                 "conf": float(scores[i])}
            z = 0 if b["cx"] < w0 / 2 else 1
            if z not in zone_best or b["conf"] > zone_best[z]["conf"]:
                zone_best[z] = b
        boxes = list(zone_best.values())
    return boxes, ms


def compute_offset(boxes, img_w):
    if not boxes:
        return None, {}
    boxes = sorted(boxes, key=lambda b: b["cx"])
    if len(boxes) == 1:
        cx = boxes[0]["cx"]
        mid = cx + img_w * 0.25 if cx < img_w / 2 else cx - img_w * 0.25
        return float(np.clip((mid - img_w/2) / (img_w/2), -1, 1)), {
            "left_cx":  cx if cx < img_w/2 else None,
            "right_cx": cx if cx >= img_w/2 else None,
            "mid_cx":   mid,
        }
    lx, rx = boxes[0]["cx"], boxes[-1]["cx"]
    mid = (lx + rx) / 2
    return float(np.clip((mid - img_w/2) / (img_w/2), -1, 1)), {
        "left_cx": lx, "right_cx": rx, "mid_cx": mid}


def draw(bgr, boxes, info, offset, ms, label=""):
    vis = bgr.copy()
    roi_y = int(bgr.shape[0] * LANE_ROI_TOP_RATIO)

    # ROI 경계선 (반투명 회색)
    overlay = vis.copy()
    cv2.rectangle(overlay, (0, 0), (vis.shape[1], roi_y), (40, 40, 40), -1)
    cv2.addWeighted(overlay, 0.35, vis, 0.65, 0, vis)
    cv2.line(vis, (0, roi_y), (vis.shape[1], roi_y), (100, 100, 100), 1)

    # 검출 박스
    for b in boxes:
        cx, cy = int(b["cx"]), int(b["cy"])
        hw, hh = int(b.get("w", 40)/2), int(b.get("h", 40)/2)
        cv2.rectangle(vis, (cx-hw, cy-hh), (cx+hw, cy+hh), (0, 255, 0), 2)
        cv2.circle(vis, (cx, cy), 5, (0, 255, 0), -1)
        cv2.putText(vis, f"{b['conf']:.2f}", (cx-hw, cy-hh-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    h, w = vis.shape[:2]
    # 좌/우 차선 수직선
    if info.get("left_cx") is not None:
        cv2.line(vis, (int(info["left_cx"]), roi_y), (int(info["left_cx"]), h), (255, 80, 80), 2)
    if info.get("right_cx") is not None:
        cv2.line(vis, (int(info["right_cx"]), roi_y), (int(info["right_cx"]), h), (80, 80, 255), 2)
    # 목표 중심선 (노랑)
    if "mid_cx" in info:
        mx = int(info["mid_cx"])
        cv2.line(vis, (mx, roi_y), (mx, h), (0, 220, 220), 2)
    # 이미지 중심선 (흰색)
    cv2.line(vis, (w//2, h-50), (w//2, h), (220, 220, 220), 1)

    # 조향 바 (하단)
    bar_cx = w // 2
    bar_y  = h - 18
    bar_hw = w // 3
    cv2.rectangle(vis, (bar_cx - bar_hw, bar_y - 8), (bar_cx + bar_hw, bar_y + 8), (50,50,50), -1)
    if offset is not None:
        fill_x = int(bar_cx + offset * bar_hw)
        color  = (0, 200, 255) if abs(offset) < 0.3 else (0, 100, 255) if abs(offset) < 0.6 else (0, 0, 255)
        cv2.rectangle(vis, (min(bar_cx, fill_x), bar_y-6), (max(bar_cx, fill_x), bar_y+6), color, -1)
    cv2.line(vis, (bar_cx, bar_y-10), (bar_cx, bar_y+10), (200,200,200), 1)

    # 텍스트 HUD
    det_count = len(boxes)
    off_str   = f"{offset:+.3f}" if offset is not None else " None"
    status    = label or ("LANE" if det_count else "MISS")
    cv2.putText(vis, f"{status}  off={off_str}  det={det_count}  {ms:.0f}ms",
                (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
    # 범례
    cv2.putText(vis, "L=left  R=right  Y=target  W=center",
                (8, h-30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180,180,180), 1)
    return vis


# ─────────────────────────────────────────────────────────────────────────────
def run_images(sess, input_name, out_names, img_files, delay_ms=1200):
    idx   = 0
    pause = False
    print(f"이미지 {len(img_files)}장  |  Space/→: 다음  ←: 이전  p: 일시정지  q: 종료")

    while True:
        path = img_files[idx]
        bgr  = cv2.imread(path)
        if bgr is None:
            idx = (idx + 1) % len(img_files)
            continue

        bgr   = cv2.resize(bgr, (IMG_W, IMG_H))
        boxes, ms = infer(sess, input_name, out_names, bgr)
        off, info = compute_offset(boxes, IMG_W)
        vis       = draw(bgr, boxes, info, off, ms)

        fname = Path(path).name
        cv2.putText(vis, f"[{idx+1}/{len(img_files)}] {fname}",
                    (8, IMG_H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
        cv2.imshow("Lane Detection", vis)

        wait = 0 if pause else delay_ms
        key  = cv2.waitKey(wait) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key in (ord(' '), 83):          # Space / Right
            idx = (idx + 1) % len(img_files)
        elif key == 81:                       # Left
            idx = (idx - 1) % len(img_files)
        elif key == ord('p'):
            pause = not pause
            print("일시정지" if pause else "재개")
        elif not pause:
            idx = (idx + 1) % len(img_files)


def run_stream(sess, input_name, out_names, cap):
    pause = False
    print("p: 일시정지  q: 종료")
    while cap.isOpened():
        if not pause:
            ret, bgr = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            bgr   = cv2.resize(bgr, (IMG_W, IMG_H))
            boxes, ms = infer(sess, input_name, out_names, bgr)
            off, info = compute_offset(boxes, IMG_W)
            vis       = draw(bgr, boxes, info, off, ms)
            cv2.imshow("Lane Detection", vis)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('p'):
            pause = not pause


# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx",   default=None,
                        help="ONNX 모델 경로 (미지정 시 자동 탐색)")
    parser.add_argument("--imgdir", default=None,
                        help="이미지 폴더 경로")
    parser.add_argument("--video",  default=None,
                        help="동영상 파일 경로")
    parser.add_argument("--cam",    type=int, default=None,
                        help="웹캠 인덱스 (예: 0)")
    parser.add_argument("--delay",  type=int, default=800,
                        help="이미지 모드 전환 간격 ms (기본 800)")
    args = parser.parse_args()

    # ONNX 모델
    onnx_path = args.onnx or find_onnx()
    if onnx_path is None:
        print("[ERROR] ONNX 모델을 찾을 수 없습니다. --onnx 로 직접 지정하세요.")
        sys.exit(1)
    print(f"[모델] {onnx_path}")

    import onnxruntime as ort
    sess       = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    input_name = sess.get_inputs()[0].name
    out_names  = [o.name for o in sess.get_outputs()]

    # 소스 결정
    if args.cam is not None:
        cap = cv2.VideoCapture(args.cam)
        if not cap.isOpened():
            print(f"[ERROR] 웹캠 {args.cam} 열기 실패")
            sys.exit(1)
        run_stream(sess, input_name, out_names, cap)
        cap.release()

    elif args.video is not None:
        cap = cv2.VideoCapture(args.video)
        if not cap.isOpened():
            print(f"[ERROR] 동영상 {args.video} 열기 실패")
            sys.exit(1)
        run_stream(sess, input_name, out_names, cap)
        cap.release()

    else:
        # 이미지 폴더 (기본값: val 셋)
        img_dir = args.imgdir or str(ROOT / "data" / "synthetic" / "images" / "val")
        files   = sorted(
            glob.glob(f"{img_dir}/*.jpg") +
            glob.glob(f"{img_dir}/*.png")
        )
        if not files:
            print(f"[ERROR] 이미지 없음: {img_dir}")
            sys.exit(1)
        print(f"[소스] 이미지 폴더: {img_dir}  ({len(files)}장)")
        run_images(sess, input_name, out_names, files, delay_ms=args.delay)

    cv2.destroyAllWindows()
    print("종료")


if __name__ == "__main__":
    main()
