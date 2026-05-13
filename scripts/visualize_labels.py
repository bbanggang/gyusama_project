#!/usr/bin/env python3
"""
visualize_labels.py — 합성 데이터 레이블 시각화
=================================================
data/synthetic/images/ 의 이미지 위에 YOLOv8-seg 레이블 폴리곤을
오버레이하여 data/synthetic/preview/ 에 저장한다.

실행:
  python3 scripts/visualize_labels.py                    # 최대 20장 (train+val)
  python3 scripts/visualize_labels.py --n 10             # 10장
  python3 scripts/visualize_labels.py --split val        # val 셋만
  python3 scripts/visualize_labels.py --show             # 화면 출력 (GUI 환경)
"""

import argparse
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent.parent

CLASS_COLORS = {
    0: (0,   200, 0,   160),   # white_lane   → 반투명 초록
    1: (0,   0,   200, 160),   # stop_line    → 반투명 빨강
}
CLASS_NAMES = {0: "white_lane", 1: "stop_line"}


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data",  default=str(ROOT / "data" / "synthetic"))
    ap.add_argument("--split", default="both", choices=["train", "val", "both"])
    ap.add_argument("--n",     type=int, default=20, help="최대 시각화 장수")
    ap.add_argument("--show",  action="store_true", help="imshow 으로 화면 출력")
    return ap.parse_args()


def draw_seg_label(img, lbl_path: Path) -> np.ndarray:
    """이미지 위에 YOLOv8-seg 레이블 폴리곤을 그린다."""
    import cv2
    h, w = img.shape[:2]
    overlay = img.copy()

    lines = [l.strip() for l in lbl_path.read_text().splitlines() if l.strip()]
    for line in lines:
        parts = line.split()
        if len(parts) < 7:
            continue
        cls_id = int(parts[0])
        coords = [float(v) for v in parts[1:]]
        pts = np.array(coords, dtype=np.float32).reshape(-1, 2)
        pts[:, 0] *= w
        pts[:, 1] *= h
        pts = pts.astype(np.int32)

        color_rgba = CLASS_COLORS.get(cls_id, (128, 128, 128, 160))
        color_bgr  = (color_rgba[2], color_rgba[1], color_rgba[0])

        cv2.fillPoly(overlay, [pts], color_bgr)
        cv2.polylines(img, [pts], isClosed=True,
                      color=(color_bgr[0]//2, color_bgr[1]//2, color_bgr[2]//2 + 100),
                      thickness=1)

    alpha = 0.45
    result = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

    # 범례
    for i, (cls_id, name) in enumerate(CLASS_NAMES.items()):
        c = CLASS_COLORS[cls_id]
        cv2.rectangle(result, (5, 5 + i * 22), (18, 18 + i * 22),
                      (c[2], c[1], c[0]), -1)
        cv2.putText(result, name, (22, 16 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return result


def collect_pairs(data_dir: Path, split: str, n: int):
    """(img_path, lbl_path) 쌍 목록 수집."""
    pairs = []
    splits = ["train", "val"] if split == "both" else [split]
    for sp in splits:
        img_dir = data_dir / "images" / sp
        lbl_dir = data_dir / "labels" / sp
        if not img_dir.exists():
            print(f"[WARN] 이미지 폴더 없음: {img_dir}")
            continue
        for img_path in sorted(img_dir.glob("*.png")):
            lbl_path = lbl_dir / (img_path.stem + ".txt")
            pairs.append((img_path, lbl_path if lbl_path.exists() else None))
    return pairs[:n]


def main():
    try:
        import cv2
    except ImportError:
        print("[ERROR] opencv-python 미설치: pip install opencv-python-headless")
        sys.exit(1)

    args    = parse_args()
    data_dir = Path(args.data)
    out_dir  = data_dir / "preview"
    out_dir.mkdir(exist_ok=True)

    pairs = collect_pairs(data_dir, args.split, args.n)
    if not pairs:
        print("[ERROR] 이미지 없음 — generate_synthetic_data.py 를 먼저 실행하세요.")
        sys.exit(1)

    print(f"[INFO] 시각화 대상: {len(pairs)}장 → {out_dir}")
    for img_path, lbl_path in pairs:
        img = cv2.imread(str(img_path))
        if img is None:
            print(f"[WARN] 읽기 실패: {img_path.name}")
            continue

        if lbl_path and lbl_path.exists():
            vis = draw_seg_label(img, lbl_path)
            n_labels = sum(1 for l in lbl_path.read_text().splitlines() if l.strip())
            tag = f"{img_path.parent.name}/{img_path.name}  ({n_labels} instances)"
        else:
            vis = img.copy()
            tag = f"{img_path.parent.name}/{img_path.name}  [레이블 없음]"

        cv2.putText(vis, tag, (5, img.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        out_path = out_dir / f"{img_path.parent.name}_{img_path.name}"
        cv2.imwrite(str(out_path), vis)

        if args.show:
            cv2.imshow("label preview", vis)
            key = cv2.waitKey(0)
            if key == ord("q"):
                break

    if args.show:
        cv2.destroyAllWindows()

    print(f"[INFO] 완료 — {out_dir} 확인")
    print(f"  파일 탐색기나 다음 명령으로 확인:")
    print(f"  eog {out_dir}/*.png   # 이미지 뷰어")
    print(f"  ls -lh {out_dir}/")


if __name__ == "__main__":
    main()
