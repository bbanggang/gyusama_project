#!/usr/bin/env python3
"""
verify_dataset.py — 합성 데이터셋 무결성 검증
===============================================
generate_synthetic_data.py 실행 후 바로 실행하여
이미지·레이블 파일 대응 여부와 레이블 형식을 확인한다.

실행:
  python3 scripts/verify_dataset.py
  python3 scripts/verify_dataset.py --data data/synthetic/dataset.yaml
"""

import argparse
import os
import sys
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parent.parent


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(ROOT / "data" / "synthetic" / "dataset.yaml"))
    return ap.parse_args()


def verify_split(img_dir: Path, lbl_dir: Path, split: str) -> bool:
    imgs = sorted(img_dir.glob("*.png")) + sorted(img_dir.glob("*.jpg"))
    lbls = sorted(lbl_dir.glob("*.txt"))

    img_stems = {p.stem for p in imgs}
    lbl_stems = {p.stem for p in lbls}

    ok = True
    print(f"\n[{split.upper()}]")
    print(f"  이미지: {len(imgs)}장   레이블: {len(lbls)}개")

    # 이미지 수 확인
    if len(imgs) == 0:
        print(f"  [ERROR] 이미지 없음: {img_dir}")
        return False

    # 1:1 대응 확인
    missing_lbl = img_stems - lbl_stems
    missing_img = lbl_stems - img_stems
    if missing_lbl:
        print(f"  [WARN]  레이블 없는 이미지 {len(missing_lbl)}개: {sorted(missing_lbl)[:5]} ...")
        ok = False
    if missing_img:
        print(f"  [WARN]  이미지 없는 레이블 {len(missing_img)}개: {sorted(missing_img)[:5]} ...")
        ok = False

    # 레이블 내용 샘플 검사
    empty_lbl  = 0
    bad_format = 0
    class_counts = {}
    for p in lbls:
        lines = [l.strip() for l in p.read_text().splitlines() if l.strip()]
        if not lines:
            empty_lbl += 1
            continue
        for line in lines:
            parts = line.split()
            if len(parts) < 7:  # class_id + 최소 3점(x,y)*3=6
                bad_format += 1
                continue
            try:
                cls_id = int(parts[0])
                coords = [float(v) for v in parts[1:]]
                if not all(0.0 <= v <= 1.0 for v in coords):
                    bad_format += 1
                    continue
                class_counts[cls_id] = class_counts.get(cls_id, 0) + 1
            except ValueError:
                bad_format += 1

    print(f"  빈 레이블 파일: {empty_lbl}개")
    if bad_format:
        print(f"  [WARN]  잘못된 레이블 행: {bad_format}개")
        ok = False

    print(f"  클래스별 검출 인스턴스: {class_counts}")
    if 0 not in class_counts:
        print("  [WARN]  white_lane(0) 인스턴스 없음 — 씬 구성 또는 시맨틱 레이블 확인 필요")
        ok = False

    if ok:
        print(f"  [OK]    {split} 검증 통과")
    return ok


def verify_image_size(img_dir: Path, expected_w: int, expected_h: int):
    """이미지 해상도 샘플 검사 (최대 10장)."""
    try:
        import cv2
    except ImportError:
        return  # opencv 없으면 건너뜀

    imgs = list(img_dir.glob("*.png"))[:10]
    wrong = []
    for p in imgs:
        img = cv2.imread(str(p))
        if img is None:
            wrong.append((p.name, "읽기 실패"))
            continue
        h, w = img.shape[:2]
        if w != expected_w or h != expected_h:
            wrong.append((p.name, f"{w}x{h}"))
    if wrong:
        print(f"  [WARN]  해상도 불일치 (기대값 {expected_w}x{expected_h}):")
        for name, info in wrong:
            print(f"          {name}: {info}")
    else:
        print(f"  해상도 {expected_w}x{expected_h} 확인 (샘플 {len(imgs)}장)")


def main():
    args  = parse_args()
    yaml_path = Path(args.data)

    if not yaml_path.exists():
        print(f"[ERROR] dataset.yaml 없음: {yaml_path}")
        print("  먼저 generate_synthetic_data.py 를 실행하세요.")
        sys.exit(1)

    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)

    base   = Path(cfg.get("path", yaml_path.parent))
    nc     = cfg.get("nc", 0)
    names  = cfg.get("names", {})

    print("=" * 55)
    print(f"dataset.yaml: {yaml_path}")
    print(f"base path   : {base}")
    print(f"nc          : {nc}  → {names}")
    print("=" * 55)

    all_ok = True
    for split in ("train", "val"):
        img_dir = base / cfg.get(split, f"images/{split}")
        lbl_dir = base / "labels" / split

        if not img_dir.exists():
            print(f"\n[{split.upper()}] [ERROR] 이미지 폴더 없음: {img_dir}")
            all_ok = False
            continue

        ok = verify_split(img_dir, lbl_dir, split)
        if ok:
            verify_image_size(img_dir, 640, 480)
        all_ok = all_ok and ok

    print("\n" + "=" * 55)
    if all_ok:
        print("[PASS] 데이터셋 검증 완료 — 학습 진행 가능")
        print(f"  python3 models/train_yolo_lane.py --epochs 100 --batch 16")
    else:
        print("[FAIL] 검증 실패 — 위 경고 확인 후 데이터 재생성 권장")
        sys.exit(1)


if __name__ == "__main__":
    main()
