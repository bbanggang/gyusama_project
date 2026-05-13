"""
train_yolo_lane.py — YOLOv8-detect 차선 인식 모델 학습
=======================================================
Isaac Replicator 로 생성한 합성 데이터셋으로 YOLOv8s-detect 를 학습하고
ONNX 모델을 추출한다.

사전 준비:
  pip install ultralytics

실행:
  python3 models/train_yolo_lane.py
  python3 models/train_yolo_lane.py --resume
  python3 models/train_yolo_lane.py --model yolov8m   # 더 큰 모델
  python3 models/train_yolo_lane.py --epochs 150 --batch 32

결과:
  models/runs/lane_det_<v>/weights/best.pt    ← PyTorch 최적 가중치
  models/runs/lane_det_<v>/weights/best.onnx  ← 추론용 ONNX
"""

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def parse_args():
    ap = argparse.ArgumentParser(description="YOLOv8-detect 차선 인식 학습")
    ap.add_argument("--data",    default=str(ROOT / "data" / "synthetic" / "dataset.yaml"))
    ap.add_argument("--model",   default="yolov8s",
                    help="베이스 모델 (yolov8n / yolov8s / yolov8m)")
    ap.add_argument("--epochs",  type=int, default=100)
    ap.add_argument("--batch",   type=int, default=16)
    ap.add_argument("--imgsz",   type=int, default=640)
    ap.add_argument("--device",  default="0")
    ap.add_argument("--resume",  action="store_true")
    ap.add_argument("--export-only", action="store_true")
    return ap.parse_args()


def validate_dataset(data_yaml: str):
    p = Path(data_yaml)
    if not p.exists():
        print(f"[ERROR] dataset.yaml 없음: {p}")
        sys.exit(1)

    import yaml
    with open(p) as f:
        cfg = yaml.safe_load(f)

    base = Path(cfg.get("path", p.parent))
    for split in ("train", "val"):
        img_dir = base / cfg.get(split, f"images/{split}")
        imgs = list(img_dir.glob("*.png")) + list(img_dir.glob("*.jpg"))
        if not imgs:
            print(f"[WARN] {split} 이미지 없음: {img_dir}")
        else:
            print(f"[INFO] {split}: {len(imgs)}개 이미지")


def find_latest_run(project_dir: Path) -> Path | None:
    runs = sorted(project_dir.glob("lane_det*/weights/best.pt"),
                  key=lambda p: p.stat().st_mtime)
    return runs[-1] if runs else None


def export_onnx(weights: Path):
    from ultralytics import YOLO
    model = YOLO(str(weights))
    out = model.export(
        format="onnx",
        imgsz=640,
        dynamic=False,
        opset=17,
        simplify=True,
    )
    print(f"[INFO] ONNX 내보내기 완료: {out}")
    return out


def train(args):
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[ERROR] ultralytics 미설치. pip install ultralytics")
        sys.exit(1)

    project_dir = ROOT / "models" / "runs"
    project_dir.mkdir(parents=True, exist_ok=True)

    if args.resume:
        latest = find_latest_run(project_dir)
        if latest is None:
            print("[WARN] 이전 체크포인트 없음 — 새로 시작")
            resume_path = f"{args.model}.pt"
        else:
            last_pt = latest.parent / "last.pt"
            resume_path = str(last_pt if last_pt.exists() else latest)
            print(f"[INFO] 이어서 학습: {resume_path}")
        model = YOLO(resume_path)
    else:
        model = YOLO(f"{args.model}.pt")

    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=str(project_dir),
        name="lane_det",
        exist_ok=False,
        resume=args.resume,

        # 차선 인식 최적화 하이퍼파라미터
        lr0=0.01,
        lrf=0.001,
        momentum=0.937,
        weight_decay=5e-4,
        warmup_epochs=3,
        box=7.5,
        cls=0.5,
        dfl=1.5,
        iou=0.45,
        conf=0.001,

        # 데이터 증강 (합성 데이터 도메인 갭 보상)
        hsv_h=0.015,
        hsv_s=0.5,
        hsv_v=0.4,
        degrees=5.0,
        translate=0.05,
        scale=0.3,
        shear=0.0,
        perspective=0.0001,
        flipud=0.0,
        fliplr=0.3,
        mosaic=0.8,
        mixup=0.1,

        save=True,
        save_period=10,
        plots=True,
        verbose=True,
    )

    print(f"\n[INFO] 학습 완료")
    print(f"  best.pt : {results.save_dir}/weights/best.pt")

    best_pt = Path(results.save_dir) / "weights" / "best.pt"
    if best_pt.exists():
        export_onnx(best_pt)
    else:
        print(f"[WARN] best.pt 없음: {best_pt}")

    return results


def main():
    args = parse_args()

    if args.export_only:
        project_dir = ROOT / "models" / "runs"
        latest = find_latest_run(project_dir)
        if latest is None:
            print("[ERROR] 학습된 가중치 없음")
            sys.exit(1)
        export_onnx(latest)
        return

    validate_dataset(args.data)
    train(args)


if __name__ == "__main__":
    main()
