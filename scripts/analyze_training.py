#!/usr/bin/env python3
"""
analyze_training.py — YOLOv8 학습 결과 분석
=============================================
models/runs/lane_seg_* 폴더의 results.csv 를 파싱하여
손실 곡선·정밀도·재현율 그래프를 저장하고 요약 지표를 출력한다.

실행:
  python3 scripts/analyze_training.py
  python3 scripts/analyze_training.py --run models/runs/lane_seg_2
"""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

MAP50_TARGET    = 0.80
MAP5095_TARGET  = 0.55


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default=None,
                    help="특정 run 폴더 지정 (기본값: 가장 최근 lane_seg_*)")
    return ap.parse_args()


def find_latest_run(project_dir: Path) -> Path | None:
    runs = sorted(project_dir.glob("lane_seg*"),
                  key=lambda p: p.stat().st_mtime)
    return runs[-1] if runs else None


def load_results(run_dir: Path):
    """results.csv → dict[column_name, list[float]]."""
    csv_path = run_dir / "results.csv"
    if not csv_path.exists():
        print(f"[ERROR] results.csv 없음: {csv_path}")
        sys.exit(1)

    import csv
    rows = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k.strip(): v.strip() for k, v in row.items()})

    if not rows:
        print("[ERROR] results.csv 가 비어 있습니다.")
        sys.exit(1)

    # 컬럼명 정규화 (YOLOv8 버전마다 다를 수 있음)
    data = {k: [] for k in rows[0]}
    for row in rows:
        for k, v in row.items():
            try:
                data[k].append(float(v))
            except ValueError:
                data[k].append(0.0)
    return data


def find_column(data: dict, *candidates: str) -> str | None:
    """대소문자 무시하고 후보 컬럼명 중 존재하는 것을 반환."""
    lower_map = {k.lower(): k for k in data}
    for c in candidates:
        if c.lower() in lower_map:
            return lower_map[c.lower()]
    return None


def print_summary(data: dict, run_dir: Path):
    """마지막 에폭 기준 주요 지표 출력."""
    col_map50     = find_column(data, "metrics/mAP50(B)", "metrics/mAP50(M)",
                                "metrics/mAP_0.5", "mAP50")
    col_map5095   = find_column(data, "metrics/mAP50-95(B)", "metrics/mAP50-95(M)",
                                "metrics/mAP_0.5:0.95", "mAP50-95")
    col_precision = find_column(data, "metrics/precision(B)", "metrics/precision(M)",
                                "metrics/precision", "precision")
    col_recall    = find_column(data, "metrics/recall(B)", "metrics/recall(M)",
                                "metrics/recall", "recall")

    epochs = len(data.get("epoch", data[list(data.keys())[0]]))

    print("\n" + "=" * 55)
    print(f"학습 결과: {run_dir.name}  ({epochs} 에폭)")
    print("=" * 55)

    def _last(col):
        return data[col][-1] if col and col in data else None

    map50   = _last(col_map50)
    map5095 = _last(col_map5095)
    prec    = _last(col_precision)
    recall  = _last(col_recall)

    def _fmt(v, target=None):
        if v is None:
            return "N/A"
        s = f"{v:.4f}"
        if target is not None:
            s += f"  {'✅' if v >= target else '⚠️  목표: ≥' + str(target)}"
        return s

    print(f"  mAP50        : {_fmt(map50,   MAP50_TARGET)}")
    print(f"  mAP50-95     : {_fmt(map5095, MAP5095_TARGET)}")
    print(f"  Precision    : {_fmt(prec)}")
    print(f"  Recall       : {_fmt(recall)}")

    best_pt   = run_dir / "weights" / "best.pt"
    best_onnx = run_dir / "weights" / "best.onnx"
    print(f"\n  best.pt  : {'있음' if best_pt.exists()   else '없음'}")
    print(f"  best.onnx: {'있음' if best_onnx.exists() else '없음 — export-only 로 변환 필요'}")

    if map50 is not None and map50 < MAP50_TARGET:
        print("\n[HINT] mAP50 목표 미달 — 권장 조치:")
        print("  1. 에폭 추가: python3 models/train_yolo_lane.py --resume --epochs 200")
        print("  2. 데이터 추가: generate_synthetic_data.py 의 N_TOTAL=1000 으로 변경 후 재생성")
        print("  3. 더 큰 모델: python3 models/train_yolo_lane.py --model yolov8s-seg")


def plot_curves(data: dict, out_dir: Path):
    """손실 곡선 + 정밀도/재현율/mAP 그래프 저장."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib 미설치 — 그래프 생략 (pip install matplotlib)")
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    epochs = list(range(1, len(data[list(data.keys())[0]]) + 1))

    # ── 손실 곡선 ─────────────────────────────────────────────────────────────
    loss_cols = [c for c in data if "loss" in c.lower()]
    if loss_cols:
        fig, ax = plt.subplots(figsize=(10, 5))
        for col in loss_cols:
            label = col.replace("train/", "train ").replace("val/", "val ")
            ax.plot(epochs, data[col], label=label)
        ax.set_xlabel("Epoch")
        ax.set_ylabel("Loss")
        ax.set_title("Training & Validation Loss")
        ax.legend()
        ax.grid(True, alpha=0.3)
        path = out_dir / "loss_curves.png"
        fig.savefig(path, dpi=120, bbox_inches="tight")
        plt.close(fig)
        print(f"  저장: {path}")

    # ── 정밀도·재현율·mAP ─────────────────────────────────────────────────────
    metric_cols = {
        "mAP50":    find_column(data, "metrics/mAP50(B)", "metrics/mAP50(M)", "mAP50"),
        "mAP50-95": find_column(data, "metrics/mAP50-95(B)", "metrics/mAP50-95(M)", "mAP50-95"),
        "Precision":find_column(data, "metrics/precision(B)", "metrics/precision(M)"),
        "Recall":   find_column(data, "metrics/recall(B)", "metrics/recall(M)"),
    }
    metric_cols = {k: v for k, v in metric_cols.items() if v}

    if metric_cols:
        fig, ax = plt.subplots(figsize=(10, 5))
        for label, col in metric_cols.items():
            ax.plot(epochs, data[col], label=label)
        ax.axhline(MAP50_TARGET,   color="gray", linestyle="--", alpha=0.6,
                   label=f"mAP50 목표 ({MAP50_TARGET})")
        ax.set_xlabel("Epoch")
        ax.set_ylabel("Metric")
        ax.set_title("Validation Metrics")
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, 1.05)
        path = out_dir / "precision_recall.png"
        fig.savefig(path, dpi=120, bbox_inches="tight")
        plt.close(fig)
        print(f"  저장: {path}")

    # ── 요약 텍스트 ────────────────────────────────────────────────────────────
    col_map50 = find_column(data, "metrics/mAP50(B)", "metrics/mAP50(M)", "mAP50")
    if col_map50:
        best_epoch = int(data[col_map50].index(max(data[col_map50]))) + 1
        best_map50 = max(data[col_map50])
        summary = (
            f"Best epoch : {best_epoch}/{len(epochs)}\n"
            f"Best mAP50 : {best_map50:.4f}\n"
            f"Target     : {MAP50_TARGET}\n"
            f"Pass       : {'Yes' if best_map50 >= MAP50_TARGET else 'No'}\n"
        )
        txt_path = out_dir / "summary.txt"
        txt_path.write_text(summary)
        print(f"  저장: {txt_path}")


def main():
    args = parse_args()
    project_dir = ROOT / "models" / "runs"

    if args.run:
        run_dir = Path(args.run)
    else:
        run_dir = find_latest_run(project_dir)
        if run_dir is None:
            print("[ERROR] 학습된 run 폴더 없음")
            print("  먼저: python3 models/train_yolo_lane.py")
            sys.exit(1)

    print(f"[INFO] 분석 대상: {run_dir}")
    data    = load_results(run_dir)
    out_dir = run_dir / "analysis"

    print_summary(data, run_dir)

    print(f"\n[INFO] 그래프 저장 중: {out_dir}")
    plot_curves(data, out_dir)
    print("[INFO] 분석 완료")


if __name__ == "__main__":
    main()
