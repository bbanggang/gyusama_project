"""
YOLOv8 ONNX INT8 양자화 스크립트
  --mode static   : QDQ 정적 양자화 (캘리브레이션 필요, per_channel=False → 신뢰도 붕괴 이슈)
  --mode dynamic  : 동적 양자화 (가중치만 INT8, 활성화 FP32 → 신뢰도 보존)

실행 예시:
  .venv_train/bin/python models/quantize_onnx.py --mode dynamic
"""
import os
import sys
import glob
import argparse
import numpy as np
import cv2

from onnxruntime.quantization import (
    quantize_static,
    quantize_dynamic,
    CalibrationDataReader,
    QuantType,
    QuantFormat,
)

IMG_SIZE   = 640
NUM_CALIB  = 100


class LaneCalibReader(CalibrationDataReader):
    def __init__(self, calib_dir: str, input_name: str):
        exts = ("*.png", "*.jpg", "*.jpeg")
        files = []
        for ext in exts:
            files.extend(glob.glob(os.path.join(calib_dir, ext)))
        files = sorted(files)[:NUM_CALIB]
        if not files:
            raise FileNotFoundError(f"캘리브레이션 이미지 없음: {calib_dir}")
        print(f"[CALIB] {len(files)}장 로드: {calib_dir}")

        self._data = []
        for f in files:
            img = cv2.imread(f)
            img = cv2.resize(img, (IMG_SIZE, IMG_SIZE))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            inp = img.transpose(2, 0, 1)[np.newaxis]
            self._data.append({input_name: inp})

        self._iter = iter(self._data)

    def get_next(self):
        return next(self._iter, None)

    def rewind(self):
        self._iter = iter(self._data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input",  default=None,
                        help="입력 ONNX 경로 (미지정 시 최신 자동 선택)")
    parser.add_argument("--output", default=None,
                        help="출력 ONNX 경로 (미지정 시 자동 결정)")
    parser.add_argument("--calib",  default="data/synthetic/images/val",
                        help="캘리브레이션 이미지 폴더 (static 모드 전용)")
    parser.add_argument("--mode",   default="dynamic",
                        choices=["static", "dynamic"],
                        help="양자화 모드 (default: dynamic)")
    args = parser.parse_args()

    # 입력 모델 자동 선택
    if args.input is None:
        candidates = sorted(
            glob.glob("models/runs/*/weights/best.onnx"),
            key=os.path.getmtime,
        )
        if not candidates:
            print("[ERROR] best.onnx를 찾을 수 없습니다.")
            sys.exit(1)
        args.input = candidates[-1]

    if args.output is None:
        suffix = "_dynamic_int8.onnx" if args.mode == "dynamic" else "_int8.onnx"
        args.output = args.input.replace(".onnx", suffix)

    print(f"[INFO] 모드:       {args.mode}")
    print(f"[INFO] 입력 모델:  {args.input}")
    print(f"[INFO] 출력 모델:  {args.output}")

    if args.mode == "dynamic":
        # 동적 양자화 — 가중치만 INT8 압축, 활성화는 FP32 유지
        # 신뢰도(0~1) 채널 정밀도 붕괴 없음, 캘리브레이션 불필요
        print("[QUANT] Dynamic INT8 양자화 시작...")
        quantize_dynamic(
            model_input=args.input,
            model_output=args.output,
            weight_type=QuantType.QInt8,
        )
    else:
        # 정적 QDQ 양자화 — per_channel=False 시 신뢰도 붕괴 위험
        import onnxruntime as ort
        sess = ort.InferenceSession(args.input, providers=["CPUExecutionProvider"])
        input_name = sess.get_inputs()[0].name
        del sess

        print(f"[INFO] 캘리브 경로: {args.calib}")
        reader = LaneCalibReader(args.calib, input_name)
        print("[QUANT] Static INT8 QDQ 양자화 시작...")
        quantize_static(
            model_input=args.input,
            model_output=args.output,
            calibration_data_reader=reader,
            quant_format=QuantFormat.QDQ,
            per_channel=False,
            weight_type=QuantType.QInt8,
            activation_type=QuantType.QInt8,
        )

    size_orig = os.path.getsize(args.input) / 1024 / 1024
    size_q    = os.path.getsize(args.output) / 1024 / 1024
    print(f"\n[DONE] 양자화 완료")
    print(f"  원본 크기:  {size_orig:.1f} MB")
    print(f"  양자화 크기: {size_q:.1f} MB")
    print(f"  압축률:     {size_orig/size_q:.1f}x")
    print(f"\n출력 파일: {args.output}")


if __name__ == "__main__":
    main()
