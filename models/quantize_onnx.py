"""
YOLOv8 ONNX INT8 정적 양자화 스크립트
실행: .venv_train/bin/python models/quantize_onnx.py
"""
import os
import sys
import glob
import argparse
import numpy as np
import cv2

from onnxruntime.quantization import (
    quantize_static,
    CalibrationDataReader,
    QuantType,
    QuantFormat,
)

IMG_SIZE   = 640
NUM_CALIB  = 100   # 캘리브레이션에 사용할 이미지 수


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
            inp = img.transpose(2, 0, 1)[np.newaxis]   # (1, 3, H, W)
            self._data.append({input_name: inp})

        self._iter = iter(self._data)

    def get_next(self):
        return next(self._iter, None)

    def rewind(self):
        self._iter = iter(self._data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input",  default=None, help="입력 ONNX 경로 (미지정 시 최신 자동 선택)")
    parser.add_argument("--output", default=None, help="출력 ONNX 경로 (미지정 시 _int8.onnx)")
    parser.add_argument("--calib",  default="data/synthetic/images/val", help="캘리브레이션 이미지 폴더")
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
        args.output = args.input.replace(".onnx", "_int8.onnx")

    print(f"[INFO] 입력 모델:  {args.input}")
    print(f"[INFO] 출력 모델:  {args.output}")
    print(f"[INFO] 캘리브 경로: {args.calib}")

    # 입력 이름 조회
    import onnxruntime as ort
    sess = ort.InferenceSession(args.input, providers=["CPUExecutionProvider"])
    input_name = sess.get_inputs()[0].name
    print(f"[INFO] 입력 텐서:  {input_name}")
    del sess

    # 정적 INT8 양자화 (QDQ 포맷 — ARM CPU에서 가장 효율적)
    reader = LaneCalibReader(args.calib, input_name)
    print("[QUANT] INT8 정적 양자화 시작...")
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
    print(f"  원본 크기:   {size_orig:.1f} MB")
    print(f"  INT8 크기:   {size_q:.1f} MB")
    print(f"  압축률:      {size_orig/size_q:.1f}x")
    print(f"\n출력 파일: {args.output}")


if __name__ == "__main__":
    main()
