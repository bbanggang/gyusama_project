"""
RPi5 ONNX 추론 속도 벤치마크
실행: python3 benchmark_rpi5.py [onnx_path]
"""
import sys
import time
import numpy as np

ONNX_PATH = sys.argv[1] if len(sys.argv) > 1 else "best.onnx"
WARMUP     = 5
RUNS       = 50
IMG_SIZE   = 640

try:
    import onnxruntime as ort
except ImportError:
    print("[ERROR] onnxruntime 미설치 → pip3 install onnxruntime")
    sys.exit(1)

print(f"onnxruntime 버전: {ort.__version__}")
print(f"사용 가능 provider: {ort.get_available_providers()}")
print(f"모델: {ONNX_PATH}\n")

sess = ort.InferenceSession(ONNX_PATH, providers=["CPUExecutionProvider"])
inp  = sess.get_inputs()[0]
print(f"입력 shape: {inp.shape}  dtype: {inp.type}")

dummy = np.random.rand(1, 3, IMG_SIZE, IMG_SIZE).astype(np.float32)

# 워밍업
print(f"워밍업 {WARMUP}회...", flush=True)
for _ in range(WARMUP):
    sess.run(None, {inp.name: dummy})

# 측정
print(f"측정 {RUNS}회...", flush=True)
times = []
for i in range(RUNS):
    t0 = time.perf_counter()
    sess.run(None, {inp.name: dummy})
    times.append((time.perf_counter() - t0) * 1000.0)
    if (i + 1) % 10 == 0:
        print(f"  {i+1}/{RUNS} 완료", flush=True)

avg = sum(times) / len(times)
mn  = min(times)
mx  = max(times)
p95 = sorted(times)[int(len(times) * 0.95)]

print("\n==============================")
print(f"  평균 추론:   {avg:.1f} ms")
print(f"  최소 추론:   {mn:.1f} ms")
print(f"  최대 추론:   {mx:.1f} ms")
print(f"  95th pct:   {p95:.1f} ms")
print(f"  예상 FPS:    {1000/avg:.1f} Hz")
print("==============================")

if avg <= 100:
    print(f"  결과: PASS — 실시간 처리 가능 (≥10 Hz)")
else:
    print(f"  결과: SLOW — INT8 양자화 검토 필요")
