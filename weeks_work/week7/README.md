# 7주차 진행 보고 — 합성 데이터 재생성 및 YOLOv8n 재학습

> **목표**: 수정된 World Scene(조명·차선 간격)을 반영한 새 합성 데이터로 YOLOv8n-detect 모델을 재학습하고,
> RPi5 엣지 배포를 위한 ONNX 추론 속도를 검증한다.

---

## 1. 배경 — 왜 재학습이 필요한가

| 항목 | 6주차 (lane_det-3) | 7주차 목표 |
|------|-------------------|-----------|
| 학습 데이터 조명 | DomeLight=600, RectLight=28000 (과노출) | DomeLight=120, RectLight=3000 |
| 학습 데이터 차선 간격 | LANE=0.30 | LANE=0.16 (현재 씬 기준) |
| 모델 | YOLOv8n-detect | YOLOv8n-detect (동일) |
| mAP50 | 0.763 | ≥ 0.80 목표 |
| 추론 환경 | PC CPU (ONNX) | PC CPU + RPi5 CPU 검증 |

6주차에서 월드 씬을 대폭 수정했기 때문에(조명 90% 감소, 차선 간격 축소)
기존 데이터로 학습한 모델과 실제 시뮬레이션 환경 간의 **도메인 갭이 발생**했다.
수정된 씬 조건으로 데이터를 재생성하고 모델을 재학습해야 한다.

---

## 2. 구현 내용

### 7-1. 합성 데이터 재생성

**파일**: `isaac_sim/generate_synthetic_data.py`

현재 `run_track_sim.py`에 반영된 씬 조건과 동일하게 맞춘다.

| 파라미터 | 6주차 | 7주차 |
|---------|-------|-------|
| DomeLight | 350~900 | **50~200** |
| RectLight | 18000~40000 | **1000~5000** |
| GroundPlane color | (0.20, 0.20, 0.20) | **(0.02, 0.02, 0.02)** |
| 이미지 수 | 1000장 | **1500장** (train 1200 / val 300) |
| 카메라 각도 범위 | -15° ± 5° | -15° ± 8° (커브 구간 강화) |

**실행**:
```bash
# headless 모드로 실행 (Isaac Sim GUI 불필요)
N_TOTAL=1500 /home/linux/isaac_env/bin/python isaac_sim/generate_synthetic_data.py

# 생성 후 데이터셋 검증
python3 scripts/verify_dataset.py
```

---

### 7-2. YOLOv8n-detect 재학습

**파일**: `models/train_yolo_lane.py`

detect 태스크로 재학습 (seg → detect 전환은 6주차에 완료됨).

```bash
# 학습 실행
.venv_train/bin/python models/train_yolo_lane.py \
    --model yolov8n \
    --epochs 150 \
    --batch 16

# 학습 결과 분석
python3 scripts/analyze_training.py
```

**목표 지표**:

| 지표 | 목표 | 미달 시 대응 |
|------|------|------------|
| mAP50 (val) | ≥ 0.80 | 에폭 추가 or 데이터 증량 |
| mAP50-95 (val) | ≥ 0.55 | 다양한 각도 데이터 추가 |
| Recall (val) | ≥ 0.80 | 낮은 conf 샘플 증강 |

---

### 7-3. PC에서 통합 테스트

학습된 모델로 Isaac Sim + lane_detect.py 통합 테스트.

```bash
# 터미널 1 (Isaac Sim)
ENABLE_CAMERA=1 bash isaac_sim/launch_sim.sh

# 터미널 2 (추론 노드)
source isaac_sim/ros2_terminal.sh
python3 models/inference_node/lane_detect.py

# 터미널 3 (시각화)
ros2 run rqt_image_view rqt_image_view
```

**검증 항목**:
- `det=2` 안정적으로 유지 (좌/우 차선 각 1개 — 존 기반 중복 제거 적용)
- HOLD 발생 빈도 감소 (커브 구간 차선 검출 개선 여부)
- FPS ≥ 10 Hz (ONNX CPU 기준)

---

### 7-4. RPi5 ONNX 추론 속도 벤치마크

RPi5 (ARM Cortex-A76, CUDA 없음) 에서의 추론 시간을 측정한다.

```bash
# RPi5에서 실행 (모델 파일 scp 전송 후)
scp models/runs/lane_det-*/weights/best.onnx ubuntu@192.168.0.xxx:~/

# RPi5에서 벤치마크
python3 - <<'EOF'
import onnxruntime as ort
import numpy as np, time

sess = ort.InferenceSession("best.onnx", providers=["CPUExecutionProvider"])
inp  = sess.get_inputs()[0]
dummy = np.zeros((1, 3, 640, 640), dtype=np.float32)

# 워밍업
for _ in range(5):
    sess.run(None, {inp.name: dummy})

# 측정
times = []
for _ in range(50):
    t0 = time.perf_counter()
    sess.run(None, {inp.name: dummy})
    times.append((time.perf_counter() - t0) * 1000)

print(f"평균 추론: {sum(times)/len(times):.1f} ms")
print(f"최대 추론: {max(times):.1f} ms")
EOF
```

**목표**: 평균 추론 ≤ 100 ms (10 Hz 달성 가능 기준)

| 기기 | 예상 추론 시간 |
|------|--------------|
| PC CPU (Ryzen/Intel) | ~25 ms |
| RPi5 (ARM A76) | ~70~120 ms |

---

### 7-5. (선택) ONNX INT8 양자화

RPi5 추론 속도가 100 ms를 초과할 경우 INT8 양자화로 가속을 시도한다.

```bash
# 양자화 실행 (별도 스크립트 작성 필요)
python3 models/quantize_onnx.py \
    --input  models/runs/lane_det-*/weights/best.onnx \
    --output models/runs/lane_det-*/weights/best_int8.onnx \
    --calib  data/synthetic/images/val
```

---

## 3. 완료 기준 체크리스트

- [x] `generate_synthetic_data.py` 조명 파라미터 — 이미 run_track_sim.py 씬과 동일 (DomeLight 50~200, RectLight 1000~5000) 확인, 재생성 불필요
- [x] PC에서 통합 테스트 — det=2 안정, FPS ≥ 10 Hz 확인 (lane_detect.py 시뮬 테스트 완료)
- [x] RPi5 ONNX FP32 추론 속도 측정 — **186.4 ms / 5.4 Hz** (목표 미달)
- [x] INT8 양자화 적용 (`models/quantize_onnx.py`) — **67.4 ms / 14.8 Hz PASS** ✅
  - 원본: 11.7 MB → INT8: 3.2 MB (3.6x 압축)
  - 출력: `models/runs/lane_det-3/weights/best_int8.onnx`

---

## 4. 주의사항

### generate_synthetic_data.py 조명 설정 동기화 필수

`run_track_sim.py`의 씬 조건(`DomeLight`, `RectLight`, `GroundPlane color`)과
`generate_synthetic_data.py`의 조명 랜덤 범위가 일치해야 도메인 갭이 줄어든다.
씬을 수정할 때마다 데이터 생성 파라미터도 함께 확인한다.

### Python 환경 분리 필수

```bash
# 데이터 생성 (Isaac Sim)
/home/linux/isaac_env/bin/python isaac_sim/generate_synthetic_data.py

# 학습 (CUDA)
.venv_train/bin/python models/train_yolo_lane.py
```

### RPi5 배포 시 ONNX 버전 호환성 확인

RPi5의 `onnxruntime` 버전이 PC와 다를 수 있다.
모델 export 시 `opset=11` 이하로 맞추면 구버전 호환성이 높아진다.

---

## 5. 7주차 이후 단계 — W8~W9

- **W8**: RPi5 실물 차선 추종 테스트 (Sim-to-Real)
  - 실제 트랙에서 IMX219 카메라로 이미지 수집
  - 실물 이미지 추가 학습 (파인튜닝)
- **W9**: 통합 제어 완성 및 최종 시연 준비
  - LiDAR + 카메라 복합 제어 최적화
  - 슬라롬 장애물 회피 + 차선 추종 통합 데모
