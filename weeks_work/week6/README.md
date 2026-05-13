# 6주차 진행 보고 — 합성 데이터 생성·YOLOv8 학습·추론 파이프라인 완성

> **목표**: Isaac Sim 디지털 트윈 단계(W4~W6) 마무리
> `generate_synthetic_data.py` → `train_yolo_lane.py` → `lane_detect.py`
> 세 단계의 end-to-end 파이프라인을 실제로 실행하고 차선 추종 동작을 검증한다.

---

## 1. 전체 파이프라인 개요

```
Isaac Sim (headless)
  generate_synthetic_data.py
       │  500장 RGB 이미지 + YOLOv8-seg 레이블
       ▼
  data/synthetic/
  ├── images/{train(400), val(100)}/
  ├── labels/{train(400), val(100)}/
  └── dataset.yaml
       │
       ▼
  train_yolo_lane.py  (RTX 5070 Ti, CUDA)
       │  YOLOv8n-seg  100 에폭
       ▼
  models/runs/lane_seg_*/weights/
  ├── best.pt    ← PyTorch 가중치
  └── best.onnx  ← ONNX 추론 모델
       │
       ▼
Isaac Sim (GUI) + lane_detect.py  (별도 터미널)
  /camera/image_raw → YOLO 추론 → /cmd_vel 발행
  → /lane/debug_image 시각화
```

---

## 2. 구현 내용

### 6-1. 합성 데이터 생성

**파일**: `isaac_sim/generate_synthetic_data.py`

Isaac Sim headless 모드로 AutoRace 트랙을 렌더링하며 카메라를
웨이포인트를 따라 이동시켜 RGB + 시맨틱 세그멘테이션 이미지를 자동 캡처한다.

| 항목 | 값 |
|------|-----|
| 총 이미지 | 500장 |
| train / val | 400 / 100 (8:2) |
| 해상도 | 640 × 480 |
| 카메라 높이 | 12 cm |
| 피치 각도 | -15° ± 5° (랜덤) |
| 조명 랜덤화 | DomeLight 350~900, RectLight 18000~40000 |
| 레이블 형식 | YOLOv8-seg 폴리곤 (클래스 0: lane, nc=1) |

**실행**:
```bash
# headless (권장 — Isaac Sim GUI 불필요)
/home/linux/isaac_env/bin/python isaac_sim/generate_synthetic_data.py

# GUI 확인 (디버깅용)
HEADLESS=0 /home/linux/isaac_env/bin/python isaac_sim/generate_synthetic_data.py
```

**데이터 검증** (생성 후 즉시 실행):
```bash
python3 scripts/verify_dataset.py
```

---

### 6-2. YOLOv8-seg 학습

**파일**: `models/train_yolo_lane.py`

합성 데이터로 `yolov8n-seg.pt`(COCO pre-trained)를 파인튜닝한다.
CUDA 학습 후 자동으로 `best.onnx`를 추출한다.

```bash
# 학습 (약 15~30분 / RTX 5070 Ti)
python3 models/train_yolo_lane.py --epochs 100 --batch 16

# 학습 중단 후 재개
python3 models/train_yolo_lane.py --resume

# ONNX 변환만 (이미 best.pt 있을 때)
python3 models/train_yolo_lane.py --export-only
```

**주요 하이퍼파라미터** (차선 인식 최적화):

| 파라미터 | 값 | 이유 |
|---------|-----|------|
| `iou` | 0.45 | 얇은 차선 마킹에 낮은 IoU 임계값 |
| `flipud` | 0.0 | 상하 반전 없음 (도로는 비대칭) |
| `fliplr` | 0.3 | 좌우 반전 30% (양방향 커버) |
| `copy_paste` | 0.1 | 세그멘테이션 전용 증강 |

---

### 6-3. 추론 노드 통합 검증

**파일**: `models/inference_node/lane_detect.py`

학습된 `best.onnx`를 로드하여 `/camera/image_raw`를 실시간 추론한다.

```bash
# 터미널 1 (Isaac Sim)
ENABLE_CAMERA=1 bash isaac_sim/launch_sim.sh

# 터미널 2 (추론 노드)
source /home/linux/gyusama-project/isaac_sim/ros2_terminal.sh
python3 models/inference_node/lane_detect.py

# 터미널 3 (시각화)
ros2 run rqt_image_view rqt_image_view    # /lane/debug_image
ros2 topic hz /lane/debug_image            # 처리율 확인
ros2 topic echo /cmd_vel                  # 제어 출력 확인
```

**검증 항목**:
- `[YOLO] 모델 로드:` 메시지 출력 (폴백 모드가 아닌지 확인)
- `/lane/debug_image`에서 차선 마스크(초록), 무게중심(노란선) 표시 확인
- 로봇이 `/cmd_vel`을 받아 차선 내에서 이동하는지 확인
- 차선 중앙 유지 및 조향 반응 확인

---

### 6-4. 학습 결과 분석

**파일**: `scripts/analyze_training.py`

YOLO 학습 결과(`results.csv`)를 파싱하여 손실 곡선과 정밀도/재현율 그래프를 저장한다.

```bash
python3 scripts/analyze_training.py
# 출력: models/runs/lane_seg_*/analysis/
#   loss_curves.png
#   precision_recall.png
#   summary.txt
```

**재학습 기준**:

| 지표 | 목표 | 미달 시 대응 |
|------|------|-------------|
| mAP50 (val) | ≥ 0.80 | 에폭 추가 (`--resume --epochs 150`) |
| mAP50-95 (val) | ≥ 0.55 | 더 많은 합성 데이터 생성 |
| Recall (val) | ≥ 0.80 | 더 다양한 각도·조명 샘플 추가 |

---

## 3. 완료 기준 체크리스트

- [x] `generate_synthetic_data.py` 실행 완료 — 1000장 이미지 생성 확인 (train 800 / val 200)
- [x] `verify_dataset.py` 통과 — train 800장 / val 200장, 레이블 파일 1:1 대응
- [x] `train_yolo_lane.py` 학습 완료 — `best.pt` + `best.onnx` 생성 (yolov8s-seg, 100 epoch)
- [x] `analyze_training.py` 실행 — 손실·정밀도 그래프 저장
- [ ] val mAP50 ≥ 0.80 달성 — 현재 best 0.716 (Precision 0.930 / Recall 0.689), 통합 테스트 후 필요 시 개선
- [ ] `lane_detect.py` ONNX 모드 실행 확인 (폴백 메시지 없음)
- [ ] `/lane/debug_image`에서 차선 마스크 시각화 정상 확인
- [ ] 로봇이 트랙 내에서 차선 추종 동작 확인 (`/cmd_vel` 변화 확인)
- [ ] 차선 중앙 유지 및 조향 반응 확인

---

## 4. 주의사항

### 터미널 격리 필수
Isaac Sim 터미널과 ROS2 터미널을 반드시 분리한다.
Isaac Sim 터미널에서 `source /opt/ros/jazzy/setup.bash`를 실행하면
`LD_LIBRARY_PATH` 오염으로 segfault가 발생한다.

### Python 환경 분리 필수
Isaac Sim(`isaac_env`)은 numpy==1.26을 요구한다.
YOLOv8 학습에 ultralytics를 설치하면 numpy 2.x로 업그레이드되어 Isaac Sim annotator가 깨진다.
반드시 별도 venv에서 학습을 실행한다.
```bash
# 학습용 venv (프로젝트에 이미 생성됨)
.venv_train/bin/python models/train_yolo_lane.py

# Isaac Sim 데이터 생성 (isaac_env 사용)
/home/linux/isaac_env/bin/python isaac_sim/generate_synthetic_data.py
```

### ONNX Runtime CUDA 가속
`lane_detect.py`는 `CUDAExecutionProvider`를 우선 사용한다.
`.venv_train`에 `onnxruntime-gpu`가 설치되어 있다.

### 합성 데이터 재생성 조건
모델 성능이 목표치 미달인 경우 데이터를 늘려 재생성한다.
환경변수로 제어: `N_TOTAL=2000 LANE_WHITE_THR=150 /home/linux/isaac_env/bin/python ...`

---

## 5. 6주차 이후 단계

6주차로 **W4~W6 Isaac Sim 디지털 트윈 단계가 완료**된다.

다음 단계:
- **W7~W9 AI 모델 학습 및 최적화**
  - 실제 트랙 이미지 추가 수집 (Sim-to-Real 갭 보상)
  - ONNX INT8 양자화 (RPi5 엣지 추론 가속)
  - 모델 성능 벤치마크 및 최종 배포
