# 8주차 진행 보고 — RPi5 실물 배포 및 Sim-to-Real 테스트

> **목표**: INT8 ONNX 모델과 `lane_detect.py`를 RPi5에 배포하고,
> 실제 트랙에서 차선 추종 동작을 검증한다.

---

## 1. 배경 — 7주차에서 이어짐

| 항목 | 결과 |
|------|------|
| 모델 | YOLOv8n-detect (lane_det-3) |
| PC CPU 추론 | ~25 ms |
| RPi5 FP32 추론 | 186.4 ms / 5.4 Hz (미달) |
| RPi5 INT8 추론 | **67.4 ms / 14.8 Hz** ✅ |
| INT8 모델 크기 | 3.2 MB (FP32 11.7 MB → 3.6x 압축) |

7주차에서 INT8 양자화로 RPi5 실시간 처리가 가능해졌다.
8주차는 이 모델을 실제 로봇에 올려 Sim-to-Real 갭을 확인하는 단계다.

---

## 2. 구현 내용

### 8-1. RPi5 배포 환경 구성

RPi5에 ROS2 Jazzy + onnxruntime + lane_detect.py 실행 환경을 구성한다.

```bash
# RPi5에서 실행
# 1) ROS2 Jazzy 설치 확인
ros2 --version

# 2) Python 패키지 설치 (venv 사용)
python3 -m venv ~/lane_env
~/lane_env/bin/pip install onnxruntime opencv-python-headless numpy

# 3) 프로젝트 파일 동기화 (PC → RPi5)
rsync -avz --exclude='.venv_train' --exclude='data/' --exclude='models/runs/*/weights/*.pt' \
    /home/linux/gyusama-project/ rapi5@192.168.0.153:~/gyusama-project/
```

### 8-2. INT8 모델 우선 사용 설정

`lane_detect.py`의 `_find_onnx()`가 `_int8.onnx`를 우선 선택하도록 수정한다.

**파일**: `models/inference_node/lane_detect.py`

```python
def _find_onnx(self) -> str | None:
    # _int8.onnx 우선, 없으면 best.onnx 선택
    candidates = sorted(
        glob.glob("models/runs/*/weights/best_int8.onnx") +
        glob.glob("models/runs/*/weights/best.onnx"),
        key=os.path.getmtime,
    )
    return candidates[-1] if candidates else None
```

### 8-3. ROS2 네트워크 구성 (PC ↔ RPi5)

두 기기가 동일한 `ROS_DOMAIN_ID`로 통신해야 한다.

```bash
# PC와 RPi5 모두 동일하게 설정
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**토픽 흐름**:
```
RPi5                           PC
/camera/image_raw  ──────────► lane_detect.py (선택적)
/scan              ──────────► lane_detect.py
                   ◄──────────  /cmd_vel
```

RPi5에서 직접 lane_detect.py를 실행하는 구조:
```
RPi5: IMX219 카메라 → /camera/image_raw → lane_detect.py → /cmd_vel → Dynamixel
```

### 8-4. RPi5에서 lane_detect.py 실행

```bash
# RPi5 터미널
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=1
cd ~/gyusama-project
~/lane_env/bin/python models/inference_node/lane_detect.py
```

**확인 사항**:
- `[YOLO] 모델 로드: ...best_int8.onnx` 메시지 출력 (INT8 모델 선택 확인)
- `/cmd_vel` 토픽 발행 확인: `ros2 topic echo /cmd_vel`
- `/lane/debug_image` 수신: PC에서 `ros2 run rqt_image_view rqt_image_view`

### 8-5. Sim-to-Real 갭 분석

Isaac Sim(가상)과 실제 트랙 간의 차이를 확인한다.

| 확인 항목 | 가상 환경 | 실제 환경 |
|---------|----------|---------|
| 조명 조건 | DomeLight 120, RectLight 3000 | 실내 형광등 |
| 차선 색상 | 흰색 (RGB 255) | 흰색 테이프/페인트 |
| 바닥 색상 | (0.02, 0.02, 0.02) 검정 | 실제 바닥 재질 |
| CONF_THRESH | 0.12 | 조정 필요할 수 있음 |
| WHITE_THRESH (폴백) | 180 | 조정 필요할 수 있음 |

**갭이 클 경우 대응**:
1. 실물 트랙 이미지 50~100장 수집
2. 기존 합성 데이터와 혼합하여 파인튜닝
3. `CONF_THRESH` 조정

---

## 3. 완료 기준 체크리스트

- [x] RPi5 배포 환경 구성 — Docker 기반 inference-node / control-node 구성 완료
- [-] INT8 모델 우선 선택 수정 — `_find_onnx()` 수정 완료, 단 INT8 실검출 실패(mAP50=0.000)로 FP32(`best.onnx`) 로 전환
- [-] RPi5에서 `lane_detect.py` 실행 — 실행 확인, 단 `best_int8.onnx` 아닌 FP32 모델 로드 중
- [x] `/cmd_vel` 토픽 발행 확인 — 모터 동작 확인
- [-] 실물 트랙에서 차선 추종 테스트 — 파이프라인 동작 확인, FPS ~2로 실질적 차선 추종 어려움
- [ ] 커브 구간 차선 추종 확인 — 실물 트랙 미테스트
- [ ] Sim-to-Real 갭 분석 — 미수행
- [ ] (선택) 실물 이미지 파인튜닝 — 미수행

---

## 4. 주의사항

### INT8 모델 정확도 저하 가능성

INT8 양자화는 속도를 높이는 대신 정확도가 소폭 낮아질 수 있다.
실물 테스트에서 인식률이 낮으면 `CONF_THRESH`를 0.10~0.08로 낮추거나
FP32 모델(`best.onnx`)로 롤백해 원인을 파악한다.

### RPi5에서 ONNX 경로 설정

RPi5의 프로젝트 경로가 PC와 다를 수 있다(`~/gyusama-project/`).
`lane_detect.py`의 `_find_onnx()`가 상대 경로로 동작하므로
반드시 프로젝트 루트에서 실행한다.

### ROS_DOMAIN_ID 일치 필수

PC와 RPi5의 `ROS_DOMAIN_ID`가 다르면 토픽이 서로 안 보인다.
양쪽 모두 `export ROS_DOMAIN_ID=1` 설정 후 실행한다.

---

## 5. 8주차 이후 단계 — W9~W10

- **W9**: 통합 제어 완성
  - LiDAR + 카메라 복합 제어 최적화 (실물 기반)
  - 슬라롬 장애물 회피 + 차선 추종 통합 데모
- **W10**: 최종 시연 준비
  - 전체 시스템 안정화
  - 발표 자료 및 시연 영상 준비
