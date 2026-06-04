# 규사마 프로젝트 최종 보고서

**자율주행 TurtleBot3 — Isaac Sim 기반 Sim-to-Real 파이프라인 구축**

> 작성일: 2026-05-27 (초안) | 시연 결과는 실물 주행 완료 후 추가 예정

---

## 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [개발 환경 및 시스템 구성](#2-개발-환경-및-시스템-구성)
3. [주차별 진행 요약](#3-주차별-진행-요약)
4. [핵심 기술 구현 상세](#4-핵심-기술-구현-상세)
5. [Sim-to-Real 갭 분석](#5-sim-to-real-갭-분석)
6. [트러블슈팅 사례](#6-트러블슈팅-사례)
7. [성능 측정 결과](#7-성능-측정-결과)
8. [시연 결과](#8-시연-결과) ← NPU 이후 작성
9. [개선 방향](#9-개선-방향)

---

## 1. 프로젝트 개요

### 1-1. 목표

Isaac Sim 디지털 트윈 환경에서 합성 데이터를 생성·학습하고,
실제 TurtleBot3 Burger에 배포하여 **차선 추종 + 장애물 회피 자율주행**을 구현한다.
DevOps 관점에서 PC → Docker Hub → RPi5 자동 배포 파이프라인까지 완성한다.

### 1-2. 프로젝트 일정

| 단계 | 주차 | 내용 |
|------|------|------|
| DevOps 기반 구축 | 1~3주 | Docker 멀티아키텍처 빌드, docker-compose 오케스트레이션 |
| Isaac Sim 디지털 트윈 | 4~6주 | TurtleBot3 USD 씬 구성, 가상 카메라 연동, 합성 데이터 생성 |
| AI 모델 학습 및 최적화 | 7~9주 | YOLOv8 재학습, APF 3노드 아키텍처 설계 |
| 실물 배포 | 10주 | RPi5 배포, 전체 노드 기동 및 장애물 감지 검증 |
| FPS 최적화 | 11~13주 | NCNN 전환, 320×320 재학습, Docker Hub 배포 파이프라인 |
| 최종 시연 | 14주 | 통합 주행 시연, 보고서 제출 |

### 1-3. 최종 시스템 구성

```
[RPi5 호스트]
  camera_node ─→ /camera/image_raw
        │
[Docker 컨테이너 — bbanggang/gyusama-rpi5:latest]
  inference-node  ─→ /lane/cmd_vel       (NCNN 320×320 차선 검출)
  obstacle-node   ─→ /obstacle/state     (LiDAR APF 척력 계산)
  behavior-node   ─→ /cmd_vel            (차선 인력 + 장애물 척력 합성)
  control-node    ─→ Dynamixel           (TurtleBot3 bringup)

[개발·배포 파이프라인]
  PC 코드 수정
    → docker buildx (ARM64 크로스빌드)
      → Docker Hub push (bbanggang/gyusama-rpi5:latest)
        → RPi5: docker compose pull && up
```

---

## 2. 개발 환경 및 시스템 구성

### 2-1. 개발 PC

| 항목 | 사양 |
|------|------|
| OS | Ubuntu 24.04.4 LTS |
| GPU | RTX 5070 Ti (Blackwell, sm_120) |
| CUDA | 13.x (Driver 580) |
| RAM | 62 GB |
| ROS2 | Jazzy |
| Isaac Sim | 5.1.0 (Python 3.11, `~/isaac_env`) |
| Docker | 29.3.1 + buildx + NVIDIA Container Toolkit |

### 2-2. 엣지 디바이스 (RPi5)

| 항목 | 사양 |
|------|------|
| 보드 | Raspberry Pi 5 |
| OS | Ubuntu 24.04 (ARM64) |
| 카메라 | IMX219 (libcamera RPi fork) |
| LiDAR | LDS-02 (9.4 Hz) |
| 모터 컨트롤러 | OpenCR (ROS2 펌웨어 V230127R1) |
| 모터 | Dynamixel XL430-W250 (1,000,000 baud) |
| Docker | 29.3.1 (호스트 설치) |

### 2-3. 노드 구성 및 토픽 흐름

```
IMX219(호스트)   ─→  /camera/image_raw
LDS-02(호스트)   ─→  /scan

/camera/image_raw ─→  inference-node  ─→  /lane/cmd_vel
                                           └── /lane/debug_image
/scan             ─→  obstacle-node   ─→  /obstacle/state

/lane/cmd_vel  ─┐
/obstacle/state─┤ behavior-node  ─→  /cmd_vel
                │
/cmd_vel        ─→  control-node (turtlebot3_node)
                      ─→  OpenCR → Dynamixel ID=1,2
```

---

## 3. 주차별 진행 요약

### 3-1. Week 1~3 — DevOps 기반 구축

**Week 1: 개발 환경 최적화 및 Docker 기초**

- Ubuntu 24.04, RTX 5070 Ti, Docker, Isaac Sim 5.1 환경 점검 완료
- ROS2 Jazzy 호스트 설치 (`ROS_DOMAIN_ID=1`)
- PC용 `Dockerfile.pc` 작성 (`nvidia/cuda:12.6.0-base-ubuntu24.04` 베이스)
- `entrypoint.sh`: 컨테이너 시작 시 ROS2 환경 자동 소싱

**Week 2: Multi-arch 빌드 파이프라인 구축**

- QEMU binfmt 등록 → ARM64 에뮬레이션 환경 구성
- `docker buildx create --name gyusama-builder` (docker-container 드라이버)
- `Dockerfile.rpi5` 작성: `ros:jazzy-ros-base` 베이스 (QEMU py3compile 버그 우회)
- x86_64 + ARM64 동시 빌드 검증 완료

**Week 3: Docker Hub 배포 및 컨테이너 오케스트레이션**

- `bbanggang/turtlebot3-ros2:v1.0` 멀티아키텍처 Hub push 완료
- `docker-compose.yml` 작성: `network_mode: host`, `ROS_DOMAIN_ID=1`, `LDS_MODEL=LDS-02`
- PC ↔ RPi5 ROS2 토픽 통신 확인 (FastDDS `ROS_DOMAIN_ID=1`)

---

### 3-2. Week 4~6 — Isaac Sim 디지털 트윈 구축

**Week 4: Isaac Sim 환경 구성 및 TurtleBot3 제어 검증**

- OmniGraph 내장 `ros2-turtlebot.usd` 확보 → `/scan`, `/odom`, `/cmd_vel` 자동 활성화
- `launch_sim.sh`: Python 버전 충돌·Intel GPU 간섭·Kit 캐시 손상 문제를 환경변수로 해결
- RTX 5070 Ti cold-start segfault 문제: `omni.graph.image.core` 제외로 해결
- IsaacLab으로 휠 조인트 ID·전진·후진·LiDAR 유효 포인트 검증 완료
- `run_track_sim.py`: AutoRace 루프 트랙 씬 구성

**Week 5: IMX219 가상 카메라 연동**

- Replicator annotator 방식으로 `omni.graph.image.core` 없이 이미지 획득
- 카메라 프림을 `base_footprint` 자식으로 등록 → 로봇 이동 자동 추종
- rclpy 직접 발행: `/camera/image_raw` (10 Hz), `/camera/camera_info`
- IMX219 정확 파라미터 적용: fl=3.04mm, HA=3.68mm, H-FOV=62.2°
- 합성 데이터 생성 스크립트 카메라 FOV 동기화

**Week 6: 합성 데이터 생성·YOLOv8 학습·추론 파이프라인 완성**

- `generate_synthetic_data.py`: 1000장 자동 생성 (train 800 / val 200)
- `train_yolo_lane.py`: YOLOv8n-detect 100 epoch, best mAP50=0.763
- `lane_detect.py`: 존 기반 중복 제거 + 비례 제어 (`angular = -Kp × offset`)
- 조명 과노출 문제 수정: DomeLight 350→120, RectLight 28000→3000

---

### 3-3. Week 7~9 — AI 모델 최적화 및 APF 아키텍처

**Week 7: 합성 데이터 재생성 및 INT8 양자화**

- 씬 조건 동기화 후 재학습: mAP50=0.763
- RPi5 FP32 추론: 186.4ms / **5.4 Hz** (목표 미달)
- 정적 INT8 양자화: 67.4ms / **14.8 Hz** (속도 통과)
  - 실제 검출: mAP50=0.000 → 캘리브레이션 데이터 분포 불일치로 실패

**Week 8: FP32 ONNX 최종 배포 모델 결정**

- Dynamic INT8: mAP50=0.432 (-38.7%) → 미채택
- **최종 배포 모델: FP32 ONNX** (정확도 손실 없음, 속도 문제는 별도 과제)
- RPi5 end-to-end 파이프라인 완성
- OpenCR 펌웨어 ROS2 교체, TwistStamped 환경변수 분기 처리

**Week 9: APF 3노드 아키텍처 구현**

- 이산 상태머신(LANE/WARN/AVOID/STOP) → APF 연속 합성 제어로 전환
- `obstacle_node.py`: LiDAR 스캔 ±90° → 선형 램프 척력 합산
- `behavior_manager.py`: `angular = clip(lane_w·lane_angular + K_REP·fy, ±0.5)`
- Latch 메커니즘: 장애물 방향 고정, FLIP_FRAMES=4 히스테리시스
- Isaac Sim 슬라롬 통합 검증 완료

---

### 3-4. Week 10 — RPi5 실물 배포

- 전체 4노드 기동 확인 (inference / obstacle / behavior / control)
- LiDAR `/scan` 9.4 Hz 수신 확인
- 장애물 전방 배치 시 stop_dist(0.22m) 기준 정지 동작 확인 ✅
- APF 파라미터 YAML 외부화 (`config/apf_params.yaml`) — 재빌드 없이 튜닝 가능
- 실물 버그 4건 발견·수정 (LiDAR 드라이버, ONNX 경로, `infer_sz`, `LDS_MODEL`)
- 차선 추종 동시 주행: FPS 2~3 → NPU 장착 후 재시도

---

### 3-5. Week 11~13 — FPS 최적화 및 Docker Hub 파이프라인

**Docker Hub 배포 파이프라인 전환 (Week 11)**

- `Dockerfile.rpi5`: `COPY models/inference_node/ config/` → 코드 이미지 내장
- `docker-compose.yml`: volumes 제거, `bbanggang/gyusama-rpi5:latest`
- `.dockerignore` 생성, `build_hub.sh` 스크립트 작성
- RPi5 native 빌드 → apt/pip 레이어 CACHED, COPY 레이어만 재빌드

**NCNN 추론 엔진 전환 (Week 11~12)**

- YOLOv8 → `best_ncnn_model/` 변환 (ultralytics CLI)
- `lane_detect_ncnn.py` 작성:
  - `_find_ncnn()`: `lane_det*/weights/best_ncnn_model` 경로 mtime 자동 선택
  - `YOLO_SZ` 환경변수로 입력 해상도 동적 제어
  - `[PERF]` 로그: 100프레임마다 FPS·추론시간·콜백시간 출력
- 640×640 NCNN 단독: 1.7~1.9 FPS (개선 미미) → 320×320 재학습 병행 결정

**320×320 모델 재학습 (Week 12~13)**

- NVIDIA driver 이슈: 595(크래시) → 580으로 재설치 후 Isaac Sim 정상 동작
- NVIDIA 595 완전 제거 (DKMS 재활성화 방지)
- 합성 데이터 씬 불일치 3건 수정:
  - 렌더러: RTX → Storm(pxr)
  - RectLight Y 위치: 3.13 → 0
  - 장애물 색상: WHITE → RED
- 1000장 재생성 (train 801 / val 201), mAP50 95.9% 달성
- Docker buildx CDI 오류 해결 (`features.cdi: false`)
- ARM64 이미지 빌드·Hub push → RPi5 pull 완료

---

## 4. 핵심 기술 구현 상세

### 4-1. Isaac Sim 합성 데이터 생성

**핵심 설계: 실제 주행 씬과의 도메인 일치**

`generate_synthetic_data.py`는 `run_track_sim.py`와 동일한 씬 조건을 유지해야
학습 데이터와 추론 환경 간 도메인 갭이 최소화된다.

| 항목 | 설정값 | 이유 |
|------|--------|------|
| 렌더러 | Storm(pxr), `omni.hydra.rtx` 제외 | run_track_sim.py 기본값과 동일 |
| DomeLight | 50~200 (랜덤) | 조명 과노출 방지 |
| RectLight | 1000~5000 (랜덤) | 씬 밝기 다양화 |
| RectLight Y | 0 (트랙 중심 상방) | 씬과 동일한 조명 위치 |
| 장애물 색상 | RED (1.0, 0.0, 0.0) | 흰색 장애물이 차선과 혼동됨 |
| 레이블 방식 | 흰색 픽셀 임계값(>150) 좌/우 분할 | 빠른 자동 레이블링 |

**최종 데이터셋 통계 (1000장, 320×320 재학습용)**

| 항목 | 값 |
|------|----|
| 총 이미지 | 1000장 (train 801 / val 201) |
| bbox 2개 이미지 (양쪽 차선) | 87.7% |
| bbox 1개 이미지 (곡선 등) | 12.3% |
| 좌/우 bbox 비율 | 47.9% / 52.1% (균형 양호) |

### 4-2. YOLOv8n 차선 검출 모델

**학습 변천**

| 단계 | 모델 | 해상도 | mAP@50 | 비고 |
|------|------|--------|--------|------|
| W6 초기 | YOLOv8n-seg | 640×640 | 0.716 | 세그멘테이션 방식 |
| W7 재학습 | YOLOv8n-detect | 640×640 | 0.763 | detect 전환, 조명 수정 |
| W13 최종 | YOLOv8n-detect | **320×320** | **0.959** | 합성 데이터 씬 불일치 3건 수정 |

**추론 노드 (`lane_detect_ncnn.py`) 핵심 로직**

```python
# 존 기반 중복 제거 — 좌(x < W/2) / 우(x ≥ W/2) 구역별 최고 confidence 1개 선택
zone_best: dict[int, dict] = {}
for box in results:
    zone = 0 if box["cx"] < img_w / 2 else 1
    if zone not in zone_best or box["conf"] > zone_best[zone]["conf"]:
        zone_best[zone] = box

# 차선 중심 오프셋 — 좌/우 bbox 중심 x 평균 기준
left_cx  = sorted_boxes[0]["cx"]
right_cx = sorted_boxes[-1]["cx"]
mid_cx   = (left_cx + right_cx) / 2.0
offset   = (mid_cx - img_w / 2.0) / (img_w / 2.0)  # [-1, 1]
angular  = -KP * offset                               # 비례 제어
```

### 4-3. APF 3노드 장애물 회피 아키텍처

**설계 원칙**: 이산 상태 전환 없이 연속적으로 차선 인력 + 장애물 척력을 합성

```
차선 인력 (inference-node)      장애물 척력 (obstacle-node)
  /lane/cmd_vel (angular_z)    /obstacle/state (fx:fy:min_front)
              ↓                          ↓
              └──────── behavior-node ───┘
                    APF 합성:
                    factor  = clip((min_front - STOP) / (SLOW - STOP), 0, 1)
                    lane_w  = LANE_MIN + (1 - LANE_MIN) × factor
                    angular = clip(lane_w × lane_angular + K_REP × fy, ±MAX_ANG)
                    linear  = lane_linear × factor
                              ↓
                          /cmd_vel
```

**파라미터** (`config/apf_params.yaml`)

| 파라미터 | 시뮬 값 | 실물 튜닝 필요 |
|---------|--------|--------------|
| `influence` | 0.65m | ★ |
| `latch_dist` | 0.55m | ★ |
| `k_rep` | 0.8 | ★ |
| `slow_dist` | 0.60m | ★ |
| `stop_dist` | 0.22m | — |
| `max_angular` | 0.5 rad/s | — |

### 4-4. Docker Hub 배포 파이프라인

```
코드 수정
   ↓
docker buildx build \
  --platform linux/arm64 \
  -t bbanggang/gyusama-rpi5:latest --push \
  -f docker/Dockerfile.rpi5 .
   ↓
ssh rapi5@192.168.0.155 \
  "cd ~/gyusama-project/docker && docker compose pull && docker compose up -d"
```

**Dockerfile.rpi5 주요 레이어**

```dockerfile
FROM ros:jazzy-ros-base                          # 멀티아키텍처 공식 이미지
RUN apt-get install ros-jazzy-turtlebot3-msgs \  # ← CACHED (변경 없으면)
    ros-jazzy-ld08-driver ... (생략)
RUN pip3 install onnxruntime ncnn "numpy<2"      # ← CACHED
COPY models/inference_node/ /ros2_ws/models/    # ← 코드 변경 시 재빌드
COPY config/                /ros2_ws/config/
COPY docker/entrypoint.sh   /entrypoint.sh
```

---

## 5. Sim-to-Real 갭 분석

### 5-1. 하드웨어 성능 차이

| 항목 | Isaac Sim (PC) | RPi5 실물 | 원인 |
|------|---------------|-----------|------|
| 추론 FPS (640×640 FP32) | ~30 FPS | ~2 FPS | RTX 5070 Ti vs ARM Cortex-A76 |
| 추론 FPS (320×320 NCNN) | — | **13~15 FPS** ✅ | NCNN ARM 최적화 |
| LiDAR 주파수 | 10 Hz | 9.4 Hz | ≈동일 |
| 모터 지연 | 없음 | ~50~80 ms | 실물 제어 루프 |

### 5-2. 소프트웨어/설정 차이

| 항목 | Isaac Sim | RPi5 실물 | 해결 방법 |
|------|-----------|-----------|-----------|
| `/cmd_vel` 타입 | `Twist` | `TwistStamped` | `CMD_VEL_STAMPED=1` 환경변수 분기 |
| LiDAR 0° 방향 | 로봇 후방 | 로봇 전방 | `LIDAR_OFFSET_DEG=180` (시뮬) |
| 카메라 드라이버 | Replicator annotator | libcamera (RPi fork) | 별도 호스트 노드 |
| FastDDS 탐색 | 로컬 루프백 | AP Isolation → 멀티캐스트 차단 | 유니캐스트 피어 설정 XML |
| OpenCR 펌웨어 | 가정 (ROS2) | ROS1 설치됨 → 교체 필요 | V230127R1 플래시 |

### 5-3. 도메인 갭 — 합성 데이터 vs 실물 이미지

| 항목 | 합성 데이터 | 실물 환경 | 완화 방법 |
|------|-----------|---------|---------|
| 조명 조건 | 균일 천장 조명 | 자연광·형광등 혼합 | DomeLight/RectLight 범위 랜덤화 |
| 바닥 텍스처 | 단색 매끄러운 표면 | 종이/PVC 트랙, 테이프 이음새 | 재질 다양화 필요 (개선 과제) |
| 차선 색상 | 순백 (RGB 255) | 약간 노란 테이프 | 임계값 튜닝 (LANE_WHITE_THR=150) |
| 카메라 왜곡 | 없음 (핀홀) | 경미한 배럴 왜곡 | 캘리브레이션 미적용 (개선 과제) |

---

## 6. 트러블슈팅 사례

### T-01. SSH 연결 불가 — AP Isolation

**증상**: PC에서 `ssh rapi5@192.168.0.155` → `No route to host`

**원인**: PC에 유선(enp4s0, metric=100)과 WiFi(wlx..., metric=600) 인터페이스가 공존.
`192.168.0.0/24` 트래픽이 metric 낮은 유선으로 라우팅되나 RPi5는 WiFi 전용 연결.
또한 공유기 AP Isolation으로 WiFi 기기 간 직접 통신 차단.

**해결**:
```bash
sudo ip route add 192.168.0.155 dev wlx705dccf3d9d3
```

---

### T-02. omni.graph.image.core segfault — RTX 5070 Ti (Blackwell)

**증상**: Isaac Sim cold-start 시 `omni.graph.image.core` 확장에서 segfault

**원인**: RTX 5070 Ti (Blackwell, sm_120) + Isaac Sim 5.1 조합에서
`isaacsim.exp.full.kit` 로드 시 해당 확장이 메모리 재할당 버그 발생

**해결**: `SimulationApp` 초기화 전 제외 선언
```python
sys.argv += ["--/app/extensions/excluded/0=omni.graph.image.core"]
```
카메라 데이터는 Replicator annotator 방식으로 대체 (OmniGraph 불필요)

---

### T-03. 카메라가 로봇을 따라오지 않음

**증상**: 로봇 이동 후 카메라 world 좌표 변화 없음

**원인**: Isaac Sim 물리 엔진은 articulation root(`turtlebot3_burger`)의 USD Xform을
갱신하지 않고 rigid body(`base_footprint`)만 매 스텝 갱신한다.
카메라를 articulation root 자식으로 붙이면 USD 계층 상속이 동작하지 않음.

**해결**:
```python
# 수정 전 (고정됨)
cam_path = _ROBOT_PATH + "/turtlebot3_burger/imx219_camera"
# 수정 후 (추종됨)
cam_path = _ROBOT_PATH + "/base_footprint/imx219_camera"
```

---

### T-04. LiDAR `/scan` 데이터 없음 — LDS_MODEL 불일치

**증상**: RPi5 control-node 기동 후 `/scan` 토픽 미발행

**원인**: `docker-compose.yml`에 `LDS_MODEL=ld08`로 설정.
`turtlebot3_bringup`의 `robot.launch.py`는 `LDS-02` 문자열 기대.
`ld08 ≠ LDS-02` → launch 파일 KeyError로 LiDAR 드라이버 미시작.

**해결**: `LDS_MODEL=LDS-02`로 수정 + `ros-jazzy-ld08-driver` Dockerfile 추가

---

### T-05. rsync 로 ONNX 모델 파일 미전송

**증상**: inference-node 기동 시 `FileNotFoundError: best.onnx`

**원인**: `deploy_rpi5.sh`에 `--exclude='runs/'` 옵션이 있었으나
`models/runs/` 경로를 앵커링하지 않아 최상위 `runs/` 뿐 아니라
모든 하위 경로의 `runs/` 디렉토리를 제외함.

**해결**: `--exclude='/runs/'` (앵커링)

---

### T-06. INT8 양자화 후 검출 완전 실패 (mAP50=0.000)

**증상**: 정적 INT8 양자화 모델이 RPi5에서 단 하나의 bbox도 검출하지 못함

**원인**: 캘리브레이션 데이터(`data/synthetic/images/val`)가 실제 추론 분포와 달라
활성화 범위가 잘못 계산됨. 특히 YOLOv8의 마지막 Detection Head의 logit 범위가
합성 이미지 전용 캘리브레이션으로는 포착되지 않았을 것으로 추정.

**해결**: INT8 포기 → FP32 ONNX 최종 배포, 속도 문제는 NCNN 전환 + 320×320 재학습으로 해결

---

### T-07. `/cmd_vel` 타입 불일치 — Twist vs TwistStamped

**증상**: RPi5에서 `/cmd_vel` 수신 후 모터 무반응

**원인**: ROS2 Jazzy의 `turtlebot3_node` 2.3.6이 `geometry_msgs/TwistStamped`를 요구.
Isaac Sim 연동용 `lane_detect.py`는 `Twist`로 발행 중.

**해결**: 환경변수로 분기
```python
_USE_STAMPED = os.environ.get('CMD_VEL_STAMPED', '0') == '1'
# docker-compose.yml: CMD_VEL_STAMPED=1 (RPi5)
# docker-compose.sim.yml: 미설정 (Isaac Sim, Twist 유지)
```

---

### T-08. OpenCR 펌웨어 버전 불일치 (ROS1 → ROS2)

**증상**: `turtlebot3_node` 기동 후 Dynamixel 연결 실패 + stack smashing

**원인**: OpenCR에 ROS1 펌웨어 탑재. ROS2 `turtlebot3_node`와 패킷 포맷 불일치로
크래시 반복 → Dynamixel 토크 자동 비활성화 (보호 모드 진입).

**해결**:
```bash
# RPi5 aarch64에서 32비트 ARM 바이너리 실행 지원
sudo dpkg --add-architecture armhf
sudo apt install libc6:armhf libstdc++6:armhf
# ROS2 펌웨어 플래시 (V230127R1)
./update.sh /dev/ttyACM0 burger.opencr
# 토크 재활성화
ros2 service call /motor_power std_srvs/srv/SetBool '{data: true}'
```

---

### T-09. FastDDS 크로스 머신 탐색 실패 — AP Isolation

**증상**: RPi5 컨테이너에서 발행한 토픽이 PC에서 보이지 않음

**원인**: FastDDS 기본 멀티캐스트 탐색이 AP Isolation으로 차단됨

**해결**: 유니캐스트 피어 XML로 RPi5 IP + 포트 직접 지정
```xml
<initialPeersList>
  <locator><udpv4><address>192.168.0.155</address><port>7650</port></udpv4></locator>
  <!-- 7660, 7662, 7664 추가 -->
</initialPeersList>
```

---

### T-10. NVIDIA 드라이버 595 → Isaac Sim 크래시

**증상**: Driver 595 업그레이드 후 Isaac Sim 실행 시 RTX API 오류로 크래시

**원인**: Driver 595가 RTX API를 변경해 Isaac Sim 5.1과 충돌.
또한 CDI(Container Device Interface) 스펙 파일 없는 환경에서
`docker buildx create` 시 "failed to discover GPU vendor from CDI" 오류 발생.

**해결**:
- NVIDIA 드라이버 580으로 재설치 (Isaac Sim 요구사항: ≥535.129.03, 580 정상 동작)
- 595 패키지 완전 제거: `sudo apt purge nvidia-firmware-595-595.71.05`
  (DKMS가 커널 업데이트 시 595 모듈 재빌드하는 것을 방지)
- Docker daemon CDI 비활성화:
  `/etc/docker/daemon.json` → `"features": {"cdi": false}`

---

### T-11. 합성 데이터 씬 불일치 — 렌더러·조명·장애물 색상

**증상**: 320×320 재학습용 합성 데이터 생성 씬이 실제 주행 씬과 상이

**원인**: `generate_synthetic_data.py`가 3가지 항목에서 `run_track_sim.py`와 다른 값 사용:
1. 렌더러: RTX(`omni.hydra.rtx`) vs 실제 주행은 Storm(pxr)
2. RectLight Y 위치: Y=3.13(트랙 중앙) vs Y=0(트랙 상방)
3. 장애물 색상: WHITE → 카메라가 차선으로 오인

**해결**:
```python
sys.argv += ["--/app/extensions/excluded/2=omni.hydra.rtx",
             "--/renderer/active=pxr"]
UsdGeom.XformCommonAPI(rect).SetTranslate(Gf.Vec3d(0, 0, 8))  # Y=0
RED = (1.00, 0.00, 0.00)
```

---

## 7. 성능 측정 결과

### 7-1. 추론 FPS 변천

| 단계 | 방법 | FPS | 추론 시간 | 결과 |
|------|------|-----|-----------|------|
| W7 초기 | FP32 ONNX 640×640 (RPi5 CPU) | 5.4 | 186 ms | 목표 미달 |
| W7 INT8 | 정적 INT8 양자화 640×640 | 14.8 | 67 ms | 검출 실패 |
| W8 배포 | FP32 ONNX 640×640 (최종) | ~2.8 | ~294 ms | 실물 배포 |
| W11 NCNN | NCNN 640×640 | 1.7~1.9 | 317~437 ms | 개선 미미 |
| **W13 최종** | **NCNN 320×320** | **13~15** | **27~33 ms** | **✅ 목표 달성** |

### 7-2. 현재 RPi5 노드별 토픽 주파수

| 노드 | 토픽 | 주파수 |
|------|------|--------|
| camera_node (호스트) | `/camera/image_raw` | ~15 FPS |
| inference-node | `/lane/cmd_vel` | 13~15 FPS |
| obstacle-node | `/obstacle/state` | ~9.4 Hz (LiDAR 동기) |
| control-node | `/scan` | 9.4 Hz |

### 7-3. 모델 크기 비교

| 모델 | 파일 크기 | 해상도 | mAP@50 |
|------|---------|--------|--------|
| best.pt (640 FP32) | ~22 MB | 640×640 | 0.763 |
| best.onnx (640 FP32) | ~11.7 MB | 640×640 | 0.763 |
| best_int8.onnx (640) | 3.2 MB | 640×640 | 0.000 (실패) |
| **best.pt (320 FP32)** | **6 MB** | **320×320** | **0.959** |
| **best.onnx (320 FP32)** | **12 MB** | **320×320** | **0.959** |
| best_ncnn_model (320) | bin 12MB + param 17KB | 320×320 | ≈0.959 |

---

## 8. 시연 결과

> **⏸ 작성 예정 — NPU 장착 후 실물 통합 주행 시연 완료 시 추가**

### 8-1. 측정 지표 (목표)

| 지표 | 목표 | 측정값 |
|------|------|--------|
| 차선 추종 완주율 | 90% 이상 | — |
| 장애물 회피 성공률 | 80% 이상 | — |
| 추론 FPS | 10 FPS 이상 | **13~15 FPS ✅** |
| LiDAR Hz | 9 Hz 이상 | **9.4 Hz ✅** |
| 전체 완주 소요 시간 | — | — |

### 8-2. 시연 영상

> 추가 예정

---

## 9. 개선 방향

### 9-1. 단기 (NPU 장착 후)

| 항목 | 방법 | 예상 효과 |
|------|------|-----------|
| NPU 추론 전환 | Hailo-8L HAT + HEF 변환 | ~30 FPS (20× 향상) |
| 실물 APF 파라미터 튜닝 | `config/apf_params.yaml` 현장 조정 | 회피 정확도 향상 |
| 차선 추종 KP 튜닝 | 실물 트랙 곡률 기반 | 오버슈팅 감소 |

### 9-2. 중기

| 항목 | 방법 | 목적 |
|------|------|------|
| 실물 이미지 파인튜닝 | 트랙 촬영 이미지 + 합성 데이터 혼합 | Sim-to-Real 갭 감소 |
| 카메라 캘리브레이션 | `camera_calibration` 패키지 | 왜곡 보정 → 검출 정확도 향상 |
| ROS2 QoS 튜닝 | 센서 토픽 Best-Effort 전환 | 네트워크 지연 감소 |

### 9-3. 장기

| 항목 | 방법 | 목적 |
|------|------|------|
| APF 자동 파라미터 최적화 | 베이지안 최적화 또는 RL | 수동 튜닝 제거 |
| 멀티 카메라 / Depth 카메라 | RealSense D435 + 깊이 기반 회피 | 장애물 3D 위치 정확도 |
| CI/CD 파이프라인 | GitHub Actions + Docker Hub | 코드 변경 → 자동 빌드·배포 |

---

## 부록 A. 주요 파일 목록

| 파일 | 설명 |
|------|------|
| `docker/Dockerfile.rpi5` | RPi5용 ARM64 이미지 정의 |
| `docker/docker-compose.yml` | 4노드 오케스트레이션 |
| `docker/entrypoint.sh` | ROS2 환경 자동 소싱 |
| `isaac_sim/launch_sim.sh` | Isaac Sim 실행 환경 변수 설정 |
| `isaac_sim/run_track_sim.py` | Isaac Sim 주행 씬 + 카메라 발행 |
| `isaac_sim/generate_synthetic_data.py` | 합성 데이터 1000장 자동 생성 |
| `models/training/train_yolo_lane.py` | YOLOv8n 학습 스크립트 |
| `models/inference_node/lane_detect_ncnn.py` | NCNN 차선 추론 + ROS2 노드 |
| `models/inference_node/obstacle_node.py` | LiDAR APF 척력 계산 노드 |
| `models/inference_node/behavior_manager.py` | APF 합성 제어 노드 |
| `config/apf_params.yaml` | APF 튜닝 파라미터 외부화 |
| `scripts/monitor_nodes.py` | 노드 FPS·상태 실시간 모니터 |
| `scripts/build_hub.sh` | PC buildx → Hub push 스크립트 |
| `scripts/start_nodes.sh` | RPi5 노드 일괄 시작 스크립트 |

---

## 부록 B. 환경변수 설정 목록

| 환경변수 | 설정값 | 적용 노드 | 설명 |
|---------|--------|----------|------|
| `ROS_DOMAIN_ID` | 1 | 전체 | PC ↔ RPi5 통신 도메인 |
| `TURTLEBOT3_MODEL` | burger | control-node | TurtleBot3 모델 지정 |
| `LDS_MODEL` | LDS-02 | control-node | LiDAR 드라이버 모델 |
| `CMD_VEL_STAMPED` | 1 | inference-node | TwistStamped 사용 (RPi5) |
| `CAMERA_FLIP` | 1 | inference-node | 카메라 상하 반전 |
| `YOLO_SZ` | 320 | inference-node | NCNN 입력 해상도 |
| `AUTOSTART` | 0/1 | inference-node | 시작 시 자동 주행 여부 |
| `LIDAR_OFFSET_DEG` | 0 (실물) / 180 (시뮬) | obstacle-node | LiDAR 0° 방향 보정 |
| `FASTDDS_BUILTIN_TRANSPORTS` | UDPv4 | 전체 | FastDDS 전송 방식 |
