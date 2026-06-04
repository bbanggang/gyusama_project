# gyusama-project 진행 정리 3

---

## 카테고리 A. lane_follow를 위한 차선 추론 성능 개선 — FPS ~2 → 13~15

기존 **FP32 ONNX (RPi5)** 기준 **1~2 FPS** 정도의 추론 속도로 인해
카메라 콜백 주기보다 추론이 느려 차선 추종 명령의 실시간 반응이 불가

→ 정적 INT8 / 동적 INT8 모두 실패하여 NPU를 사용하여 실시간 추종을 진행하기로 결정
→ 금일 NPU 도착 전까지 추론 성능 향상을 위해 다음과 같은 방법 수행

### A-1. NCNN 추론 엔진 전환

- Tencent NCNN: ARM NEON SIMD에 고도 최적화된 추론 엔진
- 가중치 양자화 없이도 ONNX Runtime 대비 ARM에서 2~3배 빠름

> **ARM NEON SIMD 작동 원리 (한 줄 요약)**
> NEON은 ARM CPU의 SIMD 명령어 셋으로, 한 번의 명령으로 128비트 레지스터에 담긴
> 여러 데이터(FP32 4개 또는 INT8 16개)를 **동시에 연산**하는 방식이며,
> NCNN은 conv·matmul·pool 등 핵심 커널을 NEON intrinsics로 직접 작성해
> 컴파일러 자동 벡터화보다 캐시 친화적·분기 최소화된 코드를 사용한다.

따라서 ARM(RPi5)에서 비효율적인 ONNX Runtime을 ARM 친화적 엔진으로 교체

**결과**

| 모델 | 해상도 | 엔진 | FPS | 추론 시간 |
| --- | --- | --- | --- | --- |
| YOLOv8s | 640×640 | ONNX (구) | ~2 | 294 ms |
| YOLOv8s | 640×640 | **NCNN** | 1.7~1.9 | 317~437 ms |

→ **FPS 개선 미미**

> **왜 NCNN 전환만으로는 효과가 작았는가**
> 640×640 YOLOv8s 1회 추론은 약 28 GFLOPs를 요구하는데,
> RPi5(Cortex-A76 4코어)의 이론 처리량은 약 8 GFLOPs 수준이다.
> NEON SIMD 가속은 **같은 연산을 더 빠른 명령으로 수행할 뿐 절대 FLOPs는 그대로**이므로,
> 입력 해상도가 일정하다면 2~3배의 엔진 가속만으로는 1 FPS 대역을 벗어나기 어렵다.
> → 연산량 자체를 줄이는 입력 해상도 축소가 병행되어야 함을 확인.

### A-2. 320×320 모델 재학습

입력 해상도를 640 → 320 축소하여 CNN 연산량이 이론적으로 약 4배 감소할 것으로 예상

→ 다만 해상도가 줄어드는 만큼 정확도 손실 우려
→ 또한 프로젝트 진행하면서 기존의 학습한 world_scene이 수정되어 수정된 map으로 재학습

목표 : NCNN(엔진 최적화) × 입력 축소(연산량 최소화)를 결합해 10+ FPS 달성

**결과** (`models/runs/lane_det/results.csv`)

| 항목 | 값 |
| --- | --- |
| 모델 | YOLOv8n, imgsz=320, epochs=100 |
| 환경 | RTX 5070 Ti |
| best epoch | **91** |
| **mAP@50** | **0.95931 (95.9%)** |
| mAP@50-95 | 0.78731 |
| 가중치 크기 | best.pt 6.0MB / best.onnx 12MB / **ncnn bin 12MB + param 17KB** |

> **NCNN 모델 파일 구조 — 왜 `param`과 `bin`이 분리되는가**
>
> | 파일 | 크기 | 형식 | 역할 |
> |------|------|------|------|
> | `model.ncnn.param` | 17 KB | **텍스트** | 네트워크 그래프 구조 정의 — 레이어 타입(Convolution, Concat, …), 입출력 텐서 이름, 커널 크기·stride 등 하이퍼파라미터 |
> | `model.ncnn.bin` | 12 MB | **바이너리** | 학습된 weights/biases 수치만 저장 |
> | `metadata.yaml` | 396 B | YAML | 클래스 이름·입력 크기 등 YOLO 후처리용 메타데이터 |
> | `model_ncnn.py` | 783 B | Python | NCNN Python 바인딩 호출 래퍼 |
>
> **분리 이유**
> - **param 은 구조만 표현** → 같은 가중치를 약간 다른 구조로 재사용하거나(예: 입력 크기 변경), 모델 구조만 검토할 때 가볍게 로드 가능
> - **param 이 텍스트** → grep/diff로 변환 결과 검증·디버깅 용이 (ONNX의 어떤 op가 NCNN의 어떤 레이어로 매핑됐는지 추적 가능)
> - **bin 은 수치만** → 압축·전송·메모리 mmap 최적화에 유리

### A-3. 최종 성능

| 구성 | FPS | 추론 시간 | 콜백 전체 |
| --- | --- | --- | --- |
| 640 ONNX (구) | ~2 | 294 ms | — |
| 640 NCNN | 1.7~1.9 | 317~437 ms | 539~591 ms |
| **320 NCNN (현)** | **13~15** | **27~33 ms** | **66~77 ms** |
| 향상 비율 | **≈ 8×** | **≈ 13×** | **≈ 8×** |

→ NPU 장착 이전에 **실시간 주행 가능 수준** 확보했지만
30FPS까지 추론 속도를 올리기 위해 NPU 장착도 진행할 예정

---

## 카테고리 B. 장애물 회피 + 차선 추종 통합 — Behavior Tree 시뮬 검증

### B-0. 배경 — 왜 통합이 필요한가

기존 구조는 `lane_detect.py` **단일 노드 내부**에서 LiDAR 상태(clear/warn/avoid/stop)를
직접 판단하여 차선 조향과 블렌딩하는 방식이었다.

문제점
- 차선 추종이 항상 지배적으로 동작 → 장애물 회피가 늦거나 무시됨
- 한 노드에 인지·판단·제어가 섞여 디버깅·튜닝 곤란
- 차선 인식이 실패하면 장애물 회피까지 같이 멈춤 (단일 실패점)

목표
- **인지(차선/장애물) → 판단(우선순위/블렌딩) → 제어** 3단계를 ROS 노드 단위로 분리
- 시뮬레이터(Isaac Sim)에서 통합 동작을 먼저 검증 후 실물 이식

### B-1. APF(Artificial Potential Field) 기반 Behavior Tree 아키텍처

전통적인 py_trees 식 BT(Sequence/Selector 노드 트리)가 아닌,
**APF 인력·척력 합성 기반 행동 트리**로 설계했다.

- **인력 (attractive)** : 차선 중심 — 항상 살아있어 회피 후 자동 복귀
- **척력 (repulsive)** : 장애물 방향 반대 — 거리 가까울수록 강함
- 둘을 가중합 → BT의 "우선순위 전환"을 **연속적인 가중치 변화**로 표현

```
                       /camera/image_raw
                              │
                              ▼
                  ┌────────────────────────┐
                  │ inference-node         │  ① 인지 - 차선
                  │ lane_detect_ncnn.py    │  →  /lane/cmd_vel
                  └────────────────────────┘
                              │
        /scan                 │
          │                   │
          ▼                   ▼
┌────────────────────┐   ┌─────────────────────────┐
│ obstacle-node      │   │ behavior-node           │   ③ 판단·합성
│ obstacle_node.py   │──▶│ behavior_manager.py     │
│ (척력 계산)         │   │ (APF 가중합)             │
└────────────────────┘   └─────────────────────────┘
   /obstacle/state                │
   (clear/warn/                   ▼
   avoid/stop + fy)         /cmd_vel  ─▶  ④ 제어 (control-node)
```

### B-2. 노드별 역할

| 노드 | 입력 | 출력 | 핵심 로직 |
|------|------|------|----------|
| `lane_detect_ncnn` | `/camera/image_raw` | `/lane/cmd_vel` | YOLO 추론 → 좌/우 차선 bbox → 중심 오프셋 → P 제어 |
| `obstacle_node` | `/scan` | `/obstacle/state` | ±90° 범위 척력 합성, 영향권 0.65m, ±22° 전방 최소거리 계산 |
| `behavior_manager` | `/lane/cmd_vel` + `/obstacle/state` | `/cmd_vel` | APF 가중합 + 감속 |

### B-3. 장애물 상태 머신 (`/obstacle/state`)

`obstacle_node.py`가 `/scan` 한 프레임마다 다음 4개 상태 중 하나를 발행

| 상태 | 조건 | 후속 행동 |
|------|------|----------|
| **clear** | 영향권(0.65m) 내 포인트 < 5개 | 차선 추종 100% 가중 |
| **warn**  | 영향권 진입, 전방최단 > slow_dist(0.60m) | 척력 발생, 감속 시작 |
| **avoid** | latch_dist(0.55m) 진입 → 회피 방향 latch | 척력 우세, 방향 고정 |
| **stop**  | 전방최단 < stop_dist(0.22m) | linear=0, 정지 |

**Hysteresis(이력) 적용** — 방위 노이즈로 조향이 흔들리지 않도록
- `release_frames=2` : latch 해제는 2프레임 연속 멀어진 뒤
- `flip_frames=4`   : 반대 방향 전환은 4프레임 연속 반대 신호 시
- `fy_eps=0.05`     : 미세한 척력은 무시 (직진 유지)

### B-4. APF 합성 공식 (`behavior_manager.py`)

```python
# 차선 가중치 — 장애물이 가까울수록 lane_w 감소 (최소 LANE_MIN=0.35)
lane_w  = LANE_MIN + (1.0 - LANE_MIN) * factor

# 각속도: 차선 인력 × 차선 가중 + 척력 게인 × y방향 척력
angular = clip(lane_w * lane_angular + K_REP * fy,  ±MAX_ANGULAR=0.5)

# 직진속도: 차선 명령 × 거리 기반 감속 계수
linear  = lane_linear * speed_factor(min_front)
```

**핵심 튜닝값** (`config/apf_params.yaml`)

| 파라미터 | 값 | 의미 |
|---------|----|----|
| `max_angular` | 0.5 rad/s | 차선 이탈 방지 상한 |
| `k_rep` | 0.8 | 척력 → 각속도 변환 게인 |
| `lane_min` | 0.35 | 회피 중에도 차선 인력 최소 유지 비중 |
| `slow_dist` | 0.60 m | 감속 시작 거리 |
| `stop_dist` | 0.22 m | 비상 정지 거리 |

### B-5. 시뮬레이터 검증 (`docker-compose.sim.yml`)

실물(RPi5)과 시뮬(Isaac Sim)을 같은 컨테이너 이미지로 실행하기 위해
별도 compose 파일을 분리

**Sim 전용 환경 조정**
- `LIDAR_OFFSET_DEG=180` — Isaac Sim LiDAR 0° 방향이 실물과 반대
- `CMD_VEL_STAMPED=0` — Isaac Sim은 `Twist` 사용 (RPi5는 `TwistStamped`)
- LiDAR scan range, /scan QoS를 시뮬에 맞춤

**검증 시나리오**
1. **직선 차선 + 정면 장애물** → clear → warn → avoid → stop 순차 발동 확인
2. **곡선 차선 + 측면 장애물** → 차선 인력 유지하며 척력으로 회피 방향 결정
3. **다중 장애물 클러스터** → latch 메커니즘으로 방향 진동 없이 한쪽 회피
4. **장애물 회피 후 복귀** — clear 복귀 즉시 lane_w → 1.0, 차선 중심 복귀 확인

**검증 결과**
- 4개 시나리오 모두 시뮬 내 정상 동작
- 차선 추종 → 회피 → 복귀 전이가 끊김 없이 연속 (BT discrete state 전환의 단점 해소)
- 단일 노드 구조 대비 obstacle만 재시작해도 lane 추종은 영향 없음 (독립 실패 도메인 확인)

### B-6. 잔여 과제 (실물 이식)

- 좁은 차선에서 차선 벽을 장애물로 오인 → `front_deg=22`로 좁혔으나 추가 필터링 필요
- 실물 LiDAR LDS-02 노이즈 패턴이 Isaac Sim과 달라 `min_obs_points` 재튜닝 필요
- behavior-node 우회 단독 테스트(DIRECT_CMD_VEL=1)로 차선 추종 안정화 후 통합 재개 예정
