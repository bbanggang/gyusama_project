# 9주차 진행 보고 — 장애물 회피 Behavior Tree 설계 및 노드 분리

> **목표**: 단일 노드에 혼합된 차선 추종 + 장애물 회피 로직을 분리하고,
> Behavior Tree 기반으로 우선순위 제어를 구현한다.

---

## 1. 배경 — 8주차에서 이어짐

| 항목 | 상태 |
|------|------|
| 전체 파이프라인 | ✅ 동작 확인 (IMX219 → YOLO → /cmd_vel → OpenCR → Dynamixel) |
| 추론 속도 | 🔶 ~4.5 FPS (ncnn fp16) — 차선 추종 품질 미흡 |
| 장애물 회피 | 🔶 lane_detect.py 내부 상태머신으로 구현, BT 미적용 |
| 차선 추종 품질 | ⏸ FPS 개선 후 별도 진행 예정 |

현재 `lane_detect.py`는 LiDAR 상태(clear/warn/avoid/stop)를 판단해 차선 조향과 블렌딩하는 방식으로, 차선 추종이 지배적으로 동작하며 장애물 회피가 제대로 우선처리되지 않는다.

---

## 2. 목표 아키텍처

원래 블록 다이어그램 기준 구조:

```
/camera/image_raw ─► inference-node ─► /lane/cmd_vel
/scan             ─► obstacle-node  ─► /obstacle/state
                                    ↓
                            behavior-manager
                                    ↓
                               /cmd_vel ─► turtlebot3_node
```

**behavior-manager**가 `/obstacle/state`를 보고 우선순위를 결정한다.
- `clear` → `/lane/cmd_vel` 그대로 `/cmd_vel`로 전달 (차선 추종)
- `warn/avoid/stop` → 장애물 회피 명령 생성 후 `/cmd_vel`로 전달

---

## 3. 구현 내용

### 9-1. obstacle_node 분리

`lane_detect.py`에서 LiDAR 처리 로직을 별도 노드로 분리한다.

**파일**: `models/inference_node/obstacle_node.py`

```
구독: /scan
발행: /obstacle/state (std_msgs/String: "clear" / "warn:left" / "avoid:right" / "stop")
```

LiDAR 파라미터는 기존 lane_detect.py와 동일하게 유지:

| 파라미터 | 값 |
|---------|---|
| `OBS_WARN`  | 0.85m |
| `OBS_AVOID` | 0.65m |
| `OBS_STOP`  | 0.30m |
| `FRONT_DEG` | 22° |
| `SIDE_DEG`  | 70° |

### 9-2. lane_detect.py 에서 장애물 로직 제거

`lane_detect.py`(및 `lane_detect_ncnn.py`)에서 `/scan` 구독과 장애물 상태머신을 제거하고, 순수 차선 추종 명령만 `/lane/cmd_vel`로 발행하도록 수정한다.

### 9-3. behavior_manager 노드 구현

**파일**: `models/inference_node/behavior_manager.py`

```
구독: /lane/cmd_vel, /obstacle/state
발행: /cmd_vel (TwistStamped)
```

Behavior Tree 우선순위:

```
Root (Selector)
├── ObstacleAvoid  ← /obstacle/state가 warn/avoid/stop이면 실행
└── LaneFollow     ← /obstacle/state가 clear이면 /lane/cmd_vel 그대로 전달
```

### 9-4. docker-compose.yml 업데이트

세 노드를 별도 서비스로 관리:

```yaml
services:
  inference-node:   # lane_detect_ncnn.py → /lane/cmd_vel 발행
  obstacle-node:    # obstacle_node.py    → /obstacle/state 발행
  behavior-node:    # behavior_manager.py → /cmd_vel 발행
  control-node:     # turtlebot3_bringup
```

---

## 4. 완료 기준 체크리스트

- [x] obstacle_node.py 구현 — `/scan` → `/obstacle/state` 발행 확인
- [x] lane_detect.py 에서 장애물 로직 분리 — `/lane/cmd_vel` 발행 확인
- [x] behavior_manager.py 구현 — 우선순위에 따라 `/cmd_vel` 전환 확인
- [x] docker-compose.yml 3노드 구조로 업데이트
- [ ] Isaac Sim에서 통합 동작 확인 — 장애물 배치 시 회피 후 차선 복귀
- [ ] RPi5 실물 테스트 — 슬라롬 구간 장애물 회피 동작 확인

---

## 5. 잔여 과제 (추후)

| 과제 | 조건 |
|------|------|
| 차선 추종 품질 개선 | FPS 개선(320×320 재학습 또는 Hailo-8L) 이후 진행 |
| Sim-to-Real 갭 분석 | 차선 추종 안정화 이후 진행 |
| 실물 트랙 커브 구간 테스트 | FPS 개선 이후 진행 |
