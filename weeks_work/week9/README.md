# 9주차 진행 보고 — APF 기반 장애물 회피 + 노드 분리

> **목표**: 단일 노드에 혼합된 차선 추종 + 장애물 회피 로직을 분리하고,
> APF(Artificial Potential Field) 기반 연속 합성 제어로 부드러운 회피를 구현한다.

---

## 1. 배경 — 8주차에서 이어짐

| 항목 | 상태 |
|------|------|
| 전체 파이프라인 | ✅ 동작 확인 (IMX219 → YOLO → /cmd_vel → OpenCR → Dynamixel) |
| 추론 속도 | 🔶 ~4.5 FPS (ncnn fp16) — 차선 추종 품질 미흡 |
| 장애물 회피 | 🔶 lane_detect.py 내부 상태머신으로 구현, BT 미적용 |
| 차선 추종 품질 | ⏸ FPS 개선 후 별도 진행 예정 |

기존 `lane_detect.py`는 LiDAR 상태(clear/warn/avoid/stop)를 판단해 차선 조향과 블렌딩하는 방식이었으나, 이산적 상태 전환으로 인한 진동 및 차선 이탈 문제가 있었다. 이를 APF 연속 합성 방식으로 전환했다.

---

## 2. 최종 아키텍처 — APF 연속 합성 제어

```
/camera/image_raw ─► inference-node ─► /lane/cmd_vel   (차선 인력)
/scan             ─► obstacle-node  ─► /obstacle/state (장애물 척력)
                                    ↓
                            behavior-manager (APF 합성)
                              angular = clip(lane_w·lane_angular + K_REP·fy, ±MAX_ANGULAR)
                              linear  = lane_linear × speed_factor(min_front)
                                    ↓
                               /cmd_vel ─► turtlebot3_node
```

**핵심 설계 원칙**:
- 차선 인력과 장애물 척력을 연속적으로 합성 → 회피 후 자동 차선 복귀
- 이산 상태 없음 → 전환 순간 충격(jerk) 없음
- Latch 메커니즘으로 방향 진동 방지

---

## 3. 구현 내용

### 3-1. obstacle_node.py — APF 척력 계산

**파일**: `models/inference_node/obstacle_node.py`

```
구독: /scan
발행: /obstacle/state (std_msgs/String)
  - "clear"              : 영향권 내 장애물 없음
  - "fx:fy:min_front"    : 합성 척력 벡터 + 전방 최소거리
```

LiDAR 스캔 ±90° 범위에서 영향권(0.65m) 내 포인트의 선형 램프 척력을 합산한다.

| 파라미터 | 값 | 설명 |
|---------|---|------|
| `INFLUENCE` | 0.65m | 척력 영향권 반경 |
| `FRONT_DEG` | 22° | 전방 최소거리 측정 범위 |
| `SCAN_HALF` | 90° | 척력 합성 스캔 반경 |
| `MIN_OBS_POINTS` | 5 | clear 판정 최소 포인트 수 |
| `LATCH_DIST` | 0.55m | 회피 방향 고정 거리 |
| `RELEASE_FRAMES` | 2 | latch 해제 디바운스 프레임 수 |
| `FLIP_FRAMES` | 4 | 방향 전환 히스테리시스 프레임 수 |

**부호 규약**: `+deg = 물리적 로봇 좌측`, `fy > 0 = 좌회전`, `fy < 0 = 우회전`

**Latch 메커니즘**: `fy` 부호로 장애물 방위를 판정해 회피 방향을 고정. 정면 대칭 장애물은 좌우 개방도(`open_dir`)로 폴백.

**Isaac Sim 설정**: LiDAR 0°가 로봇 후방을 향함 → `LIDAR_OFFSET_DEG=180` 필요

### 3-2. lane_detect.py — 장애물 로직 제거

`/scan` 구독과 LiDAR 상태머신 제거. 순수 차선 추종 명령만 `/lane/cmd_vel`로 발행.  
속도 1.5× 상향: `MAX_SPEED=0.21`, `MIN_SPEED=0.12`

### 3-3. behavior_manager.py — APF 합성 제어

**파일**: `models/inference_node/behavior_manager.py`

```
구독: /lane/cmd_vel, /obstacle/state
발행: /cmd_vel
```

| 파라미터 | 값 | 설명 |
|---------|---|------|
| `MAX_ANGULAR` | 0.5 rad/s | 최대 각속도 제한 (차선 이탈 방지) |
| `K_REP` | 0.8 | 척력 게인 |
| `LANE_MIN` | 0.35 | 장애물 근접 시 차선 인력 최소 가중치 |
| `SLOW_DIST` | 0.60m | 감속 시작 거리 |
| `STOP_DIST` | 0.22m | 정지 거리 |

**제어 법칙**:
```
factor  = clip((min_front - STOP_DIST) / (SLOW_DIST - STOP_DIST), 0, 1)
lane_w  = LANE_MIN + (1 - LANE_MIN) × factor   # 근접 시 차선 인력 감쇠
angular = clip(lane_w × lane_angular + K_REP × fy, ±MAX_ANGULAR)
linear  = lane_linear × factor
```

### 3-4. docker-compose 업데이트

**`docker-compose.yml`** — 실물(RPi5) 3노드 구조:
```yaml
services:
  inference-node:   # lane_detect.py    → /lane/cmd_vel
  obstacle-node:    # obstacle_node.py  → /obstacle/state  (LIDAR_OFFSET_DEG=0)
  behavior-node:    # behavior_manager  → /cmd_vel
  control-node:     # turtlebot3_bringup
```

**`docker-compose.sim.yml`** — Isaac Sim 시뮬레이션:
```yaml
services:
  inference-node:   # AUTOSTART=1
  obstacle-node:    # LIDAR_OFFSET_DEG=180 (Isaac Sim LiDAR 후방 오프셋)
  behavior-node:    # CMD_VEL_STAMPED=0 (Twist 사용)
```

---

## 4. 트러블슈팅 이력

| 문제 | 원인 | 해결 |
|------|------|------|
| LiDAR 항상 clear | 장애물 높이(OBH=0.09m) < LiDAR 마운트 | OBH=0.25m로 상향 |
| 흰색 장애물 차선 오인식 | 카메라가 흰 큐브를 차선으로 추종 | 장애물 색상 RED로 변경 |
| LIDAR_OFFSET_DEG 혼선 | Isaac Sim LiDAR 0°=후방 | `LIDAR_OFFSET_DEG=180` 환경변수 |
| 좌우 회피 방향 반전 | sin 부호 착오 | `fy += -mag·sin(rad)` (음부호 확인) |
| 진동(oscillation) | 이산 상태 전환 + 대칭 탈출력 | APF 연속 합성 + Latch 메커니즘 |
| 슬라롬 두 번째 장애물 방향 오류 | latch 이월 | FLIP_FRAMES=4 히스테리시스 |
| 차선 이탈 | MAX_ANGULAR 과대 + 차선 인력 과소 | MAX_ANGULAR=0.5, LANE_MIN=0.35 |

---

## 5. 완료 기준 체크리스트

- [x] obstacle_node.py 구현 — `/scan` → `/obstacle/state` 발행 확인
- [x] lane_detect.py 에서 장애물 로직 분리 — `/lane/cmd_vel` 발행 확인
- [x] behavior_manager.py 구현 — APF 합성 제어로 `/cmd_vel` 발행 확인
- [x] docker-compose.yml 3노드 구조로 업데이트
- [x] Isaac Sim에서 통합 동작 확인 — 슬라롬 장애물 배치 시 회피 후 차선 복귀 ✅
- [ ] RPi5 실물 테스트 — 슬라롬 구간 장애물 회피 동작 확인

---

## 6. 잔여 과제 (추후)

| 과제 | 조건 |
|------|------|
| RPi5 실물 슬라롬 테스트 | 10주차 진행 |
| 차선 추종 품질 개선 | FPS 개선(320×320 재학습 또는 Hailo-8L) 이후 진행 |
| Sim-to-Real 갭 분석 | 실물 테스트 이후 진행 |
| APF 파라미터 실물 재튜닝 | 실물 트랙 특성에 맞게 INFLUENCE, K_REP 등 조정 필요 |
