# 10주차 진행 보고 — RPi5 실물 배포 및 노드 기동 검증

> **목표**: Isaac Sim에서 검증된 APF 장애물 회피 시스템을 RPi5 실물에 배포하고,
> 카메라·LiDAR·모터 전체 노드 기동 및 장애물 감지 동작을 확인한다.

---

## 1. 배경 — 9주차에서 이어짐

| 항목 | 상태 |
|------|------|
| APF 3노드 아키텍처 | ✅ Isaac Sim 슬라롬 검증 완료 |
| RPi5 실물 배포 | ✅ 이번 주 완료 |
| 차선 추종 + 장애물 회피 동시 주행 | ⏸ FPS 부족 → NPU 장착 후 진행 |
| 실물 APF 파라미터 튜닝 | ⏸ 동시 주행 가능 시점에 진행 |

Isaac Sim과 실물 환경의 주요 차이:
- LiDAR 해상도·노이즈 특성
- 카메라 노출·색상 응답
- 바닥 반사·조명 조건
- 로봇 모터 지연(실물 약 50~80ms)

---

## 2. 구현 내용

### 10-1. APF 파라미터 YAML 외부화

**파일**: `config/apf_params.yaml`

재빌드 없이 파라미터를 조정할 수 있도록 모든 튜닝 값을 YAML로 분리했다.
`obstacle_node.py`·`behavior_manager.py`가 시작 시 이 파일을 로드한다.
환경변수(`LIDAR_OFFSET_DEG` 등)는 YAML보다 우선 적용된다.

```yaml
obstacle_node:
  influence: 0.65      # ★ 실물 재튜닝 필요
  latch_dist: 0.55     # ★ 실물 재튜닝 필요
behavior_manager:
  k_rep: 0.8           # ★ 실물 재튜닝 필요
  max_angular: 0.5
  slow_dist: 0.60      # ★ 실물 재튜닝 필요
  stop_dist: 0.22
```

### 10-2. 노드 헬스 모니터링

**파일**: `scripts/monitor_nodes.py`

```bash
python3 scripts/monitor_nodes.py
# [14:32:01]  camera=15.2fps  scan=12.0fps  lane=14.8fps  obs=12.1fps  cmd=14.8fps
#             obs_state: fx=-0.12 fy=+0.34 d=0.52m [AVOID]
```

### 10-3. RPi5 배포 자동화

**파일**: `scripts/deploy_rpi5.sh`

```bash
./scripts/deploy_rpi5.sh            # 코드 동기화만
./scripts/deploy_rpi5.sh --restart  # 동기화 + 컨테이너 재시작
./scripts/deploy_rpi5.sh --build    # 동기화 + 이미지 재빌드 + 재시작
```

### 10-4. 실물 배포 중 발견·수정한 버그

| 버그 | 원인 | 수정 |
|------|------|------|
| LiDAR `/scan` 데이터 없음 | `LDS_MODEL=ld08` → launch 파일이 `LDS-02` 기대 | `LDS-02`로 수정 |
| LiDAR 드라이버 미설치 | `ros-jazzy-ld08-driver` Dockerfile 누락 | Dockerfile에 추가 |
| ONNX 모델 미전송 | rsync `--exclude='runs/'`가 `models/runs/`까지 차단 | `--exclude='/runs/'`로 앵커링 |
| 추론 크기 불일치 | `YOLO_SZ=640` 하드코딩, 실제 모델은 320×320 | 모델에서 동적 로드 (`infer_sz`) |

### 10-5. 실물 노드 기동 확인

| 노드 | 상태 | 비고 |
|------|------|------|
| `camera_node` (호스트) | ✅ 640×480 스트리밍 | IMX219, libcamera RPi fork |
| `inference-node` | ✅ 640×640 모델 로드 | 2~3 FPS (NPU 후 개선) |
| `obstacle-node` | ✅ LiDAR 구독 중 | |
| `behavior-node` | ✅ APF 준비 완료 | |
| `control-node` | ✅ Dynamixel + LiDAR | LDS-02 9.4 Hz |

---

## 3. 실물 테스트 결과

### 장애물 감지 → 정지 동작
- LiDAR `/scan` 9.4 Hz 정상 수신
- `obstacle_node` → `behavior_manager` 연동 확인
- 장애물 전방 배치 시 `stop_dist(0.22m)` 기준 정지 동작 확인 ✅

### 차선 추종 + 장애물 회피 동시 주행
- 추론 FPS 2~3 (목표 10+) → 동시 주행 테스트 보류
- 원인: FP32 640×640 모델을 RPi5 CPU에서 추론 (300~430 ms/frame)
- 해결 계획: NPU 장착 후 재시도

---

## 4. 완료 기준 체크리스트

- [x] `config/apf_params.yaml` — 파라미터 YAML 외부화
- [x] `obstacle_node.py` — YAML 로드 적용 (환경변수 우선)
- [x] `behavior_manager.py` — YAML 로드 적용
- [x] `scripts/monitor_nodes.py` — 토픽 FPS·장애물 상태 모니터
- [x] `scripts/deploy_rpi5.sh` — RPi5 배포 자동화
- [x] RPi5 실물 배포 및 전체 노드 기동 확인
- [x] LiDAR `/scan` 데이터 수신 확인 (9.4 Hz)
- [x] 장애물 감지 → 정지 동작 확인
- [⏸] 슬라롬 구간 차선 추종 + 장애물 회피 동시 테스트 → NPU 후 진행
- [⏸] 실물 APF 파라미터 튜닝 (`config/apf_params.yaml` 업데이트) → NPU 후 진행

---

## 5. 트러블슈팅 가이드

| 증상 | 확인 사항 | 조치 |
|------|-----------|------|
| SSH 연결 안 됨 (No route to host) | PC 유선 ↔ RPi5 WiFi AP Isolation | `sudo ip route add 192.168.0.155 dev wlx705dccf3d9d3` |
| `/scan` 데이터 없음 | `LDS_MODEL` 환경변수 확인 | `docker-compose.yml`에서 `LDS-02`로 설정 |
| 추론 오류 (input dimension mismatch) | `YOLO_SZ` vs 실제 모델 크기 | `infer_sz` 동적 로드로 해결 (이미 수정됨) |
| `obs_state` 토픽 없음 | obstacle_node 미실행 | `docker compose logs obstacle-node` |
| camera=DEAD | libcamera 호스트 실행 필요 | 호스트에서 `ros2 run camera_ros camera_node` |

---

## 6. 다음 주차 과제

| 과제 | 주차 |
|------|------|
| NPU 장착 및 FPS 개선 | 11주차 |
| INT8/NCNN 전환으로 중간 FPS 확보 | 11주차 |
| Sim-to-Real 갭 분석 문서화 | 11주차 |
| 차선 추종 + 장애물 회피 동시 주행 테스트 | NPU 후 |
| 실물 APF 파라미터 튜닝 | NPU 후 |
| 최종 시연 준비 | 12~13주차 |
