# 10주차 진행 보고 — RPi5 실물 배포 및 APF 파라미터 튜닝

> **목표**: Isaac Sim에서 검증된 APF 장애물 회피 시스템을 RPi5 실물에 배포하고,
> 실물 트랙 환경에 맞게 파라미터를 튜닝하여 슬라롬 구간 자율주행을 완성한다.

---

## 1. 배경 — 9주차에서 이어짐

| 항목 | 상태 |
|------|------|
| APF 3노드 아키텍처 | ✅ Isaac Sim 슬라롬 검증 완료 |
| RPi5 실물 테스트 | ⏳ 이번 주 진행 |
| APF 파라미터 | 🔶 Isaac Sim 기준, 실물 재튜닝 필요 |
| 파라미터 관리 | 🔶 하드코딩 → YAML 외부화 진행 |

Isaac Sim과 실물 환경의 차이:
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
  ...
behavior_manager:
  k_rep: 0.8           # ★ 실물 재튜닝 필요
  max_angular: 0.5
  slow_dist: 0.60      # ★ 실물 재튜닝 필요
  stop_dist: 0.22
  ...
```

실물 튜닝 권장 순서:
1. `stop_dist` — 정지 거리 확인 (로봇이 실제로 멈추는 위치)
2. `slow_dist` / `influence` — 감속 시작 거리 확인
3. `k_rep` — 회피 각속도 세기 조정
4. `max_angular` — 차선 이탈 여부 보고 제한

### 10-2. 노드 헬스 모니터링

**파일**: `scripts/monitor_nodes.py`

실물 테스트 시 각 노드의 토픽 FPS·장애물 상태·최종 cmd_vel을 2초 간격으로 출력한다.

```bash
# 실행 (RPi5에서)
python3 scripts/monitor_nodes.py

# 출력 예시
[14:32:01]  camera=15.2fps  scan=12.0fps  lane=14.8fps  obs=12.1fps  cmd=14.8fps
            obs_state: fx=-0.12 fy=+0.34 d=0.52m [AVOID]
            cmd_vel:   linear=0.168 angular=+0.324
```

| 색상 | 의미 |
|------|------|
| 초록 | FPS ≥ 10 (정상) |
| 노랑 | FPS 5~10 (주의) |
| 빨강 | FPS < 5 또는 DEAD (5초 이상 미수신) |

### 10-3. RPi5 배포 자동화

**파일**: `scripts/deploy_rpi5.sh`

```bash
# 코드 동기화만
./scripts/deploy_rpi5.sh

# 동기화 + 컨테이너 재시작
./scripts/deploy_rpi5.sh --restart

# 동기화 + 이미지 재빌드 + 재시작
./scripts/deploy_rpi5.sh --build

# 접속 정보 변경
RPI5_HOST=192.168.0.XXX ./scripts/deploy_rpi5.sh --restart
```

rsync 제외 목록: `.git/`, `runs/`, `data/`, `.pt` 파일  
(INT8/FP32 `.onnx`는 포함)

---

## 3. 실물 테스트 절차

### 3-1. 배포

```bash
# PC에서
./scripts/deploy_rpi5.sh --restart
```

### 3-2. 노드 확인

```bash
# RPi5에서
docker compose logs -f inference-node
docker compose logs -f obstacle-node
docker compose logs -f behavior-node

# 또는 모니터 스크립트
python3 scripts/monitor_nodes.py
```

### 3-3. 슬라롬 테스트 절차

1. 트랙 위 슬라롬 구간에 장애물(원뿔) 2~3개 배치
2. 로봇 출발 → `s` 키 입력 (AUTOSTART=0인 경우)
3. monitor_nodes.py 출력에서 `obs_state`, `cmd_vel` 확인
4. 결과에 따라 `config/apf_params.yaml` 조정 후 재시작

### 3-4. 실물 vs 시뮬 파라미터 비교

| 파라미터 | Isaac Sim | RPi5 (초기값) | 비고 |
|---------|-----------|--------------|------|
| `influence` | 0.65 | 0.65 | 실물 LiDAR 노이즈 따라 조정 |
| `latch_dist` | 0.55 | 0.55 | 실물 제동거리 따라 조정 |
| `k_rep` | 0.8 | 0.8 | 회피력 약하면 ↑, 이탈하면 ↓ |
| `max_angular` | 0.5 | 0.5 | 차선 이탈 시 ↓ |
| `slow_dist` | 0.60 | 0.60 | 실물 감속 응답 따라 조정 |
| `stop_dist` | 0.22 | 0.22 | 실물 제동거리 확인 후 조정 |

---

## 4. 완료 기준 체크리스트

- [x] `config/apf_params.yaml` — 파라미터 YAML 외부화
- [x] `obstacle_node.py` — YAML 로드 적용 (환경변수 우선)
- [x] `behavior_manager.py` — YAML 로드 적용
- [x] `scripts/monitor_nodes.py` — 토픽 FPS·장애물 상태 모니터
- [x] `scripts/deploy_rpi5.sh` — RPi5 배포 자동화
- [ ] RPi5 실물 배포 및 노드 기동 확인
- [ ] 슬라롬 구간 장애물 회피 실물 테스트
- [ ] 실물 APF 파라미터 튜닝 완료 (`config/apf_params.yaml` 업데이트)

---

## 5. 트러블슈팅 가이드

| 증상 | 확인 사항 | 조치 |
|------|-----------|------|
| obs=DEAD | obstacle_node 미실행 / /scan 미수신 | `docker compose logs obstacle-node` |
| obs_state=clear (장애물 앞에서도) | INFLUENCE 너무 작음 | `influence` 값 ↑ |
| 장애물 너무 일찍 반응 | INFLUENCE 너무 큼 | `influence` 값 ↓ |
| 회피 후 차선 이탈 | k_rep 또는 max_angular 너무 큼 | `k_rep` ↓ 또는 `max_angular` ↓ |
| 회피 방향 진동 | latch 미작동 (latch_dist 너무 작음) | `latch_dist` ↑ |
| 부딪힘 (회피 불충분) | k_rep 너무 작음 | `k_rep` ↑ |
| 정지 후 미출발 | stop_dist 너무 큼 | `stop_dist` ↓ |
| camera=DEAD | libcamera 호스트 실행 필요 | 호스트에서 `ros2_camera_node` 직접 실행 |

---

## 6. 잔여 과제

| 과제 | 주차 |
|------|------|
| RPi5 실물 슬라롬 완성 | 10주차 (이번 주) |
| Sim-to-Real 갭 분석 및 문서화 | 11주차 |
| 차선 추종 품질 개선 (FPS 향상) | 11~12주차 |
| 최종 시연 준비 | 13~14주차 |
