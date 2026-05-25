# 12주차 진행 보고 — 최종 시연 준비 및 배포 파이프라인 완성

> **목표**: 전체 시스템을 통합하여 최종 시연을 준비하고,
> Docker Hub 기반 배포 파이프라인으로 전환하여 운용 편의성을 높인다.

---

## 1. 배경 — 11주차에서 이어짐

| 항목 | 상태 |
|------|------|
| NPU 장착 및 FPS 개선 | 11주차 완료 예정 |
| 차선 추종 + 장애물 회피 동시 주행 | 11주차 완료 예정 |
| APF 파라미터 튜닝 | 11주차 완료 예정 |
| Docker Hub 배포 전환 | ⏸ 이번 주 진행 |
| 최종 시연 준비 | ⏸ 이번 주 진행 |

---

## 2. 구현 계획

### 12-1. Docker Hub 배포 파이프라인 전환 (Direction A)

현재 volume mount 방식 → 코드를 이미지에 내장하는 방식으로 전환.

**Dockerfile.rpi5 수정**
```dockerfile
# 코드 이미지에 내장
COPY models/inference_node/ /ros2_ws/models/inference_node/
COPY config/ /ros2_ws/config/
```

**docker-compose.yml 수정**
```yaml
# volumes 제거
# image: bbanggang/gyusama-rpi5:latest 로 교체
```

**빌드 및 push**
```bash
docker buildx build \
  --platform linux/arm64 \
  -t bbanggang/gyusama-rpi5:latest \
  --push \
  -f docker/Dockerfile.rpi5 .
```

**RPi5 배포**
```bash
# RPi5에서
docker compose pull && docker compose up -d
```

### 12-2. 최종 시연 시나리오

1. RPi5 전원 ON → Docker 컨테이너 자동 시작 (`restart: unless-stopped`)
2. 호스트에서 카메라 노드 실행
3. `docker attach inference-node` → `s` 키로 주행 시작
4. 직선 구간 차선 추종 → 슬라롬 구간 장애물 회피 → 복귀

### 12-3. 성능 측정 지표

| 지표 | 목표 | 측정 방법 |
|------|------|-----------|
| 차선 추종 완주율 | 90% 이상 | 5회 시도 중 성공 횟수 |
| 장애물 회피 성공률 | 80% 이상 | 장애물 5개 중 회피 성공 수 |
| 추론 FPS | 10 FPS 이상 | `monitor_nodes.py` |
| LiDAR Hz | 9 Hz 이상 | `ros2 topic hz /scan` |

### 12-4. 최종 보고서 작성

- 주차별 진행 내용 요약
- Sim-to-Real 성능 비교 분석
- 트러블슈팅 사례 정리
- 개선 방향 제안

---

## 3. 완료 기준 체크리스트

- [ ] Docker Hub 이미지 push (`bbanggang/gyusama-rpi5:latest`)
- [ ] RPi5 `docker compose pull && up` 으로 단독 실행 확인
- [ ] 차선 추종 트랙 완주율 90% 이상
- [ ] 장애물 배치 시 회피 성공률 80% 이상
- [ ] 전체 시스템 시연 영상 촬영
- [ ] 최종 보고서 제출

---

## 4. 최종 시스템 구성

```
[RPi5 호스트]
  camera_node (/camera/image_raw)
        │
[Docker 컨테이너]
  inference-node ─→ /lane/cmd_vel
  obstacle-node  ─→ /obstacle/state
  behavior-node  ─→ /cmd_vel (APF 통합)
  control-node   ─→ Dynamixel (TurtleBot3)
```

```
[개발·배포 파이프라인]
  PC 코드 수정
    → docker buildx (ARM64 크로스빌드)
      → Docker Hub push
        → RPi5 pull & up
```
