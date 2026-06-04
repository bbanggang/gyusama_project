# 13주차 진행 보고 — Docker Hub 배포 파이프라인 전환 및 Sim-to-Real 분석

> **목표**: volume mount 방식에서 Docker Hub 이미지 배포 방식(Direction A)으로 전환하여
> RPi5에서 `pull` 만으로 실행 가능한 파이프라인을 완성하고,
> Sim-to-Real 갭을 분석·문서화한다.

---

## 1. 배경 — 12주차에서 이어짐

| 항목 | 상태 |
|------|------|
| NPU 장착 및 FPS 개선 | 11주차 완료 예정 |
| 차선 추종 + 장애물 회피 동시 주행 | 12주차 완료 예정 |
| APF 파라미터 실물 튜닝 | 12주차 완료 예정 |
| Docker Hub 배포 전환 | ⏸ 이번 주 진행 |
| Sim-to-Real 갭 분석 | ⏸ 이번 주 진행 |

현재 배포 방식: `rsync` 로 코드를 RPi5에 동기화 → 컨테이너 내에서 volume mount로 접근
목표 배포 방식: 코드를 이미지에 내장 → Docker Hub push → RPi5 `docker compose pull && up`

---

## 2. 구현 계획

### 13-1. Dockerfile.rpi5 수정 (코드 이미지 내장)

```dockerfile
# 기존: volume mount로 외부에서 주입
# 변경: 빌드 시 코드를 이미지에 포함

COPY models/inference_node/ /ros2_ws/models/inference_node/
COPY config/ /ros2_ws/config/
```

### 13-2. docker-compose.yml 수정

```yaml
services:
  inference-node:
    image: bbanggang/gyusama-rpi5:latest   # Docker Hub 이미지
    # volumes: 제거 (코드가 이미지에 내장)
    restart: unless-stopped
```

### 13-3. PC에서 ARM64 빌드 및 push

```bash
# PC에서 ARM64 크로스빌드 후 Docker Hub push
docker buildx build \
  --platform linux/arm64 \
  -t bbanggang/gyusama-rpi5:latest \
  --push \
  -f docker/Dockerfile.rpi5 .
```

### 13-4. RPi5에서 pull & 실행

```bash
# RPi5에서 (코드 동기화 불필요)
docker compose pull
docker compose up -d
```

### 13-5. Sim-to-Real 갭 분석 문서화

실물 주행 데이터를 기반으로 시뮬레이션과 비교 분석:

| 항목 | Isaac Sim | RPi5 실물 | 차이 원인 |
|------|-----------|-----------|-----------|
| 추론 FPS | ~30 FPS | 10+ FPS (NPU) | 하드웨어 차이 |
| LiDAR Hz | 10 Hz | 9.4 Hz | 거의 동일 |
| 차선 검출 신뢰도 | 0.7~0.9 | 측정값 기입 | 조명 차이 |
| APF `k_rep` | 0.8 | 재튜닝 값 기입 | 실물 제동거리 차이 |
| APF `influence` | 0.65 | 재튜닝 값 기입 | 실물 LiDAR 노이즈 |
| `stop_dist` | 0.22 m | 재튜닝 값 기입 | 실물 바닥 마찰 |

---

## 3. 배포 파이프라인 변경 전/후 비교

```
[기존 — volume mount 방식]
PC 코드 수정
  → rsync로 RPi5 동기화
    → docker compose restart

[변경 — Docker Hub 방식 (Direction A)]
PC 코드 수정
  → docker buildx (ARM64 크로스빌드)
    → Docker Hub push
      → RPi5: docker compose pull && up
```

**장점**: RPi5에서 코드 별도 동기화 불필요, 버전 태그로 롤백 가능
**단점**: 코드 변경 시마다 이미지 재빌드 필요 (빌드 시간 ~5분)

---

## 4. 완료 기준 체크리스트

- [ ] `Dockerfile.rpi5` — `COPY` 명령으로 코드 내장
- [ ] `docker-compose.yml` — volumes 제거, `bbanggang/gyusama-rpi5:latest` 적용
- [ ] `docker buildx build --platform linux/arm64 --push` 성공
- [ ] RPi5 `docker compose pull && up` 단독 실행 확인
- [ ] Sim-to-Real 갭 분석 문서 작성 완료

---

## 5. 트러블슈팅 가이드

| 증상 | 원인 | 조치 |
|------|------|------|
| buildx push 실패 | Docker Hub 미로그인 | `docker login` 후 재시도 |
| ARM64 빌드 지연 | QEMU 에뮬레이션 (정상) | 완료 대기 (~5분) |
| RPi5 pull 후 실행 오류 | 이미지 내 경로 불일치 | Dockerfile `COPY` 목적지 확인 |
| config 파일 미반영 | 이미지 재빌드 필요 | `--no-cache` 옵션 추가 후 재빌드 |
