# 14주차 진행 보고 — 최종 주행 시연 및 보고서 제출

> **목표**: 최종 시연 시나리오로 실물 트랙에서 차선 추종 + 장애물 회피 통합 주행을 완수하고,
> 프로젝트 보고서를 제출하여 14주 과정을 마무리한다.

---

## 1. 배경 — 13주차에서 이어짐

| 항목 | 상태 |
|------|------|
| Docker Hub 배포 파이프라인 전환 | 13주차 완료 예정 |
| Sim-to-Real 갭 분석 | 13주차 완료 예정 |
| 최종 주행 시연 | ⏸ 이번 주 진행 |
| 최종 보고서 제출 | ⏸ 이번 주 진행 |

---

## 2. 최종 시연 시나리오

### 절차

1. RPi5 전원 ON → Docker 컨테이너 자동 시작 (`restart: unless-stopped`)
2. 호스트 PC에서 카메라 노드 실행:
   ```bash
   ros2 run camera_ros camera_node
   ```
3. RPi5 컨테이너 접속 후 주행 시작:
   ```bash
   docker attach inference-node
   # → 's' 키 입력으로 주행 시작
   ```
4. 주행 경로: 직선 구간 차선 추종 → 슬라롬 구간 장애물 회피 → 출발점 복귀

### 모니터링

```bash
# 노드 상태 실시간 확인 (RPi5)
python3 ~/gyusama-project/scripts/monitor_nodes.py
# [HH:MM:SS]  camera=15.2fps  scan=12.0fps  lane=14.8fps  obs=12.1fps  cmd=14.8fps
#             obs_state: fx=-0.12 fy=+0.34 d=0.52m [AVOID]

# LiDAR 주파수 확인
ros2 topic hz /scan
```

---

## 3. 성능 측정 기준

| 지표 | 목표 | 측정 방법 |
|------|------|-----------|
| 차선 추종 완주율 | 90% 이상 | 5회 시도 중 성공 횟수 |
| 장애물 회피 성공률 | 80% 이상 | 장애물 5개 중 회피 성공 수 |
| 추론 FPS | 10 FPS 이상 | `monitor_nodes.py` camera/lane 항목 |
| LiDAR Hz | 9 Hz 이상 | `ros2 topic hz /scan` |
| 전체 완주 소요 시간 | 측정 후 기록 | 시연 영상 기준 |

---

## 4. 최종 보고서 구성

### 4-1. 주차별 진행 요약

| 단계 | 주차 | 주요 성과 |
|------|------|-----------|
| DevOps | 1~3 | Docker 멀티아키텍처 빌드, docker-compose 오케스트레이션 |
| 시뮬레이션 | 4~6 | TurtleBot3 USD 변환, Isaac Sim 가상 센서, 합성 데이터 생성 |
| AI 모델 | 7~9 | YOLOv8 학습, ONNX INT8 양자화, APF 3노드 구조 |
| 실물 배포 | 10~12 | RPi5 배포, LiDAR 연동, NPU FPS 개선, 실물 주행 |
| 최종 시연 | 13~14 | Docker Hub 파이프라인, 최종 주행 완주 |

### 4-2. Sim-to-Real 성능 비교

시뮬레이션(Isaac Sim)과 실물(RPi5) 간 성능 차이 분석 및 극복 방법 서술

### 4-3. 트러블슈팅 사례

- SSH AP Isolation 우회 (`ip route add`)
- LiDAR 드라이버 불일치 (`LDS-02` 수정)
- ONNX 입력 차원 불일치 (동적 `infer_sz` 적용)
- rsync 경로 앵커링 (`--exclude='/runs/'`)

### 4-4. 개선 방향 제안

- NPU 추론 엔진 최적화 (batch 추론, 전처리 GPU 오프로드)
- ROS2 QoS 튜닝 (센서 토픽 Best-Effort → 지연 감소)
- 자동 파라미터 튜닝 (APF k_rep, influence → 베이지안 최적화)

---

## 5. 완료 기준 체크리스트

- [ ] 차선 추종 트랙 완주율 90% 이상 달성
- [ ] 장애물 배치 시 회피 성공률 80% 이상 달성
- [ ] 전체 시스템 시연 영상 촬영 완료
- [ ] 최종 보고서 작성 및 제출 완료

---

## 6. 최종 시스템 구성 요약

```
[RPi5 호스트]
  camera_node (/camera/image_raw)
        │
[Docker 컨테이너 — bbanggang/gyusama-rpi5:latest]
  inference-node ─→ /lane/cmd_vel
  obstacle-node  ─→ /obstacle/state
  behavior-node  ─→ /cmd_vel (APF 통합)
  control-node   ─→ Dynamixel (TurtleBot3)

[개발·배포 파이프라인]
  PC 코드 수정
    → docker buildx (ARM64 크로스빌드)
      → Docker Hub push (bbanggang/gyusama-rpi5:latest)
        → RPi5 pull & up
```
