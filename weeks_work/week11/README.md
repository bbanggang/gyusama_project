# 11주차 진행 보고 — NPU 장착 및 추론 FPS 개선

> **목표**: NPU를 RPi5에 장착하여 추론 FPS를 10+ 이상으로 끌어올리고,
> 차선 추종 + 장애물 회피 동시 주행을 실물 트랙에서 검증한다.

---

## 1. 배경 — 10주차에서 이어짐

| 항목 | 상태 |
|------|------|
| RPi5 전체 노드 기동 | ✅ 완료 |
| 장애물 감지 → 정지 | ✅ 확인 |
| 추론 FPS | ❌ 2~3 FPS (목표 10+) |
| 동시 주행 테스트 | ⏸ 이번 주 진행 |

FPS 병목 원인: FP32 640×640 ONNX 모델 → RPi5 CPU 추론 300~430 ms/frame

---

## 2. FPS 개선 방법 비교

| 방법 | 예상 FPS | 비고 |
|------|----------|------|
| INT8 모델 (`best_int8.onnx`) | 6~10 FPS | 모델 경로 변경만으로 적용 가능 |
| 입력 해상도 320×320 축소 | 8~12 FPS | `infer_sz` 동적 적용으로 자동 반영 |
| NCNN 추론 (`lane_detect_ncnn.py`) | 10~15 FPS | ARM 최적화 추론 엔진 |
| **NPU 추론** | **20~30 FPS** | **최종 목표** |

---

## 3. 구현 계획

### 11-1. NPU 장착 및 드라이버 설치

- NPU 하드웨어 연결 (USB / M.2 / HAT 방식에 따라 절차 상이)
- 드라이버 및 런타임 설치
- Dockerfile.rpi5 업데이트 (NPU 런타임 의존성 추가)

### 11-2. 추론 노드 NPU 전환

- `lane_detect.py` — NPU execution provider 적용
- ONNX Runtime NPU 지원 또는 별도 NPU SDK 연동
- 기존 `infer_sz` 동적 로드 구조 유지

### 11-3. 실물 동시 주행 테스트

```bash
# 배포
./scripts/deploy_rpi5.sh --build

# 모니터링
ssh rapi5@192.168.0.155 "python3 ~/gyusama-project/scripts/monitor_nodes.py"
```

테스트 시나리오:
1. 직선 구간 차선 추종 확인
2. 슬라롬 구간 장애물 회피 확인
3. 차선 추종 중 장애물 출현 시 회피 후 복귀 확인

### 11-4. Sim-to-Real 갭 분석 문서화

| 항목 | Isaac Sim | RPi5 실물 | 차이 |
|------|-----------|-----------|------|
| 추론 FPS | ~30 FPS | 2~3 FPS (NPU 전) | 하드웨어 차이 |
| LiDAR Hz | 10 Hz | 9.4 Hz | ≈동일 |
| 차선 검출 신뢰도 | 0.7~0.9 | 측정 예정 | 조명 차이 영향 |
| APF `influence` | 0.65 | 재튜닝 필요 | 실물 LiDAR 노이즈 |
| APF `k_rep` | 0.8 | 재튜닝 필요 | 실물 제동거리 차이 |

---

## 4. APF 파라미터 실물 튜닝 순서

1. `stop_dist` — 정지 거리 확인 (로봇이 실제로 멈추는 위치)
2. `slow_dist` / `influence` — 감속 시작 거리 확인
3. `k_rep` — 회피 각속도 세기 조정
4. `max_angular` — 차선 이탈 여부 보고 제한

```bash
# 파라미터 수정 후 재시작 (재빌드 불필요)
vi config/apf_params.yaml
./scripts/deploy_rpi5.sh --restart
```

---

## 5. 완료 기준 체크리스트

- [ ] NPU 장착 및 드라이버 설치 완료
- [ ] 추론 FPS 10 이상 달성
- [ ] 차선 추종 단독 주행 실물 확인
- [ ] 장애물 회피 + 차선 추종 동시 주행 확인
- [ ] 실물 APF 파라미터 튜닝 완료 (`config/apf_params.yaml` 업데이트)
- [ ] Sim-to-Real 갭 분석 문서화

---

## 6. 트러블슈팅 가이드

| 증상 | 원인 | 조치 |
|------|------|------|
| NPU 미인식 | 드라이버 미설치 | NPU SDK 재설치 확인 |
| NPU 사용 중 추론 오류 | execution provider 설정 오류 | `CPUExecutionProvider` fallback 후 로그 확인 |
| 회피 후 차선 이탈 | `k_rep` 또는 `max_angular` 과대 | 값 낮추기 |
| 회피 방향 진동 | `latch_dist` 부족 | 값 높이기 |
| 정지 후 미출발 | `stop_dist` 과대 | 값 낮추기 |
