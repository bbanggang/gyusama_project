# gyusama-project 진행 정리 3

> 이전 Notion 보고서(`gyusama-project 정리 2`)에서 도출된 두 가지 핵심 잔여 과제 —
> ① RPi5 추론 FPS ~2 (실시간 주행 불가), ② rsync volume mount 의존 배포 —
> 를 해결하고, 실물 환경에서 차선 추종을 단독 검증하기 위한 작업 기록 (Week 11~14).
>
> 전반적인 흐름: **추론 성능 개선 → 배포 자동화 → 실물 검증 → 문서화**
>
> **수치 출처 / 검증 범위**
> - 본 문서의 mAP·데이터셋 통계·파일 크기·코드 위치·Docker Hub 이미지 manifest는 모두
>   PC 로컬 파일과 Docker Hub manifest 직접 조회로 검증된 값입니다 (2026-06-04 검증).
> - RPi5 측 측정값(FPS, 콜백 시간, angular.z, Dynamixel 크래시 로그 등)은 별도 표기 없으면
>   2026-05-27 ~ 2026-06-04 RPi5 실측 로그를 인용한 것이며, 본 검증 세션에서는 직접 재측정하지 않았습니다.

---

## 카테고리 A. 추론 성능 개선 — FPS ~2 → 13~15

### 배경 — 왜 필요한가

- 이전 보고서 시점 RPi5 추론 속도: **FP32 ONNX 기준 ~2 FPS (294ms/frame)**
- 카메라 콜백 주기보다 추론이 느려 차선 추종 명령이 실시간 반응 불가
- 정적 INT8(mAP50=0.000) / 동적 INT8(mAP50=0.432) 모두 실패 → 양자화 경로 폐기
- NPU(Hailo-8L) 도착 전까지 진행할 수 있는 CPU-side 최적화 필요

### A-1. NCNN 추론 엔진 전환

**개념**
- Tencent NCNN: ARM NEON SIMD에 고도 최적화된 추론 엔진
- 가중치 양자화 없이도 ONNX Runtime 대비 ARM에서 2~3배 빠름

**무엇의 향상을 위해**
- ARM(RPi5)에서 비효율적인 ONNX Runtime을 ARM 친화적 엔진으로 교체

**결과**
| 모델 | 해상도 | 엔진 | FPS | 추론 시간 |
|------|--------|------|-----|-----------|
| YOLOv8s | 640×640 | ONNX (구) | ~2 | 294 ms |
| YOLOv8s | 640×640 | **NCNN** | 1.7~1.9 | 317~437 ms |

→ **640×640 단독 전환만으로는 FPS 개선 미미** (연산량 자체가 병목)
→ 입력 해상도 축소와 병행 필요 결론

### A-2. 320×320 모델 재학습

**개념**
- 입력 해상도 640 → 320 축소 시 CNN 연산량 약 4배 감소
- 단, 기존 모델은 640 학습 → 320 입력 시 정확도 손실
- 따라서 320 데이터셋으로 YOLOv8n 재학습 필요

**무엇의 향상을 위해**
- NCNN(엔진 최적화) × 입력 축소(연산량 최소화)를 결합해 10+ FPS 달성

**결과** (`models/runs/lane_det/results.csv` 기준)
| 항목 | 값 |
|------|----|
| 모델 | YOLOv8n, imgsz=320, epochs=100 |
| 환경 | RTX 5070 Ti |
| best epoch | **91** |
| **mAP@50** | **0.95931 (95.9%)** — 목표 0.65 대비 초과 달성 |
| mAP@50-95 | 0.78731 |
| 가중치 크기 | best.pt **6.0MB** / best.onnx **12MB** / ncnn bin **12MB** + param **17KB** |
| 학습 완료 시점 | 2026-05-27 19:47 |

### A-3. 합성 데이터 품질 개선 (Sim-to-Real 갭 축소)

**개념**
- 학습용 합성 데이터(`generate_synthetic_data.py`)와 실주행 USD 트랙(`run_track_sim.py`)의 렌더링 차이로 분포 오염
- 320 재학습 데이터 1000장 생성하면서 동시에 갭 축소

**무엇의 향상을 위해**
- 학습 분포 ≈ 추론 분포 일치 → 모델 일반화 능력 + 실주행 신뢰도 향상

**불일치 3건 수정** (`run_track_sim.py` 기준 정렬)
| 항목 | 수정 전 | 수정 후 |
|------|---------|---------|
| 렌더러 | RTX (omni.hydra.rtx) | **Storm (pxr)** |
| RectLight Y | Y=3.13 (트랙 중앙) | **Y=0** |
| 장애물 색상 | WHITE (차선과 혼동) | **RED** |

**데이터셋 통계 (총 1002장)**
- train **801장** / val **201장** (이미지·라벨 각 동수 확인됨)
- 라벨 있는 이미지 100%, 빈 라벨 0개
- bbox 2개(양쪽 차선) 87.7%, bbox 1개(곡선 등) 12.3%
- 좌/우 bbox 비율 47.9% / 52.1% — 균형 양호

> 통계 분석 시 주의: `xargs cat`은 파일 경계 줄 합쳐짐 → Python으로 파일별 개별 검증

### A-4. NVIDIA 드라이버 이슈 해결 (A-3 진행 중 발생)

**개념**
- Isaac Sim 595 드라이버 RTX API 변경 + CDI 오류로 크래시
- 합성 데이터 생성 작업 자체가 중단

**무엇의 향상을 위해**
- 데이터 재생성 파이프라인 자체의 안정성 회복

**조치**
- 580으로 다운그레이드 → Isaac Sim 정상 동작
- 595 패키지 `rc`(removed but config remains) 잔존 → 커널 업데이트 시 DKMS가 595 재빌드 위험
- `sudo apt purge nvidia-firmware-595-595.71.05` → `dkms status`에 580만 남도록 완전 정리

### A-5. 최종 성능

| 구성 | FPS | 추론 시간 | 콜백 전체 |
|------|-----|-----------|-----------|
| 640 ONNX (구) | ~2 | 294 ms | — |
| 640 NCNN | 1.7~1.9 | 317~437 ms | 539~591 ms |
| **320 NCNN (현)** | **13~15** | **27~33 ms** | **66~77 ms** |
| 향상 비율 | **≈ 8×** | **≈ 13×** | **≈ 8×** |

→ NPU 장착 이전에 **실시간 주행 가능 수준** 확보

---

## 카테고리 B. 배포 자동화 — rsync 의존 제거

### 배경 — 왜 필요한가

- 이전 보고서까지의 배포: PC에서 rsync로 코드 동기화 + RPi5에서 volume mount로 컨테이너 내부 코드 교체
- 문제점:
  - 컨테이너 격리가 깨짐 (호스트 파일 시스템 의존)
  - 모델 가중치 / 코드 버전 불일치 위험
  - RPi5 단독으로 실행 불가 (PC에 종속)
- 목표: 이미지에 코드 내장 → **`docker compose pull && up`만으로 실행**

### B-1. Dockerfile / docker-compose 전환

**개념**
- 빌드 타임에 코드를 이미지 레이어로 COPY → 런타임 mount 제거
- buildx multi-platform으로 PC(amd64)에서 linux/arm64 이미지 크로스 빌드

**무엇의 향상을 위해**
- 배포 신뢰성, 재현성, 운영 단순화

**변경 사항** (모두 2026-05-25 작업)
- `docker/Dockerfile.rpi5` — `COPY models/inference_node/`, `COPY models/runs/`, `COPY config/` 3개 추가
- `docker/docker-compose.yml` — image: `bbanggang/gyusama-rpi5:latest`로 전환
- `.dockerignore` (16줄) 추가 — `*.pt`, `best_int8.onnx`, `best_dynamic_int8.onnx`, `data/`, `weeks_work/`, `isaac_sim/` 등 이미지에서 제외하여 비대화 방지
- `scripts/build_hub.sh` (70줄) — buildx + push 명령 자동화, `--no-push` 옵션 지원

### B-2. 시행착오 — CDI 크래시

**현상**
- PC에서 buildx 컨테이너가 시작 직후 CDI(Container Device Interface) 관련 크래시
- 595 드라이버 이슈와 별개로 daemon 측 CDI 활성화가 builder 컨테이너와 충돌

**조치**
- `/etc/docker/daemon.json`에 `"features": { "cdi": false }` 추가 → 빌더 정상 동작

### B-3. 결과

```bash
# PC: 한 번의 명령으로 빌드 + push
docker buildx build --platform linux/arm64 \
  -t bbanggang/gyusama-rpi5:latest --push \
  -f docker/Dockerfile.rpi5 .

# RPi5: 한 번의 명령으로 실행
docker compose pull && docker compose up -d
```

**Docker Hub 배포 검증** (`docker buildx imagetools inspect bbanggang/gyusama-rpi5:latest`)
- linux/arm64 manifest 존재 확인
- Index digest: `sha256:f38075f3...3146db65`
- arm64 manifest digest: `sha256:4b88b0a2...beeea67d`

→ 코드 핫 리로드가 필요한 튜닝 단계에서는 `models/inference_node/`만 일시적으로
volume mount하여 병행 운영 (현재 docker-compose 15줄에 적용)

---

## 카테고리 C. 실물 차선 추종 검증

### 배경 — 왜 필요한가

- FPS 개선과 배포 단순화 완료 → 이제는 모델이 실주행에서 의도대로 동작하는지 검증
- behavior-node(장애물 회피 블렌딩)가 차선 추종 응답을 가릴 수 있음 → **단독 평가 필요**
- 실물 환경에서만 드러나는 카메라 장착 오프셋, 모터 통신 안정성 문제 식별

### C-1. PC 원격 모니터링 환경 (FastDDS 크로스 머신)

**개념**
- RPi5에는 디스플레이 미연결 → 추론 결과(차선 박스, 오프셋)를 PC에서 시각 확인
- ROS2 토픽을 PC에서 직접 구독하면 RPi5 부하 없이 실시간 확인 가능
- FastDDS는 멀티캐스트로 같은 네트워크의 참여자를 자동 탐색

**문제**
- 공유기가 멀티캐스트 패킷 차단 → 자동 탐색 실패
- 두 머신이 같은 LAN임에도 토픽 발견 불가

**무엇의 향상을 위해**
- 디버깅 가시성 확보 (rqt_image_view에서 `/lane/debug_image` 실시간 확인)

**조치**
- 유니캐스트 피어 XML로 RPi5 IP(192.168.0.155) + ROS_DOMAIN_ID=1 포트 직접 명시
- 도메인 1 → 포트 7650, 7660, 7662, 7664 모두 명시
- `FASTRTPS_DEFAULT_PROFILES_FILE` 환경변수로 적용 후 `ros2 daemon` 재시작

### C-2. behavior-node 우회 — DIRECT_CMD_VEL

**개념**
- 평소 inference-node는 `/lane/cmd_vel`로 발행 → behavior-node가 obstacle/lane 블렌딩 후 `/cmd_vel`로 전달
- 차선 추종 모델의 출력 품질만 평가하려면 블렌딩 단계 제거 필요

**무엇의 향상을 위해**
- 모델 단독 평가 가능 → 차선 인식·제어 게인의 직접적 검증

**구현**
- `lane_detect_ncnn.py` 302~307줄: `DIRECT_CMD_VEL=1`이면 `/cmd_vel`로 직접 발행
- `docker-compose.yml` 25줄: `DIRECT_CMD_VEL=${DIRECT_CMD_VEL:-0}` 추가 (쉘 변수로 토글)
- 튜닝 단계 한정 — `models/inference_node`를 volume mount하여 핫 리로드 (compose 15줄)

### C-3. 디버그 발행 토글 — PUBLISH_DEBUG

**개념**
- `/lane/debug_image` 발행 시 JPEG 인코딩 + 토픽 publish에 약 170ms/frame 소요
- 시각화가 ON이면 FPS가 크게 떨어짐

**무엇의 향상을 위해**
- 주행 검증(고FPS) ↔ 시각 디버깅(rqt 확인)을 1초 만에 전환

**측정 결과** (RPi5 실측 — 본 PC 검증 범위 외)
| PUBLISH_DEBUG | FPS | 콜백 시간 | rqt 확인 |
|---------------|-----|-----------|----------|
| 0 (기본) | **14.7** | 36 ms | ❌ |
| 1 | 4.7 | 206 ms | ✅ |

**구현**
- `lane_detect_ncnn.py` 473줄: `os.environ.get('PUBLISH_DEBUG', '1') != '0'` (코드 기본값 발행 ON, compose에서 OFF로 override)
- `docker-compose.yml` 24줄: `PUBLISH_DEBUG=${PUBLISH_DEBUG:-0}` 도입
- 필요시 `PUBLISH_DEBUG=1 docker compose up -d`로 일시 활성화

### C-4. 카메라 시스템 오프셋 진단

**현상**
- 실주행 중 `angular.z`가 **항상 -1.49~-2.28 (우회전만)**
- 90+ 샘플(20초)에서 양의 angular.z 한 번도 관측 안 됨
- rqt 디버그 이미지: 양쪽 차선 박스 모두 정상 검출
- `LANE off: +0.24~+0.32`로 일정한 양의 오프셋 누적

**개념 — `compute_lane_offset()`**
- `offset = (mid_cx - img_w/2.0) / (img_w/2.0)`
- mid_cx가 이미지 중심보다 75~100px 우측 → 모델은 항상 "중심에서 우측으로 벗어남"으로 판단
- → 좌회전(angular.z > 0)으로 보정해야 함에도 코드 부호상 우회전 명령 발생

**무엇의 향상을 위해**
- 직선 주행 검증 가능 (현재는 카메라 편향으로 우측 벽 충돌 직진)

**대응 방안 (검토 중)**
- **A안**: 카메라 물리 장착 위치를 로봇 중심선으로 정렬
- **B안**: `apf_params.yaml`에 `center_bias_px` 추가 → SW 보정
- A 우선 검토 후 잔여 편향을 B로 보정 예정

### C-5. Dynamixel 통신 안정성

**현상**
- `/cmd_vel` 정상 발행 중에도 모터 무반응
- `ros2 topic info /cmd_vel`: 구독자 수 = 0 → control-node 내 turtlebot3_node 크래시

**원인**
- `DynamixelSDK: Failed to read: no status packet` 3회 연속 → `stack smashing detected` 크래시
- 컨테이너 자체는 생존 → `restart: unless-stopped` 미발동 (subprocess 죽음)

**임시 조치**
- `docker restart control-node` → turtlebot3_node 재기동, 구독자 수 1 복원

**잔여 과제**
- 컨테이너 헬스 체크 또는 watchdog (turtlebot3_node liveness 감시) 추가 필요

---

## 카테고리 D. 문서화

### D-1. 최종 보고서 초안

**무엇을 위한 작업인가**
- W1~W10 완료 내용을 보고서 골격으로 미리 정리
- 시연 영상만 추가하면 제출 가능한 상태로 만들기

**산출물** (`weeks_work/week14/report_draft.md`, 총 663줄)
- 본문 **9개 챕터** + 부록 A(주요 파일 목록) / 부록 B(환경변수 설정 목록)
  1. 프로젝트 개요
  2. 개발 환경 및 시스템 구성
  3. 주차별 진행 요약 (W1~W13)
  4. 핵심 기술 구현 상세
  5. Sim-to-Real 갭 분석
  6. 트러블슈팅 사례
  7. 성능 측정 결과
  8. 시연 결과 (NPU 이후 채움)
  9. 개선 방향
- 트러블슈팅 사례 **T-01 ~ T-11** (11건 모두 작성 완료)
- Sim-to-Real 갭 분석 표 (하드웨어 / 소프트웨어 / 도메인 3개 축)
- 시연 결과(8장)는 NPU 도착 후 추가 예정

---

## 첨부 권장 이미지 / 자료

검증과 가독성을 위해 다음 자료를 보고서/슬라이드에 함께 첨부하면 좋습니다.

| 카테고리 | 자료 종류 | 확인 가능한 것 |
|---------|-----------|----------------|
| **A-1, A-5** | FPS 비교 막대 그래프 (640 ONNX / 640 NCNN / 320 NCNN) | 8× 성능 향상 한눈에 확인 |
| **A-2** | YOLOv8n 학습 곡선 (`results.png`, PR-curve, F1-curve) | mAP@50=95.9% 신뢰성 |
| **A-2** | val 예측 결과 샘플 (`val_batch0_pred.jpg`) | 320 모델 검출 품질 |
| **A-3** | 합성 데이터 수정 전/후 비교 (RTX vs Storm, RectLight, 장애물 색상) | Sim-to-Real 갭 축소 |
| **A-3** | 합성 데이터 1000장 라벨 통계 차트 (bbox 수, 좌/우 비율) | 데이터 균형 검증 |
| **B-1** | 배포 흐름 다이어그램 (구 rsync → 신 buildx push → RPi5 pull) | 파이프라인 단순화 시각화 |
| **B-2** | `docker buildx ls` + Docker Hub 태그 캡처 | 멀티 아키 빌드 증빙 |
| **C-1** | PC에서 rqt_image_view로 `/lane/debug_image` 본 캡처 | 크로스 머신 모니터링 동작 |
| **C-2** | `docker compose ps` 캡처 (inference-node만 동작, behavior-node 정지) | 단독 테스트 구성 검증 |
| **C-3** | `[PERF]` 로그 비교 (PUBLISH_DEBUG=0 vs 1) | FPS 트레이드오프 가시화 |
| **C-4** | rqt 디버그 이미지 (`LANE off: +0.32` 표시 + 양쪽 차선 박스) | 모델은 정상, 카메라 편향 근거 |
| **C-4** | TurtleBot3 측면 사진 (카메라-중심선 위치) | 물리적 좌측 편향 확인 |
| **C-4** | angular.z 시계열 플롯 (20초, -1.5~-2.3 일정) | 우회전 편향 객관화 |
| **C-5** | `docker logs control-node` (stack smashing 메시지) | 크래시 패턴 기록 |
| **전체** | 실주행 단축 영상 또는 GIF (10~20초) | 차선 추종 동작 시연 |

---

## 진행 로그

| 날짜 | 카테고리 | 작업 | 결과 |
|------|---------|------|------|
| 2026-05-25 | B-1 | Dockerfile COPY 추가, docker-compose image 전환, .dockerignore, build_hub.sh | 완료 |
| 2026-05-25 | B-1 | buildx ARM64 push → RPi5 pull & up 검증 | 완료 |
| 2026-05-26 | A-1 | `lane_detect_ncnn.py` 작성, ONNX→NCNN 변환, 이미지 재빌드·배포 | 완료 |
| 2026-05-26 | A-4 | NVIDIA 595 크래시 → 580 다운그레이드, 595 패키지 완전 제거 | 완료 |
| 2026-05-27 | A-3 | `generate_synthetic_data.py` 씬 불일치 3건 수정 (렌더러/조명/색상) | 완료 |
| 2026-05-27 | A-3 | 합성 데이터 1000장 생성 (train 801 / val 201) | 완료 |
| 2026-05-27 | A-2 | YOLOv8n 320×320 학습 (mAP@50=95.9%, epoch 91) | 완료 |
| 2026-05-27 | B-2 | buildx CDI 크래시 → daemon.json features.cdi=false 적용 후 ARM64 push | 완료 |
| 2026-05-27 | A-5 | RPi5 실측 — 320 NCNN 13~15 FPS (640 NCNN 대비 8×) | 완료 |
| 2026-05-27 | D-1 | 최종 보고서 초안 (week14/report_draft.md, 9장, T-11) | 완료 |
| 2026-06-04 | C-1 | FastDDS 유니캐스트 피어 XML 적용, PC rqt 확인 성공 | 완료 |
| 2026-06-04 | C-2 | `DIRECT_CMD_VEL` 환경변수 추가, behavior-node 우회 단독 테스트 구성 | 완료 |
| 2026-06-04 | C-3 | `PUBLISH_DEBUG=${PUBLISH_DEBUG:-0}` 토글 도입, FPS 14.7 복구 | 완료 |
| 2026-06-04 | C-4 | angular.z 우편향 진단 — 카메라 좌측 장착 추정, 보정안 검토 중 | 진행 중 |
| 2026-06-04 | C-5 | control-node stack smashing 크래시 발견, restart로 복구 | 진행 중 |

---

## 다음 단계

```
🔄 C-4 카메라 오프셋 보정 (물리 위치 우선, 잔여 편향은 center_bias_px)
🔄 C-5 control-node watchdog/health check 추가
⏸ behavior-node 통합 재개 → 차선 추종 + 장애물 회피 동시 동작 검증
⏸ NPU(Hailo-8L) 도착 후:
   - HEF 변환 + 추론 파이프라인 전환
   - APF 파라미터 실물 튜닝
   - 시연 영상 촬영 → report_draft.md 시연 챕터 완성
```
