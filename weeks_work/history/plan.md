# gyusama-project 최종 진행 계획 — NPU 도입 후 시연까지

> NPU(Hailo-8L) 도착 완료 → 640×640 재학습 + 실시간 추론 검증 → 실세계 파라미터 튜닝 →
> 최종 시연 + 보고서 완성까지의 단계별 계획.
>
> **전체 흐름**
> ```
> Phase 1 NPU 셋업 → Phase 2 640 재학습 → Phase 3 HEF 변환 →
> Phase 4 추론 노드 교체 → Phase 5 FPS 검증 → Phase 6 실세계 튜닝 →
> Phase 7 통합 시나리오 → Phase 8 안정성 보완 → Phase 9 시연/보고서
> ```
>
> **현재 baseline** (NPU 도입 전 기준 — 2026-06-04 PC 측 검증)
> - 모델: YOLOv8n 320×320 NCNN, mAP@50 = 0.95931 ✅ 파일로 확인
> - RPi5 FPS: 13~15 (콜백 66~77 ms) — 이전 RPi5 실측 로그 인용
>
> **저장소 측 검증 완료 (확정)**
> - FastDDS XML **영구 설정 없음** → Phase 8-②
> - 컨테이너 **로그 회전 미설정** → Phase 8-③
> - Dockerfile에 **HailoRT 미설치** → Phase 1
> - `compute_lane_offset` NCNN/ONNX **2곳 중복 정의** → Phase 4 선행 리팩토링
> - control-node에 **healthcheck 없음** (코드/설정 확인) → Phase 8-① 인프라 측면은 보강 가능
>
> **재검증 필요 (실물 확인 후 처리)**
> - ⚠️ 카메라 좌측 편향 (`LANE off +0.24~+0.32`): 이전 세션 로그에만 존재 → **Phase 6 시작 시 RPi5 재주행으로 재현 여부 먼저 확인**, 없으면 `center_bias_px` 추가 작업 스킵
> - ⚠️ turtlebot3_node stack smashing: 이전 1회 발생 → **상시 재현되는지 확인**, 일시적 문제라면 healthcheck로 충분, 상습적이면 USB/펌웨어 점검 필요
>
> 통합 BT는 시뮬에서만 검증, 실물 미완.

---

## Phase 1. Hailo-8L 하드웨어/소프트웨어 셋업

### 목표
RPi5에 Hailo-8L를 장착하고 `hailortcli`로 디바이스 인식 확인까지 완료.

### 체크리스트
- [ ] M.2 HAT+ 또는 AI Kit 장착, PCIe 인식 확인
  ```bash
  lspci | grep -i hailo        # 0000:01:00.0 처럼 보이면 OK
  ```
- [ ] PCIe 드라이버 설치 (`hailort-pcie-driver`)
- [ ] **HailoRT 런타임** 설치 (RPi5 aarch64 .deb)
- [ ] **HailoRT Python 바인딩** (`hailort` pip 패키지) 설치
- [ ] 디바이스 인식 검증
  ```bash
  hailortcli fw-control identify
  # → Identifying board, Board Name: Hailo-8L
  ```
- [ ] Dockerfile.rpi5에 HailoRT 통합
  ```dockerfile
  # /etc/apt/sources.list.d 에 hailo repo 추가 또는 .deb COPY 후 dpkg -i
  RUN dpkg -i /tmp/hailort_*_arm64.deb
  RUN pip3 install --break-system-packages hailo-platform
  ```
- [ ] 컨테이너 내부에서도 `hailortcli identify` 동작 확인 (디바이스 마운트 `/dev/hailo0`)

### 완료 기준
- 호스트와 컨테이너 양쪽에서 NPU가 인식됨
- 샘플 추론 (Hailo가 제공하는 yolov8 demo) 실행 성공

---

## Phase 2. 640×640 모델 재학습

### 목표
NPU 활용을 전제로 입력 해상도를 640×640으로 올려 차선 검출 정확도를 1차 향상.

### 학습 설정

> **확인 결과**: `train_yolo_lane.py`의 디폴트가 이미 `--model yolov8s --imgsz 640 --epochs 100 --batch 16`.
> 320 학습은 명시적으로 `--imgsz 320 --model yolov8n`을 줬던 경우.
> 따라서 **별도 인자 없이 그냥 실행하면 640 학습**.

권장 인자
- `--epochs 150` (해상도 증가에 따른 수렴 마진 확보 — 디폴트보다만 늘림)
- 나머지는 디폴트 유지
- 필요 시 `--model yolov8n`으로 더 가볍게 (NPU 여유 있으면 `yolov8s` 권장)

### 데이터셋
- 기존 `data/synthetic/dataset.yaml` 그대로 사용 (1002장)
- **해상도 무관**한 합성 데이터지만, 잘게 보이는 차선 마커 패턴을 활용 가능해 mAP 추가 상승 기대
- 필요 시 추가 1000장 더 생성 (curve / 교차 구간 비중 증가)

### 체크리스트
- [ ] `train_yolo_lane.py --model yolov8s --imgsz 640 --epochs 150`
- [ ] 학습 곡선 확인 — `models/runs/lane_det-N/results.png`
- [ ] **mAP@50 ≥ 0.96** 목표 (320 기준 0.959 대비 +α)
- [ ] best.pt / best.onnx 생성 확인
- [ ] val 예측 샘플(`val_batch0_pred.jpg`) 정성 검증

### 완료 기준
- `models/runs/lane_det-N/weights/best.onnx` 12~20MB 규모로 존재
- mAP@50, mAP@50-95 baseline 대비 동등 이상

---

## Phase 3. ONNX → HEF 변환 (Hailo 컴파일)

### 목표
PC에서 YOLOv8 ONNX → Hailo HEF 컴파일 (RPi5에서 NPU가 직접 실행 가능한 포맷).

### 변환 도구
- **Hailo Dataflow Compiler (DFC)** — PC에서 실행 (Docker 권장)
- Hailo Model Zoo의 YOLOv8 예시 retraining/conversion 스크립트 활용

### 캘리브레이션
- HEF는 INT8 양자화가 필수 — **캘리브레이션 데이터 필요**
- val 셋 201장 중 약 64~128장을 캘리브 셋으로 사용
- 학습 분포(합성)와 실주행 분포 차이로 정확도 하락 가능 → 실주행 일부 캡처 혼합 권장

### 체크리스트
- [ ] PC에 Hailo DFC 컨테이너 설치 (`hailo_ai_sw_suite_docker`)
- [ ] best.onnx → har (Hailo intermediate) 변환
- [ ] 캘리브 셋 준비 (val에서 100장 + 실주행 캡처 30장 혼합)
- [ ] har → HEF 컴파일 (INT8 양자화 적용)
- [ ] **양자화 후 mAP@50 검증** — 손실 5%p 이내 목표
  - 이전 INT8 실패 사례 있음 → 캘리브 셋이 핵심
  - 실패 시: 캘리브 셋 확대, mixed precision, layer-wise QAT 고려
- [ ] HEF 파일 RPi5로 전송 (`models/runs/lane_det-N/weights/best.hef`)

### 완료 기준
- best.hef 파일 생성 (수 MB)
- Hailo DFC profile 결과: 예상 FPS 30+ 표시

---

## Phase 4. RPi5 Hailo 추론 노드 구현

### 목표
NCNN 노드를 NPU 노드로 교체하되, 환경변수로 toggle 가능하게 유지.

### 새 파일
- `models/inference_node/lane_detect_hailo.py`
  - 기존 `lane_detect_ncnn.py` 구조 복사
  - NCNN 호출부만 HailoRT API로 교체
  - 전후처리(`compute_lane_offset`, ROI, bbox 역변환) 그대로 재사용

### 선행 리팩토링 권장 (검증 기반)
- 현재 `compute_lane_offset` / `_col_center` / ROI 계산이 NCNN·ONNX 두 파일에 **중복 정의**되어 있음 (`lane_detect_ncnn.py:193`, `lane_detect.py:176`)
- Hailo 노드가 추가되면 3중 복제 → 유지보수 부담
- `models/inference_node/lane_common.py`로 추출 후 3개 노드가 import 하는 구조 권장
- 이 단계에서 `center_bias_px` 적용도 한 곳에서 끝남 (Phase 6-A의 패치 비용도 줄어듦)

### 환경변수 설계
```yaml
# docker-compose.yml
- INFER_ENGINE=${INFER_ENGINE:-hailo}   # hailo | ncnn (롤백 가능)
- HEF_PATH=/ros2_ws/models/runs/lane_det/weights/best.hef
```
- compose `command:`에서 `INFER_ENGINE` 분기 또는 entrypoint에서 분기

### 체크리스트
- [ ] `lane_detect_hailo.py` 작성 (HailoRT VDevice / InferModel API)
- [ ] HEF 로드 + 단일 프레임 추론 동작 확인
- [ ] /camera/image_raw 콜백 → HEF 추론 → /lane/cmd_vel 발행
- [ ] DIRECT_CMD_VEL / PUBLISH_DEBUG 토글 유지 (NCNN 노드와 동일 인터페이스)
- [ ] docker-compose.yml에 INFER_ENGINE 분기 추가
- [ ] Dockerfile.rpi5에 HailoRT 의존성 추가, 이미지 재빌드 + Hub push

### 완료 기준
- `INFER_ENGINE=hailo docker compose up -d` 로 정상 기동
- `/lane/cmd_vel` 발행 확인
- 롤백 시 `INFER_ENGINE=ncnn` 으로 즉시 복귀 가능

---

## Phase 5. 추론 속도 검증

### 목표
NPU 추론 FPS가 실시간 제어에 충분한지 정량 확인.

### 측정값
| 지표 | 목표 |
|------|------|
| 추론 시간 (model only) | < 15 ms |
| 콜백 전체 시간 | < 40 ms |
| FPS | **≥ 25** (목표 30) |
| /lane/cmd_vel 발행 주기 | ≥ 20 Hz |

### 체크리스트
- [ ] `[PERF]` 로그로 100프레임 평균 측정 (NCNN 노드와 동일 포맷)
- [ ] `/lane/cmd_vel` 실측 Hz (`ros2 topic hz /lane/cmd_vel`)
- [ ] CPU 점유율 — `top` 또는 `docker stats` (NPU 사용 시 CPU 여유 확인)
- [ ] NPU 사용률 — `hailortcli monitor`
- [ ] PUBLISH_DEBUG=1 일 때도 10 FPS 이상 유지되는지 확인

### 완료 기준
- 평균 FPS 25 이상, 최저 20 이상
- 미달 시:
  - 전후처리 병목 분석 (`time.perf_counter()` 구간 측정)
  - 카메라 publish QoS / 큐 크기 확인
  - 입력 전처리 GPU/NEON 가속 (cv2 → libcamera native)

---

## Phase 6. 실세계 파라미터 튜닝

### 6-A. 카메라 시스템 오프셋 보정 (조건부)

**[0] 재현 여부 먼저 확인** ← 실물 측 작업의 시작점
- [ ] `DIRECT_CMD_VEL=1, PUBLISH_DEBUG=1`로 직선 트랙 정지 상태 주행
- [ ] 20초 이상 angular.z 시계열 측정 — 한쪽 방향만 출력되는지
- [ ] `LANE off` 평균값 측정 — |값| > 0.1 이면 편향 인정
- [ ] 편향이 없거나 미미하면 (|off| ≤ 0.05) **6-A 자체를 스킵하고 6-B로 진행**

**현상 가설 (이전 1회 측정 — 재현 시 적용)**
- angular.z 항상 -1.49 ~ -2.28 (우회전만)
- `LANE off: +0.24 ~ +0.32` 일정 양수
- mid_cx가 이미지 중심보다 75~100px 우측

**조치 순서** (재현 확인 시)
1. **물리 정렬** — 카메라 마운트를 로봇 중심선으로 조정 후 재측정
2. **잔여 보정** — `config/apf_params.yaml` `lane_detect` 섹션에 `center_bias_px` 추가
   ```yaml
   lane_detect:
     center_bias_px: 0   # 양수 = mid_cx 좌측 보정
   ```

> **코드 패치 위치 (검증됨)**
> `compute_lane_offset`이 **두 파일에 중복 정의**되어 있어 양쪽 모두 패치 필요:
> - `models/inference_node/lane_detect_ncnn.py:193` (현재 사용 중)
> - `models/inference_node/lane_detect.py:176` (시뮬용)
> - 향후 `lane_detect_hailo.py`까지 더해지면 3중 → **공통 utils 모듈로 분리** 권장
>
> 또한 `mid_cx` 계산이 **두 분기**에서 일어남 — 양쪽 모두 적용해야 함:
> - 단일 박스 분기 (`lane_detect_ncnn.py:213, 216`): `mid_cx = cx ± img_w*0.25`
> - 이중 박스 분기 (`lane_detect_ncnn.py:227`): `mid_cx = (left_cx + right_cx) / 2.0`
> - 둘 다 직후에 `mid_cx -= center_bias_px` 적용

**완료 기준**
- 직선 주행 5초 동안 angular.z 평균이 |0.3| rad/s 이하
- LANE off 절댓값 0.05 이하

### 6-B. 차선 추종 튜닝 (`lane_detect` 섹션)

| 파라미터 | 현재값 | 튜닝 방향 |
|---------|--------|-----------|
| `max_speed` | 0.14 m/s | 안정화 후 0.18 ~ 0.22까지 상향 |
| `min_speed` | 0.08 m/s | 곡선 통과 가능한 최저속도로 |
| `kp_angular` | 11.0 | 진동 시 ↓ (7~8), 반응 느리면 ↑ |
| `steer_exp` | 1.4 | 1.0 (선형)부터 시작, 큰 오차에서만 1.4로 |
| `lane_roi_top` | 0.50 | 곡선 추종 약하면 0.40 (시선 멀리 보기) |
| `conf_thresh` | 0.12 | NPU 정확도 ↑ 시 0.20~0.30 상향 가능 |

**튜닝 절차**
- [ ] `DIRECT_CMD_VEL=1` 단독 모드에서 직선 주행 안정화 (kp, steer_exp)
- [ ] 좌/우 완만한 곡선 추종 (lane_roi_top, steer_exp)
- [ ] 급커브 추종 (max_angular, steer_exp 비선형성)
- [ ] 속도 점진 상향 (max_speed) — 각 단계에서 직선/곡선 모두 통과 확인

### 6-C. 장애물 회피 튜닝 (`obstacle_node` 섹션)

**LDS-02 노이즈 차이 (시뮬과 다름)** 대응
| 파라미터 | 현재값 | 튜닝 포인트 |
|---------|--------|------------|
| `influence` | 0.65 m | 실측 LiDAR 노이즈 확인 후 조정 |
| `min_obs_points` | 5 | LDS-02 노이즈 많으면 8~10으로 ↑ |
| `front_deg` | 22° | 좁은 차선에서 벽 오인 시 18° 이하로 |
| `latch_dist` | 0.55 m | 회피 늦으면 0.65로 ↑ |
| `stop_dist` | 0.22 m | 안전 마진 따라 조정 |
| `slow_dist` | 0.60 m | latch_dist 보다 커야 자연스러운 감속 |

**튜닝 절차**
- [ ] LiDAR raw scan 검사 — 차선 벽까지의 거리 분포 측정 (`ros2 topic echo /scan`)
- [ ] 차선 벽 영향 받지 않는 `front_deg`, `influence` 조합 탐색
- [ ] 정적 장애물 1개 → clear/warn/avoid/stop 전이 시점 측정
- [ ] `latch` 방향 안정성 검증 (flip 발생 빈도)
- [ ] 정지 거리 stop_dist 정밀 측정 (TurtleBot3 관성 포함)

### 6-D. APF 합성 튜닝 (`behavior_manager` 섹션)

| 파라미터 | 현재값 | 튜닝 포인트 |
|---------|--------|------------|
| `k_rep` | 0.8 | 회피 거리 부족 시 ↑, 과도하면 ↓ |
| `lane_min` | 0.35 | 회피 중 차선 인력 최소치 — 차선 완전 무시 vs 일부 유지 |
| `max_angular` | 0.5 rad/s | 급선회 한계 |
| `obs_timeout` | 0.5 s | LiDAR 끊김 시 안전 정지 타이밍 |

**튜닝 절차**
- [ ] 회피 중 차선 이탈 없이 복귀하는 `lane_min` 탐색
- [ ] 측면 장애물 통과 시 `k_rep` 검증
- [ ] 회피 후 자연스러운 복귀(lane_w → 1.0) 시각화

### 6-E. 안전 가드 (튜닝 중 반드시 켜둠)
- E-stop: 키보드 / 무선 / 토픽 기반
- 최대 속도 상한 안전치 (튜닝 중 0.15 m/s 절대 초과 방지)
- LiDAR 미수신 시 즉시 정지 (`obs_timeout`)

---

## Phase 7. 통합 시나리오 검증

### 시나리오 (각 3회 이상 반복, 성공률 기록)
1. **단순 직선 완주** — 차선 추종만, 장애물 없음
2. **곡선 완주** — 좌/우 곡선 모두
3. **정적 장애물 1개** — 정면 회피 → 복귀
4. **정적 장애물 2개** — 연속 회피
5. **곡선 + 장애물** — 곡선 중 측면 장애물 회피
6. **차선 끊김 구간** — 차선 미검출 시 정지 또는 직진 유지

### 측정 지표
| 지표 | 목표 |
|------|------|
| 차선 추종 완주율 (시나리오 1~2) | 100% |
| 장애물 회피 성공률 (시나리오 3~5) | ≥ 90% |
| 평균 FPS | ≥ 25 |
| /cmd_vel 평균 Hz | ≥ 20 |
| stop_dist 정지 정확도 | ±5 cm |

### 체크리스트
- [ ] 각 시나리오별 영상 녹화 (외부 카메라)
- [ ] 토픽 rosbag 기록 (재현 가능성 확보)
- [ ] 실패 케이스 원인 분류 (인지 / 판단 / 제어 / 통신)

---

## Phase 8. 시스템 안정성 보완

### 알려진 이슈 — 우선순위 순

**① turtlebot3_node stack smashing 크래시 (재검증 필요)**
- 현상 (이전 1회): DynamixelSDK "no status packet" 3회 → 프로세스 죽음, 컨테이너는 생존
- **[0] 재현 빈도 먼저 확인** — 실주행 30분 이상 운영하며 크래시 발생 횟수 기록
  - 0회: USB 일시 노이즈로 판정, healthcheck만 추가
  - 1회 이상: USB 케이블/허브 점검, OpenCR 펌웨어 재플래시 고려
- **저장소 측 확인 결과**: docker-compose.yml 74줄에 `restart: unless-stopped`만 있고 **healthcheck 없음** → 컨테이너가 살아있는 한 재시작 안 됨
- 해결안 (빈도와 무관하게 안전망으로 적용 권장):
  - [ ] control-node에 **healthcheck** 추가
    ```yaml
    healthcheck:
      test: ["CMD-SHELL", "ros2 topic info /cmd_vel | grep -q 'Subscription count: [1-9]'"]
      interval: 5s
      timeout: 3s
      retries: 3
    ```
  - [ ] turtlebot3_node 죽으면 컨테이너 자동 재시작 (`exit on subprocess death` 래퍼 또는 `ros2 launch` 종료 코드 propagate)
  - [ ] `/dev/ttyUSB0` USB 끊김 감지 후 자동 reconnect

**② FastDDS 크로스 머신 안정성**
- **현재 상태 (확인됨)**: `scripts/`, `config/`, repo 어디에도 영구 XML 없음 → 매번 `/tmp/fastdds_peer.xml`로 수동 생성
- 해결안:
  - [ ] `config/fastdds_peer.xml`을 repo에 커밋 (RPi5 IP는 환경변수로 치환)
  - [ ] PC `~/.bashrc`에 `export FASTRTPS_DEFAULT_PROFILES_FILE=~/gyusama-project/config/fastdds_peer.xml`
  - [ ] 부팅 시 자동 적용 systemd unit 검토 (필요 시)

**③ 컨테이너 로그 회전**
- **현재 상태 (확인됨)**: docker-compose.yml에 `logging:` 키 자체가 없음 → 기본(json-file 무제한)
- 해결안:
  - [ ] 각 서비스에 추가
    ```yaml
    logging:
      driver: json-file
      options:
        max-size: "50m"
        max-file: "3"
    ```
  - [ ] 장시간 운영 시 디스크 사용량 통제 (실측 후 사이즈 조정)

**④ RPi5 부팅 시 자동 시작 (선택)**
- **현재 상태 (확인됨)**: `scripts/start_nodes.sh`는 수동 실행만, systemd 미연동
- 시연 당일 안정성을 위해 권장:
  - [ ] `systemd unit` 작성 (`/etc/systemd/system/gyusama.service`)
  - [ ] `ExecStart=docker compose up -d` + `Restart=on-failure`

---

## Phase 9. 최종 시연 + 보고서

### 시연 영상
- [ ] **메인 시연 영상 (1~2분)** — 직선 + 곡선 + 회피 + 복귀를 한 번에
- [ ] **분리 시연 영상**
  - 차선 추종 단독 (DIRECT_CMD_VEL=1)
  - 장애물 회피 단독
  - 통합 BT 동작
- [ ] **NPU 가속 비교 영상** — NCNN 13 FPS vs Hailo 30 FPS 분할 화면

### 보고서 마무리 (`weeks_work/week14/report_draft.md`)
- [ ] **8장 시연 결과** 채움
  - 완주율 / 회피 성공률 / 최종 FPS 표
  - 실패 케이스 분석
- [ ] **7장 성능 측정 결과** 업데이트 — NPU 수치 추가
- [ ] **5장 Sim-to-Real 갭** — 튜닝 전후 비교 표 추가
- [ ] **6장 트러블슈팅** — NPU 도입 과정의 신규 이슈 (T-12~) 추가
- [ ] **9장 개선 방향** — 잔여 한계 정리

### 산출물 정리
- [ ] `models/runs/lane_det-N/` 최종 학습 디렉토리 정리 (best.pt/.onnx/.hef 보존)
- [ ] `config/apf_params.yaml` 최종 튜닝값 커밋
- [ ] `weeks_work/week14/README.md` 최종 정리
- [ ] `weeks_work/history/work4.md` 작성 (NPU 도입 ~ 시연 전체 회고)

---

## 일정 (제안)

| Phase | 예상 소요 | 비고 |
|-------|----------|------|
| 1. Hailo 셋업 | 0.5~1일 | 하드웨어 + 드라이버 |
| 2. 640 재학습 | 0.5일 | 100~150 epoch ≈ 3~5시간 |
| 3. HEF 변환 | 1일 | INT8 캘리브가 변수 |
| 4. 추론 노드 구현 | 1일 | NCNN 노드 기반 포팅 |
| 5. FPS 검증 | 0.5일 | 측정 + 디버그 |
| 6. 실세계 튜닝 | **2~3일** | 가장 큰 변수 — 카메라 보정 + APF |
| 7. 시나리오 검증 | 1일 | 6 시나리오 × 3 반복 |
| 8. 안정성 보완 | 0.5~1일 | 병행 가능 |
| 9. 시연 + 보고서 | 1~2일 | 영상 촬영 + 보고서 마무리 |
| **합계** | **8~11일** | |

---

## 리스크 & 백업 플랜

| 리스크 | 발생 시 대응 |
|--------|------------|
| HEF 양자화 mAP 폭락 | 캘리브 셋 확대 + 실주행 캡처 혼합. 안 되면 FP16 모드 시도 |
| Hailo 추론이 30 FPS 미달 | 입력 480×480로 절충, 후처리 멀티스레드 |
| 카메라 물리 보정 불가 | `center_bias_px` SW 보정만으로 진행 |
| Dynamixel 크래시 빈발 | turtlebot3_node 재기동 자동화 + USB 케이블 점검 |
| 차선 벽 오인 빈발 | LiDAR 마스크(차선 위치 기반) 추가 — `obstacle_node` 필터 강화 |
| 시연 당일 환경 변동 (조명 등) | 데이터 증강 학습 + conf_thresh 보수적 설정 |

---

## 완료 정의 (Definition of Done)

- [ ] **모델**: 640×640 YOLOv8 + HEF, mAP@50 ≥ 0.93 (양자화 후)
- [ ] **추론**: RPi5 Hailo 25 FPS 이상 안정
- [ ] **차선 추종**: 직선·곡선 100% 완주
- [ ] **장애물 회피**: 6 시나리오 평균 90% 성공
- [ ] **시연 영상**: 메인 1편 + 분리 시연 2편 이상
- [ ] **보고서**: 9장 + 부록 모두 작성, 시연 결과 채움
- [ ] **코드**: main 브랜치에 최종 커밋 + 태그(`v1.0`)
