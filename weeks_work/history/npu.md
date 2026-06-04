# RPi5 + Hailo NPU 도입 보고서

> Hailo-8 AI HAT을 RPi5(rapi5, 192.168.0.155)에 장착한 시점부터 컨테이너 내부에서
> NPU 추론이 동작하기까지 발생한 모든 이슈/결정/조치 기록.
> 본 문서는 진행하면서 **시간순 추가**하는 살아있는 로그.
>
> **작성 시작**: 2026-06-04
> **목표 환경**
> - HW: RPi5 + Hailo-8 (AI HAT, PCIe Gen2 x1 → Gen3 x1로 확장 예정)
> - OS: Ubuntu 24.04.4 LTS (Noble Numbat), Kernel 6.8.0-1053-raspi (aarch64)
> - SW: HailoRT 런타임 + PCIe DKMS 드라이버 + Python 바인딩

---

## Phase 1-1. 설치 전 시스템 상태 점검

### 배경
사용자가 NPU(Hailo)를 물리적으로 RPi5에 장착하고 전원을 켠 상태.
드라이버·런타임 모두 미설치 상태에서 시작.

### 1-1-A. 시스템 정보 (2026-06-04 확인)
| 항목 | 값 |
|------|-----|
| hostname | rapi5 |
| OS | Ubuntu 24.04.4 LTS (Noble Numbat) |
| Kernel | 6.8.0-1053-raspi |
| 아키텍처 | aarch64 |
| 메모리 | 7.8 GiB (사용 510 MiB, 가용 7.3 GiB) |
| 디스크 | 59 GB 중 34 GB 가용 |
| 펌웨어 | 2024-09-23T13:02:56 (start_cd) |

### 1-1-B. Hailo 칩 PCIe 인식 — ✅ 성공
`lspci -vvv -s 0000:01:00.0`로 확인:
```
0000:01:00.0 Co-processor: Hailo Technologies Ltd. Hailo-8 AI Processor (rev 01)
Vendor ID: 1e60   Device ID: 2864
LnkCap:  Speed 8GT/s (Gen3), Width x4
LnkSta:  Speed 5GT/s (Gen2, downgraded), Width x1 (downgraded)
```

**확인 사실**
- 인식된 칩은 **Hailo-8 (26 TOPS)** — 계획 문서엔 "Hailo-8L (13 TOPS)"로 가정했으나 실측 칩이 더 강력함
- LnkCap이 Gen3 x4 → 그러나 LnkSta는 Gen2 x1로 동작 중
- Width x1은 RPi5 M.2 슬롯의 물리적 한계 (변경 불가)
- Speed는 RPi5 펌웨어 기본값이 Gen2 → 사용자 설정으로 Gen3 가능

**dmesg 핵심 라인**
```
brcm-pcie 1000110000.pcie: Forcing gen 2
brcm-pcie 1000110000.pcie: link up, 5.0 GT/s PCIe x1 (!SSC)
pci 0000:01:00.0: 4.000 Gb/s available PCIe bandwidth,
  limited by 5.0 GT/s PCIe x1 link at 0000:00:00.0
  (capable of 31.504 Gb/s with 8.0 GT/s PCIe x4 link)
```

→ Gen3로 올리면 약 **8 Gb/s** 사용 가능 (현재 대비 2배).

### 1-1-C. 소프트웨어 상태 — 전부 미설치 확인
| 항목 | 상태 |
|------|------|
| `/dev/hailo0` | ❌ 없음 |
| `hailortcli` 명령 | ❌ 미설치 |
| `dpkg -l | grep hailo` | ❌ 없음 |
| `dkms` 패키지 | ❌ 미설치 (드라이버 빌드용 필수) |
| `linux-headers-$(uname -r)` | ❌ 미설치 (드라이버 빌드용 필수) |
| `build-essential` | ✅ 설치됨 |
| `/etc/apt/sources.list.d/hailo*` | ❌ 없음 |
| 로컬 `*.deb` 파일 | ❌ 없음 |

---

## 이슈 #1: Ubuntu 24.04 ↔ Hailo 공식 가이드 불일치

### 발견 시점
2026-06-04, Phase 1-1 중 패키지 입수 경로 탐색 시.

### 현상
- Hailo의 공식 RPi5 설치 가이드(raspberrypi.com/documentation/computers/ai)는 **Raspberry Pi OS Trixie (64-bit)** 기준
- 권장 명령: `sudo apt install hailo-all` — 그러나 이 패키지는 `archive.raspberrypi.com` 저장소에만 존재
- 현재 시스템은 **Ubuntu 24.04 (Noble)** — Ubuntu 기본 저장소에는 `hailo-all` 없음 (`apt-cache search hailo` 빈 결과)
- GitHub `hailo-ai/hailo-rpi5-examples/install.sh`도 hailo-all이 **이미 설치된 상태를 전제** → 시스템 .deb 설치 단계 없음

### 영향
- 공식 한 줄 명령으로 끝나는 경로 사용 불가
- 다음 3가지 길 중 선택 필요

### 검토한 대안
| 옵션 | 안정성 | 소요 | 비고 |
|------|-------|------|------|
| A. developer.hailo.ai에서 .deb 직접 다운로드 | ⭐⭐⭐ | 30분 | 계정 가입 필요, 가장 표준적 |
| B. Raspberry Pi apt repo를 Ubuntu에 추가 | ⭐⭐ | 30분 | 의존성(libc/python) 충돌 가능 |
| C. HailoRT GitHub 소스 빌드 | ⭐ | 2~4시간 | CMake/의존성 복잡, 디버깅 부담 |

### 결정 — 2026-06-04
**옵션 A 채택**. 사용자가 developer.hailo.ai 계정으로 다음 3개 .deb를 다운로드:
1. `hailort-pcie-driver_*_all.deb` (DKMS PCIe 드라이버)
2. `hailort_*_arm64.deb` (HailoRT 런타임)
3. `hailort-python_*_arm64.deb` 또는 `*.whl` (Python 바인딩)

**중요**: Ubuntu 24.04용이 있다면 그대로 사용. 없으면 Debian 12 (Bookworm) arm64용 시도 → 의존성 충돌 시 옵션 C 대안.

---

## 이슈 #2: PCIe Gen2 강제 동작 — Gen3 활성화 필요

### 발견 시점
2026-06-04, dmesg / lspci 검토 시.

### 현상
- 펌웨어 기본값이 `Forcing gen 2` → Hailo-8 ↔ RPi5 통신 대역폭이 약 4 Gb/s로 제한
- Hailo-8는 Gen3 x4 (이론 31.5 Gb/s) 지원
- RPi5 M.2 슬롯 x1 제한이 있어도 Gen3 활성화 시 약 **8 Gb/s** 가능 (현재 대비 2배)

### 영향
- 추론 입력 전송이 PCIe 대역폭에 묶이면 FPS 손해
- 차선 검출처럼 작은 입력(320×320 = 약 0.4MB)은 큰 차이 없을 수 있으나, 640×640 RGB(약 1.2MB) 입력 + 다중 모델 시 격차 발생

### 대응
공식 라즈베리파이 문서가 제시하는 두 가지 방법:
1. **config.txt 직접 편집** (`/boot/firmware/config.txt`에 `dtparam=pciex1_gen=3` 추가)
2. **raspi-config** → Advanced Options → PCIe Speed (Ubuntu에선 raspi-config 미설치 가능성 큼)

### 결정 — 2026-06-04
**방법 1 채택**. Hailo 설치와 병행 진행. 안정성 문제 발생 시 해당 라인 제거 후 재부팅으로 즉시 롤백 가능.

---

## 이슈 #3: dkms / linux-headers 부재

### 현상
- 드라이버는 DKMS로 커널마다 자동 빌드되어야 함
- 현재 `dkms` 패키지 자체와 `linux-headers-6.8.0-1053-raspi`가 모두 미설치

### 영향
- `hailort-pcie-driver_*.deb` 설치 시 빌드 실패 → /dev/hailo0 생성 안 됨

### 대응 (예정)
```bash
sudo apt install -y dkms linux-headers-$(uname -r)
```
- 패키지 설치 가능 여부는 다음 단계에서 확인
- linux-headers가 Ubuntu raspi 커널 저장소에 있어야 함

---

---

## 이슈 #4: 칩(Hailo-8) ↔ 동작 모드(HAILO8L) 불일치 — **컴파일 타겟 결정적**

### 발견 시점
2026-06-04, `hailortcli fw-control identify` 실행 시.

### 현상
```
Board Name:         Hailo-8
Device Architecture: HAILO8L      ← 동작 모드
Firmware Version:    4.23.0 (release,app,extended context switch buffer)
```
- PCIe lspci 인식: "Hailo-8 AI Processor (rev 01)"
- 그러나 펌웨어 측 Device Architecture는 **HAILO8L (13 TOPS)**

### 원인 (추정)
- Raspberry Pi AI Kit의 표준 구성: Hailo-8 실리콘이지만 펌웨어 단에서 8L 모드로 fuse
- 또는 Hailo-8L 보드인데 lspci device ID가 Hailo-8 시리즈 공통값(1e60:2864)으로 표시되는 것

### 영향 (매우 중요)
- **HEF(Hailo Executable Format) 컴파일 시 타겟 디바이스 결정에 직결**
  - `--hw-arch hailo8` 으로 컴파일 → 8L 보드에서 동작 안 함 (또는 성능 손실)
  - `--hw-arch hailo8l` 으로 컴파일 → 이 보드에서 정상 동작
- 향후 Phase 3 (ONNX → HEF 변환) 시 **반드시 `hailo8l` 사용**

### 검증
- 13 TOPS도 YOLOv8s 640×640 추론에 충분 (예상 25~40 FPS)

---

## 진행 로그 (시간순)

| 시각 | 단계 | 결과 |
|------|------|------|
| 2026-06-04 10:25 | Phase 1-1 — SSH 연결 + 시스템 정보 수집 | ✅ Hailo-8 PCIe 인식 |
| 2026-06-04 10:30 | 이슈 #1 식별 (Ubuntu vs Raspberry Pi OS) | 옵션 A 결정 (.deb 수동) |
| 2026-06-04 10:30 | 이슈 #2 식별 (PCIe Gen2 강제 동작) | Gen3 활성화 진행 결정 |
| 2026-06-04 10:30 | 이슈 #3 식별 (dkms/headers 부재) | apt install 예정 |
| 2026-06-04 10:36 | 사용자 .deb 3개 PC → RPi5 SCP (legacy -O 모드) | ✅ ~/Downloads/ 도착 |
| 2026-06-04 10:38 | `dkms` + `linux-headers-6.8.0-1053-raspi` 설치 | ✅ 빌드 환경 완성 |
| 2026-06-04 10:38 | config.txt에 `dtparam=pciex1_gen=3` 추가 + 백업 | ✅ 백업 파일 보존 |
| 2026-06-04 10:39 | RPi5 재부팅 | ✅ 약 25초 만에 SSH 복귀 |
| 2026-06-04 10:40 | **PCIe Gen3 활성화 검증** — `LnkSta: 8GT/s, Width x1` | ✅ 5GT/s → 8GT/s |
| 2026-06-04 10:41 | `hailort-pcie-driver_4.23.0_all.deb` 설치 | ✅ DKMS 빌드 + 모듈 로드 |
| 2026-06-04 10:41 | 펌웨어 자동 로드 (`hailo8_fw.bin` 150ms) | ✅ `/dev/hailo0` 생성 |
| 2026-06-04 10:42 | `hailort_4.23.0_arm64.deb` (런타임 + CLI) 설치 | ✅ hailortcli 4.23.0 |
| 2026-06-04 10:42 | **`hailortcli fw-control identify`** 실행 | ✅ 통신 성공 |
| 2026-06-04 10:42 | **이슈 #4 발견** (Board=Hailo-8, Arch=HAILO8L) | ⚠️ HEF 타겟은 hailo8l |
| 2026-06-04 10:43 | Python whl 설치 (`hailort-4.23.0-cp312...`) | ✅ import OK |
| 2026-06-04 10:43 | `Device.scan()` Python 검증 | ✅ 1개 디바이스 발견 |

---

## 누적 검증 결과 (호스트 측)

| 항목 | 상태 |
|------|------|
| `/dev/hailo0` | ✅ 생성됨, 권한 `crw-rw-rw-` (컨테이너 마운트 시 sudo 불필요) |
| `lsmod | grep hailo` | ✅ `hailo_pci 122880` |
| `dkms status` | ✅ `hailo_pci/4.23.0, 6.8.0-1053-raspi, aarch64: installed` |
| `hailortcli --version` | ✅ `HailoRT-CLI version 4.23.0` |
| `hailortcli fw-control identify` | ✅ 정상 통신, Arch=HAILO8L 확인 |
| `hailortcli scan` | ✅ `Device: 0000:01:00.0` |
| `python3 -c "import hailo_platform"` | ✅ OK, version 4.23.0 |
| `Device.scan()` (Python) | ✅ 1개 디바이스 |
| PCIe 링크 속도 | ✅ Gen3 (8 GT/s), Width x1 |

---

## Phase 1-4. Dockerfile 통합 (완료)

### 설계 원칙
- **PCIe 드라이버는 호스트에만 설치** (DKMS, 커널 모듈) — 컨테이너에는 설치 안 함
- 컨테이너는 **/dev/hailo0만 마운트** 받고 사용자 공간 라이브러리(libhailort) + Python 바인딩만 내장
- 베이스 이미지가 `ros:jazzy-ros-base` (Ubuntu 24.04 Noble + Python 3.12) → cp312 wheel 정확히 매치 ✅

### 변경 파일
**`docker/hailo/`** 디렉토리 신설 — 빌드 컨텍스트용
- `hailort_4.23.0_arm64.deb` (6.6 MB)
- `hailort-4.23.0-cp312-cp312-linux_aarch64.whl` (9.5 MB)

**`docker/Dockerfile.rpi5`** — HailoRT 설치 레이어 추가
```dockerfile
# HailoRT 런타임 + Python 바인딩
COPY docker/hailo/hailort_4.23.0_arm64.deb /tmp/hailort.deb
COPY docker/hailo/hailort-4.23.0-cp312-cp312-linux_aarch64.whl /tmp/hailort.whl
RUN set -eux \
 && apt-get update \
 && (dpkg -i /tmp/hailort.deb || true) \
 && apt-get install -fy --no-install-recommends \
 && pip3 install --no-cache-dir --break-system-packages /tmp/hailort.whl \
 && rm -f /tmp/hailort.deb /tmp/hailort.whl \
 && rm -rf /var/lib/apt/lists/*
```
> `dpkg -i || true` + `apt-get install -fy`: 의존성 부족으로 dpkg가 실패해도 apt가 자동 보정

**`docker/docker-compose.yml`** — inference-node에 디바이스 마운트
```yaml
inference-node:
  ...
  devices:
    - /dev/hailo0:/dev/hailo0   # Hailo-8 NPU PCIe 디바이스
```

### 진행 상태
- ✅ Hailo 파일 빌드 컨텍스트(`docker/hailo/`) 배치
- ✅ Dockerfile.rpi5 HailoRT 레이어 추가
- ✅ docker-compose.yml `/dev/hailo0` devices 마운트 추가
- ⚠️ 1차 buildx 실패 → 이슈 #5 발견 (아래)
- 🔄 2차 buildx 진행 중 (touch /.dockerenv 우회 적용)

---

## 이슈 #5: hailort.deb post-install이 buildkit/QEMU 빌드에서 실패

### 발견 시점
2026-06-04, 첫 buildx 빌드 시.

### 현상
```
Failed. Exited with status 127. See /var/log/hailort.deb.log
dpkg: error processing package hailort (--configure):
 installed hailort package post-installation script subprocess returned error exit status 127
```

### 원인 분석
hailort.postinst 스크립트:
```bash
function activate_hailort_service_if_required(){
    if [ ! -f /.dockerenv ]; then        # ← Docker 감지 체크
        systemctl daemon-reload
        # ... systemctl restart hailort.service 등
    fi
}
```
- 일반 Docker 컨테이너는 `/.dockerenv` 자동 생성됨 → 분기 진입 안 함 → 안전
- 그러나 **buildkit + QEMU 에뮬레이션 빌드 환경**에서는 `/.dockerenv` 가 자동 생성되지 않음
- `systemctl daemon-reload`가 PID 1과 통신 못 함 → exit 127
- `set -eEuo pipefail` 때문에 즉시 종료 → 전체 dpkg 실패

### 해결책
빌드 시점에 `/.dockerenv` 를 명시적으로 생성:
```dockerfile
RUN set -eux \
 && touch /.dockerenv \           # ← 빌드 환경에서 Docker 감지 신호 강제
 && dpkg -i /tmp/hailort.deb \
 && ...
```
- `/.dockerenv`는 빈 파일로 존재하기만 하면 됨
- 런타임 컨테이너는 Docker가 자동으로 다시 마운트하므로 충돌 없음
- 다른 패키지에는 영향 없음 (이 한 파일만 본다)

### 교훈
- Docker 환경 감지는 보통 `/.dockerenv` 또는 `cgroup` 확인으로 함
- buildkit 빌드 환경은 **표준 Docker 컨테이너와 다름** → 일부 패키지의 환경 감지 우회 필요
- 비슷한 문제는 nvidia-driver, hailo 등 systemd 서비스 등록하는 패키지에서 자주 발생

---

## 이슈 #6: pip이 wheel 파일명을 메타데이터로 파싱 → 단순화 시 거부

### 발견 시점
2026-06-04, 2차 buildx 빌드 시 (이슈 #5 해결 직후).

### 현상
```
ERROR: hailort.whl is not a valid wheel filename.
```
Dockerfile에서 `COPY ... /tmp/hailort.whl`로 단순 이름으로 복사한 게 원인.

### 원인
- PEP 427 (wheel format)에 따르면 wheel 파일명 자체가 메타데이터:
  `{distribution}-{version}(-{build})?-{python}-{abi}-{platform}.whl`
- pip은 설치 전 파일명을 파싱해 호환성을 확인
- `hailort.whl` 처럼 dash 없는 이름은 파싱 실패 → 거부

### 해결책
원본 파일명을 그대로 유지:
```dockerfile
COPY docker/hailo/hailort-4.23.0-cp312-cp312-linux_aarch64.whl \
     /tmp/hailort-4.23.0-cp312-cp312-linux_aarch64.whl
RUN pip3 install /tmp/hailort-4.23.0-cp312-cp312-linux_aarch64.whl
```

### 교훈
- **.deb 파일은 dpkg가 파일명을 신경 안 씀** → 단순화 가능 (`hailort.deb` OK)
- **.whl 파일은 pip이 파일명을 메타데이터로 사용** → 원본 그대로 유지 필수
- 일반적인 함정: COPY 시 짧은 이름으로 줄이고 싶을 때 발생

---

---

## Phase 1-5. 컨테이너 NPU 검증 (완료)

### 절차 및 결과
1. ✅ RPi5에서 `docker compose pull` — 새 이미지(digest `0adcbaf4...`) 받기
2. ⚠️ `docker compose up -d inference-node` — 컨테이너가 crash loop (이슈 #7, NPU와 무관)
3. ✅ **단발성 컨테이너로 검증 우회**:
   ```bash
   docker run --rm --device /dev/hailo0:/dev/hailo0 \
     bbanggang/gyusama-rpi5:latest hailortcli fw-control identify
   ```
   결과:
   ```
   Board Name:           Hailo-8
   Device Architecture:  HAILO8L
   Firmware Version:     4.23.0
   ```
4. ✅ **Python 바인딩 검증**:
   ```bash
   docker run --rm --device /dev/hailo0:/dev/hailo0 \
     bbanggang/gyusama-rpi5:latest python3 -c "
   from hailo_platform import Device, VDevice
   print(Device.scan())     # → ['0000:01:00.0']
   with VDevice() as vd: pass    # → 정상 생성"
   ```

### 검증 종합

| 항목 | 결과 |
|------|------|
| 컨테이너 내 `hailortcli fw-control identify` | ✅ 성공 |
| 컨테이너 내 `python3 -c "from hailo_platform import Device"` | ✅ OK |
| 컨테이너 내 `Device.scan()` | ✅ 1개 발견 |
| 컨테이너 내 `VDevice()` 객체 생성 | ✅ 성공 (실제 NPU 점유 가능) |
| `/dev/hailo0` 디바이스 마운트 | ✅ docker-compose devices: 정상 |
| 호스트 ↔ 컨테이너 펌웨어 버전 일치 | ✅ 4.23.0 |
| HEF 컴파일 타겟 (HAILO8L 확정) | ✅ Phase 3에서 `--hw-arch hailo8l` 사용 |

→ **Phase 1 (NPU 셋업) 완전 종료**.

---

## 이슈 #7: `sudo docker compose` 사용 시 volume mount `~` 경로 잘못 확장 (NPU와 무관)

### 발견 시점
2026-06-04, Phase 1-5 검증 중 inference-node crash loop 진단.

### 현상
- `docker logs inference-node`:
  ```
  python3: can't open file '/ros2_ws/models/inference_node/lane_detect_ncnn.py':
  [Errno 2] No such file or directory
  ```
- 그러나 호스트 `~/gyusama-project/models/inference_node/lane_detect_ncnn.py` (25KB)는 존재

### 원인
- docker-compose.yml의 volume 라인:
  ```yaml
  volumes:
    - ~/gyusama-project/models/inference_node:/ros2_ws/models/inference_node
  ```
- 사용자가 `sudo docker compose ...` 로 실행 → docker compose의 `~`가 **root의 home인 `/root`로 확장**
- 실제 마운트 경로: `/root/gyusama-project/models/inference_node` → 존재 안 함 → 빈 디렉토리 마운트
- 결과: 컨테이너 안에서 코드 파일 없음 → 매번 즉시 크래시 → restart 정책으로 무한 재시작

### 영향
- NPU 통합과는 별개. 단발성 컨테이너 검증으로 Phase 1-5는 완료.
- 그러나 Phase 4 (lane_detect_hailo.py 배포 후 inference-node 정상 기동) 전에 반드시 해결 필요

### 해결책 (Phase 4 진행 전 적용 예정)
- **방법 A (권장)**: `sudo` 없이 `docker compose` 실행
  - rapi5 사용자는 이미 docker 그룹에 속함 (`groups` 출력에 docker 있음)
  - `~`가 정상적으로 `/home/rapi5`로 확장
- **방법 B**: docker-compose.yml의 `~` → `/home/rapi5` 절대경로로 변경
  - 단점: 다른 머신/사용자명에 종속

### 교훈
- compose 파일의 `~` 사용은 실행자 home 의존 → `sudo` 또는 root cron 등에서 다른 home으로 확장
- 시연 안정성을 위해 절대 경로 또는 `${HOME}` 환경변수가 더 안전

---

## Phase 1 종합 정리

| Phase | 결과 |
|-------|------|
| 1-1. 시스템 정보 수집 | ✅ Hailo-8 PCIe 인식 (1e60:2864) |
| 1-2. PCIe 드라이버 설치 | ✅ DKMS 빌드, `/dev/hailo0` 생성, 펌웨어 로드 150ms |
| 1-3. HailoRT 런타임 + Python 설치 | ✅ hailortcli 4.23.0, `hailo_platform` import OK |
| 1-4. Dockerfile 통합 | ✅ 3차 빌드 끝에 성공 (이슈 #5, #6 해결) |
| 1-5. 컨테이너 검증 | ✅ NPU 인식, VDevice 생성 성공 |

**식별된 이슈 (총 7건)**
- #1 Ubuntu 24.04 ↔ Hailo 공식 가이드 불일치 (해결)
- #2 PCIe Gen2 강제 동작 (Gen3 활성화로 해결)
- #3 dkms / linux-headers 부재 (apt install로 해결)
- #4 Board=Hailo-8 / Arch=HAILO8L (HEF 컴파일 타겟 결정적)
- #5 hailort.deb post-install systemctl 실패 (`touch /.dockerenv` 우회)
- #6 wheel 파일명 단순화 시 pip 거부 (원본 파일명 유지)
- #7 sudo docker compose volume `~` 경로 (Phase 4 전 해결 예정)

---

## 다음 단계 — Phase 2: 640×640 모델 재학습

- Phase 1과 별개로 PC에서 진행 가능
- `python3 models/training/train_yolo_lane.py --epochs 150`
- 디폴트: yolov8s / imgsz=640 / batch=16
- 약 3~5시간 소요 예상 (RTX 5070 Ti)
- 완료 후 Phase 3 (ONNX → HEF 변환, `--hw-arch hailo8l` 필수) 진입
