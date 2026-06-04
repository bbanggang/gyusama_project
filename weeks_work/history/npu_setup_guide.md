# RPi5 + Hailo NPU + Docker 통합 셋업 — 한 번에 성공 가이드

> **목적**: gyusama-project에서 진행한 시행착오(npu.md 이슈 #1~#7)를 미리 우회해서
> Hailo NPU를 RPi5에 장착한 시점부터 **컨테이너 내부에서 NPU 추론이 동작하는 상태까지
> 재부팅 1회, 총 약 30~40분 안에** 도달하는 절차를 정리한 가이드.
>
> **대상 환경**
> - HW: Raspberry Pi 5 + Hailo-8 / 8L (AI Kit 또는 AI HAT+)
> - OS: Ubuntu 24.04 LTS (Noble Numbat) — **Raspberry Pi OS가 아닌 점에 주의**
> - Python: 3.12 (Ubuntu 24.04 기본)
> - HailoRT: 4.23.0 (2025-09 기준)

---

## 0. 사전 체크리스트 (5분)

### 0-1. 하드웨어
- [ ] Hailo-8 또는 Hailo-8L 보드를 RPi5의 M.2 슬롯에 물리적으로 장착
- [ ] RPi5 전원 ON, 정상 부팅 확인
- [ ] PC에서 RPi5 SSH 접근 가능 (`ssh user@rpi5_ip`)

### 0-2. 환경 정보 수집 (값 적어둘 것)
```bash
ssh user@rpi5_ip '
echo "=== 시스템 정보 ==="
uname -m                    # aarch64 여야 함
lsb_release -ds             # Ubuntu 24.04.x LTS
uname -r                    # 6.8.0-1053-raspi 같은 형식
python3 --version           # Python 3.12.x

echo "=== Hailo 칩 PCIe 인식 ==="
lspci | grep -i hailo       # "Hailo-8" 또는 "Hailo-8L" 라인 출력

echo "=== PCIe 현재 링크 속도 ==="
sudo lspci -vvv -s 0000:01:00.0 | grep LnkSta
# "Speed 5GT/s, Width x1 (downgraded)" 이면 Gen2 (Step 2에서 Gen3로 올릴 예정)
'
```

**중단 기준**
- lspci에 Hailo 없으면 → 보드 재장착 / 케이블 점검 필요
- Ubuntu 22.04 또는 다른 OS면 → 본 가이드 적용 안 됨 (패키지 호환 별도 검증)

---

## 1. Hailo 패키지 다운로드 (PC에서, 약 5분)

### 1-1. developer.hailo.ai 가입 + 로그인
- https://hailo.ai/developer-zone/software-downloads/
- 무료 계정 가입 필요

### 1-2. 필터 설정
| 필터 | 선택값 |
|------|--------|
| Software Sub-Package | **HailoRT** |
| Architecture | **ARM64** |
| OS | **Linux** |
| Python Version | **3.12** |

### 1-3. 다운로드할 3개 파일 (모두 같은 버전이어야 함)

| 파일명 (4.23.0 기준) | 역할 |
|---------------------|------|
| `hailort-pcie-driver_4.23.0_all.deb` | PCIe DKMS 드라이버 (`/dev/hailo0` 노드 생성) |
| `hailort_4.23.0_arm64.deb` | 런타임 라이브러리(`libhailort`) + `hailortcli` |
| `hailort-4.23.0-cp312-cp312-linux_aarch64.whl` | Python 바인딩 (`import hailo_platform`) |

> ⚠️ **다른 OS/Python 버전 패키지로 받지 말 것**. RPi5(aarch64) + Python 3.12 정확히 일치해야 함.
>
> ❌ AI Software Suite (Docker/Self-Extractable)는 **PC용 HEF 컴파일러**이므로 지금은 받을 필요 없음 (Phase 3에서 사용).

---

## 2. 시스템 의존성 + PCIe Gen3 (재부팅 1회, 약 5분)

> 시행착오 회피: dkms 설치와 PCIe Gen3 활성화를 **묶어서 1회 재부팅**으로 처리.

```bash
ssh user@rpi5_ip '
# 2-1. apt 업데이트
sudo apt update

# 2-2. DKMS + 커널 헤더 + 필수 의존성 설치
sudo apt install -y \
    dkms \
    linux-headers-$(uname -r) \
    build-essential

# 2-3. PCIe Gen3 활성화 (config.txt 백업 후 라인 추가)
sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.bak.$(date +%Y%m%d_%H%M%S)
sudo bash -c "
  if ! grep -q \"pciex1_gen=3\" /boot/firmware/config.txt; then
    echo \"\"  >> /boot/firmware/config.txt
    echo \"# Hailo NPU PCIe Gen3 활성화\" >> /boot/firmware/config.txt
    echo \"dtparam=pciex1_gen=3\" >> /boot/firmware/config.txt
  fi
"

# 2-4. 변경 확인 후 재부팅
tail -5 /boot/firmware/config.txt
sudo reboot
'
```

**재부팅 후 검증** (PC에서 SSH 재연결, 약 30초~1분 대기):
```bash
ssh user@rpi5_ip '
sudo lspci -vvv -s 0000:01:00.0 | grep LnkSta
# 예상 출력: "Speed 8GT/s, Width x1 (downgraded)" ← 8 GT/s 면 Gen3 활성화 성공
'
```

> ❗ 만약 Gen3로 안 올랐다면: `/boot/firmware/config.txt`에서 추가한 라인 확인, 오타 점검.
> 안정성 문제 발생 시 백업 파일로 복원 후 재부팅하면 즉시 Gen2로 롤백 가능.

---

## 3. Hailo 패키지 RPi5로 전송 + 호스트 설치 (약 5분)

### 3-1. SCP 전송 (PC에서)
```bash
# ❗ 반드시 -O 옵션 사용 (legacy 프로토콜). 미사용 시 "subsystem request failed" 오류
scp -O \
    ~/Downloads/hailort-pcie-driver_4.23.0_all.deb \
    ~/Downloads/hailort_4.23.0_arm64.deb \
    ~/Downloads/hailort-4.23.0-cp312-cp312-linux_aarch64.whl \
    user@rpi5_ip:~/Downloads/
```

### 3-2. 설치 (RPi5에서, 순서 중요)
```bash
ssh user@rpi5_ip '
# 3-2-a. PCIe 드라이버 먼저 (DKMS 빌드 + 펌웨어 자동 로드)
sudo dpkg -i ~/Downloads/hailort-pcie-driver_4.23.0_all.deb
sudo apt-get install -fy        # 의존성 자동 보정

# 검증: /dev/hailo0 생성됐는지
ls -la /dev/hailo0              # crw-rw-rw- 로 보여야 함
dkms status                     # hailo_pci/4.23.0, installed 확인

# 3-2-b. HailoRT 런타임 (libhailort + hailortcli)
sudo dpkg -i ~/Downloads/hailort_4.23.0_arm64.deb
sudo apt-get install -fy

# 3-2-c. Python 바인딩
pip3 install --break-system-packages \
    ~/Downloads/hailort-4.23.0-cp312-cp312-linux_aarch64.whl
'
```

### 3-3. 호스트 검증 (필수)
```bash
ssh user@rpi5_ip '
# 3-3-a. CLI 통신
hailortcli fw-control identify
# 예상 출력:
#   Firmware Version: 4.23.0
#   Board Name: Hailo-8                     ← 보드 명칭
#   Device Architecture: HAILO8L            ← ★ HEF 컴파일 타겟 (중요)

# 3-3-b. Python 바인딩
python3 -c "
from hailo_platform import Device
print(Device.scan())   # → ['0000:01:00.0']
"
'
```

> ## ★ 핵심 확인 사항: Board Name vs Device Architecture
> Raspberry Pi AI Kit / AI HAT+의 표준 동작:
> - `Board Name: Hailo-8` (칩 자체)
> - `Device Architecture: HAILO8L` (펌웨어 모드, 13 TOPS로 제한)
>
> **HEF(Hailo Executable) 컴파일 시 반드시 `--hw-arch hailo8l` 지정**.
> `hailo8`로 컴파일하면 이 보드에서 동작 안 함.

---

## 4. Docker 통합 (PC에서 빌드, 약 10분)

### 4-1. 빌드 컨텍스트에 Hailo 파일 배치
```bash
# 프로젝트 루트에서
mkdir -p docker/hailo
cp ~/Downloads/hailort_4.23.0_arm64.deb docker/hailo/
cp ~/Downloads/hailort-4.23.0-cp312-cp312-linux_aarch64.whl docker/hailo/
ls -la docker/hailo/
```

> ❗ **PCIe 드라이버 .deb는 컨테이너에 넣지 말 것**.
> DKMS 빌드는 호스트 커널 모듈이며, 컨테이너는 `/dev/hailo0`만 마운트해서 쓰면 됨.

### 4-2. Dockerfile 패턴 (시행착오 회피)

```dockerfile
FROM ros:jazzy-ros-base   # Ubuntu Noble + Python 3.12 (wheel과 매치)

ENV DEBIAN_FRONTEND=noninteractive

# ... (기존 ROS2 / 패키지 설치) ...

# ── HailoRT 런타임 + Python 바인딩 ─────────────────────────────────────
# 주의 1: wheel 파일명은 원본 그대로 유지 (pip이 파일명을 메타데이터로 파싱)
# 주의 2: dpkg 전에 /.dockerenv 생성 (post-install이 systemctl 호출 회피)
COPY docker/hailo/hailort_4.23.0_arm64.deb /tmp/hailort.deb
COPY docker/hailo/hailort-4.23.0-cp312-cp312-linux_aarch64.whl \
     /tmp/hailort-4.23.0-cp312-cp312-linux_aarch64.whl

RUN set -eux \
 && touch /.dockerenv \
 && apt-get update \
 && dpkg -i /tmp/hailort.deb \
 && apt-get install -fy --no-install-recommends \
 && pip3 install --no-cache-dir --break-system-packages \
      /tmp/hailort-4.23.0-cp312-cp312-linux_aarch64.whl \
 && rm -f /tmp/hailort.deb /tmp/hailort-4.23.0-cp312-cp312-linux_aarch64.whl \
 && rm -rf /var/lib/apt/lists/*
```

> **⚠️ 두 가지 함정 (둘 다 보고 시행착오로 30분씩 잃었음)**
>
> 1. `touch /.dockerenv` 빠지면: post-install이 systemctl 호출 → buildkit에서 exit 127
> 2. wheel을 `/tmp/hailort.whl`로 단순화하면: pip이 `not a valid wheel filename` 오류

### 4-3. docker-compose.yml — `/dev/hailo0` 마운트

```yaml
inference-node:
  image: bbanggang/gyusama-rpi5:latest
  network_mode: host
  ipc: host
  devices:
    - /dev/hailo0:/dev/hailo0     # ★ Hailo NPU PCIe 디바이스
  volumes:
    - ${HOME}/gyusama-project/config:/ros2_ws/config
    # ★ `~` 대신 `${HOME}` 또는 절대경로 사용 (sudo로 실행 시 `~`가 /root로 확장됨)
  environment:
    - ROS_DOMAIN_ID=1
  command: python3 /ros2_ws/models/inference_node/lane_detect_hailo.py
  restart: unless-stopped
```

### 4-4. ARM64 빌드 + Hub push (PC에서)
```bash
# buildx 멀티아키 빌더 확인 (없으면 docker buildx create --use)
docker buildx ls

# 빌드 + push
docker buildx build \
    --platform linux/arm64 \
    -t YOUR_REPO/rpi5-image:latest \
    --push \
    -f docker/Dockerfile.rpi5 \
    .
```

> 빌드 시간 약 5~10분 (QEMU 에뮬레이션 + HailoRT 의존성 + push).

---

## 5. 컨테이너 검증 (RPi5에서, 약 3분)

### 5-1. Pull + 단발성 컨테이너 검증
```bash
ssh user@rpi5_ip '
# ★ sudo 없이 docker compose 실행
# (docker 그룹에 사용자 있어야 함: id -nG | grep docker)
cd ~/your-project/docker
docker compose pull

# 5-1-a. 단발성 컨테이너로 NPU 검증
docker run --rm \
    --device /dev/hailo0:/dev/hailo0 \
    YOUR_REPO/rpi5-image:latest \
    hailortcli fw-control identify
# 호스트와 동일한 출력 (Board/Arch/Firmware) 나오면 ✅

# 5-1-b. Python 바인딩 검증
docker run --rm \
    --device /dev/hailo0:/dev/hailo0 \
    YOUR_REPO/rpi5-image:latest \
    python3 -c "
from hailo_platform import Device, VDevice
print(Device.scan())          # → ['0000:01:00.0']
with VDevice() as vd: pass    # → 정상 생성
print(\"OK\")
"
'
```

### 5-2. 정상 출력 예시
```
Executing on device: 0000:01:00.0
Identifying board
Control Protocol Version: 2
Firmware Version: 4.23.0 (release,app,extended context switch buffer)
Logger Version: 0
Board Name: Hailo-8
Device Architecture: HAILO8L
```

→ 여기까지 도달하면 **NPU 셋업 완료**. HEF 파일만 준비하면 추론 가능한 상태.

---

## 부록 A. 트러블슈팅 빠른 참조

| 증상 | 원인 | 해결 |
|------|------|------|
| `lspci`에 Hailo 없음 | 보드 미장착 / PCIe 인식 실패 | 재장착, M.2 슬롯 확인 |
| `apt install hailo-all` 실패 | Ubuntu 24.04엔 패키지 없음 | 본 가이드 .deb 수동 설치 사용 |
| `/dev/hailo0` 생성 안 됨 | DKMS 빌드 실패 (`linux-headers` 부재) | Step 2 의존성 설치 |
| dpkg 빌드 시 exit 127 | post-install이 systemctl 호출 (컨테이너) | Dockerfile에 `touch /.dockerenv` |
| `not a valid wheel filename` | wheel 파일명 단순화 | 원본 파일명 그대로 사용 |
| SCP `subsystem request failed` | sftp 서브시스템 비활성 | `scp -O` legacy 모드 |
| volume mount된 코드가 없음 | sudo 사용 시 `~` → `/root` 확장 | sudo 빼거나 `${HOME}` 사용 |
| 컨테이너에서 NPU 인식 안 됨 | devices 마운트 누락 | docker-compose `devices:` 추가 |
| HEF가 동작 안 함 | 컴파일 타겟이 hailo8 (실제는 8L) | `--hw-arch hailo8l` 재컴파일 |
| LnkSta가 5 GT/s (Gen2) | `dtparam=pciex1_gen=3` 누락 또는 재부팅 안 함 | Step 2 재진행 |

---

## 부록 B. 체크리스트 (한 번에 성공 여부)

설치 완료 후 다음 명령이 모두 성공해야 합니다.

**호스트 측 (RPi5)**
- [ ] `lspci | grep -i hailo` → Hailo 라인 출력
- [ ] `sudo lspci -vvv -s 0000:01:00.0 | grep LnkSta` → 8 GT/s
- [ ] `ls /dev/hailo0` → 디바이스 노드 존재
- [ ] `dkms status` → `hailo_pci/4.23.0 ... installed`
- [ ] `lsmod | grep hailo` → 모듈 로드 확인
- [ ] `hailortcli fw-control identify` → Firmware/Board/Arch 출력
- [ ] `python3 -c "from hailo_platform import Device; print(Device.scan())"` → 1개 발견

**컨테이너 측**
- [ ] `docker run --rm --device /dev/hailo0:/dev/hailo0 IMAGE hailortcli fw-control identify` → 호스트와 동일 출력
- [ ] `docker run --rm --device /dev/hailo0:/dev/hailo0 IMAGE python3 -c "from hailo_platform import VDevice; VDevice()"` → 에러 없음

**Phase 3 (HEF 컴파일) 사전 메모**
- [ ] **HEF 타겟은 반드시 `hailo8l`** (Board는 Hailo-8지만 Arch가 HAILO8L)
- [ ] 캘리브레이션 데이터에 합성 + 실주행 캡처 혼합 권장
- [ ] PC에서 Hailo AI Software Suite (DFC) 별도 설치 필요

---

## 부록 C. 본 가이드와 npu.md의 관계

- **`npu.md`**: 진행하면서 만난 모든 이슈를 시간순으로 기록한 **실시간 로그**
- **`npu_setup_guide.md`** (본 문서): 그 이슈들을 미리 우회한 **재현 가능한 절차서**

향후 다른 RPi5에 같은 작업을 반복할 때는 본 가이드만 따라가면 한 번에 성공 가능합니다.
시행착오의 상세 분석이나 디버깅 자료가 필요하면 `npu.md`를 참조하세요.

---

## 부록 D. 예상 총 소요 시간

| 단계 | 시간 |
|------|------|
| 0. 사전 체크 | 5분 |
| 1. PC에서 .deb 다운로드 | 5분 (네트워크 의존) |
| 2. 의존성 + Gen3 + 재부팅 | 5분 |
| 3. .deb 설치 + 호스트 검증 | 5분 |
| 4. Dockerfile 작성 + buildx + push | 10분 |
| 5. RPi5 pull + 컨테이너 검증 | 5분 |
| **합계** | **~35분** |

(참고: gyusama-project에서는 시행착오 7건으로 약 3시간 소요. 본 가이드 사용 시 약 1/5로 단축 가능)
