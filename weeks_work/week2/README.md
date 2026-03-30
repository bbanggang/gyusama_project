# 2주차 진행 보고 — Multi-arch 빌드 파이프라인 구축

> **기간**: 2026.03.11 ~ 2026.03.17
> **실제 작업일**: 2026.03.30
> **목표**: PC(x86_64)에서 빌드한 이미지가 RPi5(ARM64)에서도 실행되도록 멀티 아키텍처 빌드 환경을 구성한다.

---

## 1. 진행한 작업 목록

### 2-1. QEMU 등록 (ARM64 에뮬레이션)
PC의 CPU는 x86_64이므로 ARM64 바이너리를 실행하려면 QEMU 에뮬레이터가 필요하다.
`multiarch/qemu-user-static` 이미지를 통해 시스템에 QEMU binfmt를 등록했다.

```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

등록된 에뮬레이터 중 이 프로젝트에서 핵심:
- `qemu-aarch64-static` → ARM64 (RPi5)
- `qemu-arm-static` → ARM32

---

### 2-2. gyusama-builder 전용 Buildx 빌더 생성
기본 `default` 빌더는 QEMU를 활용하지 못한다.
`docker-container` 드라이버를 사용하는 전용 빌더를 생성해야 멀티아키텍처 빌드가 가능하다.

```bash
docker buildx create --name gyusama-builder --driver docker-container --use
docker buildx inspect --bootstrap
```

빌더 생성 후 지원 플랫폼:
```
linux/amd64, linux/arm64, linux/arm/v7, linux/arm/v6, linux/riscv64 ...
```

---

### 2-3. RPi5용 Dockerfile.rpi5 작성

**작성 파일**: [docker/Dockerfile.rpi5](../../docker/Dockerfile.rpi5)

**PC용(Dockerfile.pc)과의 차이점**:

| 항목 | Dockerfile.pc (x86_64) | Dockerfile.rpi5 (ARM64) |
|------|------------------------|--------------------------|
| 베이스 이미지 | `nvidia/cuda:12.6.0-base-ubuntu24.04` | `ros:jazzy-ros-base` |
| GPU 지원 | O (CUDA) | X (CPU 전용) |
| ROS2 패키지 | `ros-jazzy-desktop` (풀 설치) | `ros-jazzy-ros-base` (경량) |
| ONNX Runtime | 미설치 (GPU 추론) | `onnxruntime` CPU 버전 설치 |
| 용도 | 개발 / 시뮬레이션 | 엣지 추론 / 실제 로봇 제어 |

**베이스 이미지를 `ros:jazzy-ros-base`로 선택한 이유**:
QEMU 에뮬레이션으로 ARM64에서 ROS2를 직접 apt 설치하면
`py3compile` BrokenPipeError 버그가 발생해 설치 전체가 실패한다.
공식 ROS2 멀티아키텍처 이미지는 네이티브 환경에서 사전 빌드되어 있으므로
이 문제가 없다.

---

### 2-4. 멀티아키텍처 동시 빌드 검증

```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f Dockerfile.rpi5 \
  -t bbanggang/turtlebot3-ros2:v1.0 \
  --load=false \
  .
```

- `--load=false`: 빌드 결과를 로컬 Docker에 로드하지 않고 빌드 캐시에만 보관
- 실제 push는 3주차에 진행

빌드 소요 시간:
- amd64: 약 2분 (TurtleBot3 패키지 + onnxruntime)
- arm64: 약 18분 (QEMU 에뮬레이션으로 onnxruntime pip 설치가 느림)

---

## 2. 완료 기준 확인

### 확인 1 — multi-platform 빌더 확인
```bash
docker buildx ls
```
**결과**:
```
NAME/NODE           DRIVER/ENDPOINT                 STATUS   PLATFORMS
gyusama-builder*    docker-container
 \_ gyusama-builder0  \_ unix:///var/run/docker.sock  running  linux/amd64, linux/arm64, ...
```
`gyusama-builder`가 현재 활성 빌더(`*`)로 amd64 + arm64 지원 확인 ✅

### 확인 2 — x86_64 + ARM64 동시 빌드 성공
```
[+] Building 1082.2s (17/17) FINISHED
```
amd64, arm64 양쪽 모두 FINISHED 출력 ✅

### 확인 3 — RPi5에서 이미지 실행 확인
RPi5가 아직 네트워크에 미연결 상태이므로 **3주차에 진행** 예정.

---

## 3. 주의사항

### QEMU 에뮬레이션의 한계: py3compile BrokenPipeError
- QEMU로 ARM64를 에뮬레이션하는 환경에서 `py3compile`이 BrokenPipeError를 발생시키는 알려진 버그가 있다.
- 영향받는 패키지: `python3-colcon-bash`, `python3-flake8-*`, 그로 인한 `ros-jazzy-*` 전체.
- **해결책**: ARM64 베이스 이미지로 공식 ROS2 멀티아키텍처 이미지(`ros:jazzy-ros-base`)를 사용할 것.
- 직접 `ubuntu:24.04`를 베이스로 ROS2를 QEMU 에뮬레이션 빌드하는 방식은 사용하지 말 것.

### --no-install-recommends 사용
- `apt-get install`에 `--no-install-recommends`를 붙이지 않으면 `cppcheck`, `flake8` 등 개발용 도구가 딸려온다.
- RPi5 이미지에서는 불필요한 패키지로 이미지 용량만 늘어나므로 반드시 붙일 것.

### QEMU는 재부팅 후 초기화됨
- `docker run --rm --privileged multiarch/qemu-user-static --reset -p yes` 로 등록한 QEMU binfmt는 **재부팅 후 사라진다**.
- 재부팅 후 ARM64 빌드가 필요하면 위 명령어를 다시 실행해야 한다.
- 영구 등록이 필요하다면 `binfmt-support` 패키지를 설치해 systemd 서비스로 관리할 수 있다.

### onnxruntime ARM64 pip 빌드 시간
- ARM64용 `onnxruntime`은 pip wheel이 제공되지만 QEMU 에뮬레이션 환경에서는 매우 느리다 (약 11분).
- 추후 Dockerfile.rpi5 수정 시 이 레이어 위에서만 변경하면 캐시 덕분에 재빌드 불필요.

---

## 4. 다음 주차 준비 사항 (3주차)

3주차에서는 빌드된 이미지를 Docker Hub에 push하고,
docker-compose로 여러 ROS2 노드를 한 번에 배포하는 구조를 만든다.

**필요 작업**:
- `docker login` 으로 `bbanggang` 계정 로그인
- `docker buildx build --push` 로 멀티아키텍처 이미지 Docker Hub 업로드
- `docker-compose.yml` 작성 (ros2-core, camera-node, inference-node, control-node)
- RPi5 네트워크 연결 및 `docker pull` 테스트

**RPi5 연결 준비**:
- 공유기 192.168.0.x 대역에 RPi5 연결 필요
- 현재 PC IP: 192.168.0.11
