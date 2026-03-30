# 1주차 진행 보고 — PC 개발 환경 최적화 및 Docker 기초

> **목표**: Ubuntu 24.04에서 GPU 가속 Docker 환경을 구축하고, ROS2가 포함된 팀 전용 베이스 이미지를 만든다.

---

## 1. 진행한 작업 목록

### 1-1. 시스템 사전 점검
프로젝트 시작 전 전체 개발 환경 상태를 점검했다.

| 항목 | 결과 |
|------|------|
| Ubuntu 24.04.4 LTS | 정상 확인 |
| RTX 5070 Ti — Driver 590.48 / CUDA 13.1 | 정상 확인 |
| Docker 29.3.1 + NVIDIA Runtime | 정상 확인 |
| NVIDIA Container Toolkit 1.19.0 | 정상 확인 |
| Isaac Sim 5.1.0 (`~/isaac_env`) | 정상 확인 |
| PyTorch 2.7.0+cu128 / ONNX 1.20.1 | 정상 확인 |
| RAM 62GB / 디스크 여유 126GB | 충분 |
| ROS2 Jazzy | **미설치 → 이번 주 설치** |

---

### 1-2. ROS2 Jazzy 설치 (호스트)
컨테이너 내부뿐 아니라 호스트에서도 `ros2` 명령어를 직접 사용하기 위해 설치했다.

```bash
# ROS2 apt 저장소 등록
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 설치
sudo apt-get update && sudo apt-get install -y ros-jazzy-desktop
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete \
  ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-msgs ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# 환경 변수 등록 (~/.bashrc)
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

---

### 1-3. 프로젝트 디렉토리 구조 생성
README에 정의된 구조대로 폴더를 생성했다.

```bash
mkdir -p ~/gyusama-project/{docker,src/{lane_detection,inference_node,nav2_config,turtlebot3_control},isaac_sim,models,data/{synthetic,real,calibration},configs}
```

생성된 구조:
```
gyusama-project/
├── docker/
├── src/
│   ├── lane_detection/
│   ├── inference_node/
│   ├── nav2_config/
│   └── turtlebot3_control/
├── isaac_sim/
├── models/
├── data/
│   ├── synthetic/
│   ├── real/
│   └── calibration/
└── configs/
```

---

### 1-4. ROS2 베이스 Dockerfile 작성 및 빌드

**작성 파일**:
- [docker/Dockerfile.pc](../../docker/Dockerfile.pc) — PC용 (x86_64)
- [docker/entrypoint.sh](../../docker/entrypoint.sh) — ROS2 환경 자동 소싱 스크립트

**Dockerfile.pc 구성 요소**:
- 베이스: `nvidia/cuda:12.6.0-base-ubuntu24.04`
- ROS2 Jazzy Desktop + TurtleBot3 + Nav2 패키지 포함
- `ROS_DOMAIN_ID=1`, `TURTLEBOT3_MODEL=burger` 환경 변수 내장
- `entrypoint.sh`로 컨테이너 시작 시 ROS2 환경 자동 소싱

**빌드 명령어**:
```bash
cd ~/gyusama-project/docker
docker build -f Dockerfile.pc -t bbanggang/ros2-base:latest .
```

---

## 2. 완료 기준 확인

### 확인 1 — 컨테이너 내 GPU 인식
```bash
docker run --rm --gpus all nvidia/cuda:12.6.0-base-ubuntu24.04 nvidia-smi
```
**결과**: RTX 5070 Ti 정상 인식, Driver 590.48 / CUDA 13.1 확인 ✅

### 확인 2 — Dockerfile 빌드 성공
```bash
docker build -f Dockerfile.pc -t bbanggang/ros2-base:latest .
```
**결과**: `FINISHED` 출력, 이미지 `bbanggang/ros2-base:latest` 생성 ✅

### 확인 3 — 컨테이너 내 ros2 topic list 정상 동작
```bash
docker run --rm --gpus all bbanggang/ros2-base:latest ros2 topic list
```
**결과**:
```
/parameter_events
/rosout
```
두 기본 토픽 정상 출력 ✅

---

## 3. 주의사항

### ROS_DOMAIN_ID는 반드시 통일
- PC 호스트, Docker 컨테이너, RPi5 모두 `ROS_DOMAIN_ID=1`로 설정해야 한다.
- 값이 다르면 같은 네트워크에 있어도 토픽이 보이지 않는다.
- 추후 RPi5 설정 시 반드시 동일하게 맞출 것.

### PYTHONPATH 환경변수 주의
- Dockerfile에서 `ENV PYTHONPATH=...:$PYTHONPATH` 형식은 경고를 발생시킨다.
  - 이유: 빌드 시점에 `$PYTHONPATH`가 정의되어 있지 않아 undefined variable 취급됨
- 올바른 방식:
  ```dockerfile
  # 잘못된 방식 (경고 발생)
  ENV PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH

  # 올바른 방식
  ENV PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages
  ```
- 추후 경로를 추가해야 한다면 `entrypoint.sh`에서 `export PYTHONPATH=추가경로:$PYTHONPATH` 방식으로 붙이는 것이 안전하다.

### docker build 캐시 활용
- ROS2 설치 레이어는 빌드 시간이 매우 길다 (약 22분).
- Dockerfile 수정 시 변경한 레이어 **아래부터** 전부 재빌드되므로, 자주 바뀌는 내용은 Dockerfile 하단에 배치해야 빌드 시간을 줄일 수 있다.
- 현재 구조는 기본 패키지 → ROS2 저장소 등록 → ROS2 설치 → 작업 디렉토리 → entrypoint 순으로 올바르게 배치되어 있다.

### Docker 이미지 용량 주의
- `bbanggang/ros2-base:latest`는 ROS2 Desktop + Nav2 포함으로 용량이 크다.
- Docker Hub push 전 `docker images` 명령어로 용량을 확인할 것.
- 불필요한 패키지를 줄이고 싶다면 `ros-jazzy-desktop` 대신 `ros-jazzy-ros-base`를 베이스로 필요한 패키지만 추가하는 방식도 고려할 수 있다.

---

## 4. 다음 주차 준비 사항 (2주차)

2주차에서는 Docker Buildx로 ARM64(RPi5용) 이미지를 빌드하기 위해
QEMU 에뮬레이터 등록과 멀티아키텍처 전용 빌더 생성을 진행한다.

**현재 Buildx 상태**:
- `gyusama-builder` 미생성
- QEMU binfmt 미등록 → ARM64 빌드 불가 상태

**RPi5 연결**:
- 2~3주차 중 공유기(192.168.0.x 대역)에 RPi5 연결 필요
- 현재 PC IP: 192.168.0.11
