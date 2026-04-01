# 3주차 진행 보고 — 원격 배포 및 컨테이너 오케스트레이션

> **목표**: Docker Hub를 통해 이미지를 관리하고, docker-compose로 여러 ROS2 노드를 한 번에 배포하는 체계를 수립한다.

---

## 1. 진행한 작업 목록

### 3-1. Docker Hub 멀티아키텍처 이미지 Push

2주차에서 빌드한 `Dockerfile.rpi5`를 기반으로 x86_64 + ARM64 멀티아키텍처 이미지를 Docker Hub에 push했다.

```bash
# QEMU 재등록 (재부팅 후 초기화되므로)
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# 빌더 재생성 (ARM64 지원 확인)
docker buildx rm gyusama-builder
docker buildx create --name gyusama-builder --use
docker buildx inspect --bootstrap

# Docker Hub 로그인
docker login -u bbanggang

# 멀티아키텍처 빌드 + push
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f docker/Dockerfile.rpi5 \
  -t bbanggang/turtlebot3-ros2:v1.0 \
  --push \
  docker/
```

**결과**: Docker Hub에 단일 이미지 태그(`v1.0`)로 amd64 + arm64 manifest list가 등록됨
![docker hub image](https://github.com/user-attachments/assets/46de69c0-5c16-4fdb-a1b4-d10d0463a38e)
- RPi5에서 pull 시 ARM64 이미지 자동 선택
- PC에서 pull 시 amd64 이미지 자동 선택

---

### 3-2. 새 RPi5 보드 환경 구성

기존 RPi5는 네이티브 ROS2 등 여러 패키지가 설치된 상태였으므로, 새 보드에 Ubuntu를 클린 설치하고 Docker를 세팅했다.

**Docker 설치 (RPi5 / ARM64 / Ubuntu)**:
```bash
sudo apt-get update && \
sudo apt-get install -y ca-certificates curl gnupg && \
sudo install -m 0755 -d /etc/apt/keyrings && \
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg && \
sudo chmod a+r /etc/apt/keyrings/docker.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null && \
sudo apt-get update && \
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin && \
sudo usermod -aG docker $USER && \
newgrp docker
```

설치 확인:
```
Docker version 29.3.1, build c2be9cc
Docker Compose version v5.1.1
```

---

### 3-3. RPi5에서 이미지 Pull

```bash
docker pull bbanggang/turtlebot3-ros2:v1.0
```

- Digest: `sha256:2453c495a319a3d5489d4d646b39819298df2225f2c1e0a04c8d9a82fe207d09`
- ARM64 manifest가 자동으로 선택되어 pull됨

---

### 3-4. docker-compose 서비스 통합

**작성 파일**: [docker/docker-compose.yml](../../docker/docker-compose.yml)

여러 ROS2 노드를 단일 YAML 파일로 정의하여 한 번에 기동하는 체계를 구성했다.

```yaml
services:
  ros2-core:      # ROS2 데몬 + 기본 통신 기반
  camera-node:    # IMX219 카메라 드라이버 (CSI)
  inference-node: # ONNX Runtime INT8 추론
  control-node:   # Dynamixel 모터 제어 (ros2_control)
```

**주요 설계 결정**:

| 설정 | 값 | 이유 |
|------|-----|------|
| `network_mode: host` | host 네트워크 | FastDDS 멀티캐스트가 bridge 네트워크에서 동작하지 않음 |
| `privileged: true` | control-node, camera-node | `/dev/ttyUSB0`, `/dev/video0` 디바이스 접근 필요 |
| `ROS_DOMAIN_ID=1` | 전 노드 공통 | PC와 RPi5가 같은 도메인에서 통신하도록 통일 |
| `LDS_MODEL=LDS-02` | control-node | `robot.launch.py`가 LiDAR 모델을 환경변수로 필수 요구 |

> **주의**: 하드웨어(Dynamixel, LiDAR, 카메라)가 물리적으로 연결되지 않은 상태에서는
> `robot.launch.py` 대신 `sleep infinity`로 컨테이너를 유지하며 통신 테스트만 수행.
> 실제 하드웨어 연결은 TurtleBot3 완전 조립 후 진행.

---

### 3-5. PC ↔ RPi5 ROS2 토픽 통신 확인

`docker compose up` 으로 컨테이너 기동 후 네트워크를 통한 ROS2 토픽 통신을 검증했다.

**RPi5에서 토픽 발행**:
```bash
docker exec -it ros2-core bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   ros2 topic pub /test_topic std_msgs/String 'data: hello_from_rpi5' --rate 1"
```

**PC에서 토픽 수신**:
```bash
export ROS_DOMAIN_ID=1
ros2 topic echo /test_topic
```

**결과**:
![communication image](https://github.com/user-attachments/assets/6b1674ba-6bf5-4d3f-93b6-2ff46bb5c779)

FastDDS를 통해 같은 `ROS_DOMAIN_ID=1` 도메인 내에서 PC와 RPi5 간 토픽 통신 정상 동작 확인.

---

## 2. 완료 기준 확인

- [x] Docker Hub에 멀티아키텍처 이미지 push 성공 (`bbanggang/turtlebot3-ros2:v1.0`)
- [x] RPi5에서 `docker pull` → `docker compose up` 으로 ROS2 노드 실행
- [x] PC ↔ RPi5 간 ROS2 토픽 통신 확인

---

## 3. 주의사항

### network_mode: host 필수
- FastDDS는 UDP 멀티캐스트를 사용한다.
- Docker의 기본 bridge 네트워크는 멀티캐스트를 차단하므로 PC ↔ RPi5 간 ROS2 통신이 되지 않는다.
- 컨테이너에 `network_mode: host`를 설정해야 호스트 네트워크를 직접 사용하여 멀티캐스트가 통과한다.

### LDS_MODEL 환경변수 필수
- `turtlebot3_bringup`의 `robot.launch.py`는 `LDS_MODEL` 환경변수가 없으면 `KeyError`로 즉시 종료된다.
- 반드시 `LDS_MODEL=LDS-01` 또는 `LDS_MODEL=LDS-02`를 환경변수로 지정해야 한다.
- 이 프로젝트에서는 LDS-02를 사용한다.

### ROS_DOMAIN_ID 통일
- PC와 RPi5의 모든 ROS2 노드(네이티브 및 컨테이너)가 동일한 `ROS_DOMAIN_ID`를 사용해야 통신이 가능하다.
- 이 프로젝트는 `ROS_DOMAIN_ID=1`로 통일한다.

### 기존 RPi5에서 네이티브 ROS2와의 충돌
- 기존 RPi5처럼 네이티브 ROS2가 설치된 보드에서 컨테이너를 실행하면 FastDDS 포트 충돌이 생길 수 있다.
- 테스트 전 `ros2 daemon stop`으로 네이티브 데몬을 종료할 것.
- 클린 Ubuntu 보드를 사용하는 것이 가장 안전하다.
