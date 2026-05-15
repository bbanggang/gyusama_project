# RPi5 실배포 세팅 보고서

## 환경 개요

| 항목 | 내용 |
|------|------|
| 보드 | Raspberry Pi 5 (Ubuntu 24.04 aarch64) |
| 카메라 | IMX219 (CSI) |
| 모터 | Dynamixel (TurtleBot3 Burger, U2D2 어댑터) |
| LiDAR | ld08 |
| ROS2 | Jazzy (Docker 컨테이너 내부) |
| 추론 모델 | YOLOv8n-detect INT8 ONNX (`best_int8.onnx`) |

---

## 1. Docker 빌드 전 — RPi5 초기 세팅

Docker 이미지만 올리면 되는 것이 아니다. 카메라(IMX219)와 관련된 드라이버와 libcamera는
호스트에서 직접 세팅해야 한다.

### 1-1. Ubuntu 24.04 설치 및 기본 패키지

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git cmake ninja-build meson python3-pip \
    python3-jinja2 python3-ply python3-yaml libyaml-dev \
    libssl-dev libgnutls28-dev libevent-dev libdw-dev \
    libboost-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libdrm-dev libudev-dev
```

### 1-2. IMX219 카메라 활성화 (`dtoverlay`)

Ubuntu 24.04 + RPi5에서 IMX219를 사용하려면 `config.txt`에 overlay를 추가해야 한다.

```bash
sudo nano /boot/firmware/config.txt
# 하단에 추가:
# dtoverlay=imx219
```

재부팅 후 적용:
```bash
sudo reboot
```

### 1-3. libcamera RPi fork 빌드 (필수)

Ubuntu apt에서 제공하는 libcamera(v0.7.0)는 RPi5의 PiSP ISP 파이프라인을 지원하지 않는다.
IMX219를 인식하려면 Raspberry Pi 공식 libcamera fork(v0.7.1+rpt)를 소스 빌드해야 한다.

```bash
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --prefix=/opt/ros/jazzy \
    -Dpipelines=rpi/pisp,rpi/vc4 \
    -Dipas=rpi/pisp,rpi/vc4 \
    -Dbuildtype=release
ninja -C build
sudo ninja -C build install
sudo ldconfig
```

> **왜 prefix를 `/opt/ros/jazzy`로?**
> `ros-jazzy-camera-ros` apt 패키지가 `/opt/ros/jazzy/lib/libcamera.so.0.7`를 참조한다.
> 이 경로에 RPi fork를 설치해야 camera_ros가 올바른 버전을 로드한다.

빌드 완료 후 카메라 인식 확인:
```bash
cam --list
# 출력: imx219 (/base/axi/pcie@.../imx219@10) 확인
```

### 1-4. video 그룹 추가

```bash
sudo usermod -aG video $USER
# 재로그인 후 적용 확인: groups | grep video
```

### 1-5. Docker 설치

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
```

---

## 2. Docker 구성

카메라 노드는 libcamera 버전 충돌로 호스트에서 직접 실행하고,
나머지 두 노드는 Docker 컨테이너로 운용한다.

```
[호스트]                         [Docker]
camera_node ──/camera/image_raw──▶ inference-node (lane_detect.py)
                                       │ /cmd_vel
                                   control-node (turtlebot3_bringup)
                                       │ Dynamixel
```

### 구성 파일 위치

```
gyusama-project/
└── docker/
    ├── Dockerfile.rpi5
    ├── docker-compose.yml
    └── entrypoint.sh
```

### 실행 순서

```bash
# 1) 호스트에서 카메라 노드 실행
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_DOMAIN_ID=1 \
  ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480

# 2) Docker 컨테이너 실행
cd ~/gyusama-project/docker
docker compose up inference-node   # 추론 노드
docker compose up control-node     # 모터 제어 노드 (Dynamixel 연결 후)
```

---

## 3. 발생한 문제와 해결

### 문제 1: camera_ros가 잘못된 libcamera 버전 로드

**증상**
```
libcamera v0.7.0
[ERROR] no cameras available
```

**원인**  
`ros-jazzy-camera-ros` apt 패키지는 빌드 시 `/opt/ros/jazzy/lib/libcamera.so.0.7`(v0.7.0)을
RPATH로 링크한다. RPi fork를 `/usr/lib/aarch64-linux-gnu/`에 설치해도
`camera_ros` 컴포넌트(`libcamera_component.so`)는 RPATH 우선순위에 따라
여전히 `/opt/ros/jazzy/lib/`의 v0.7.0을 로드했다.

확인 방법:
```bash
ldd /opt/ros/jazzy/lib/libcamera_component.so | grep libcamera
# → /opt/ros/jazzy/lib/libcamera.so.0.7 (v0.7.0)
strings /opt/ros/jazzy/lib/libcamera.so.0.7 | grep "^v0\."
# → v0.7.0  (RPi fork가 아닌 apt 버전)
```

**해결**  
RPi fork를 `/opt/ros/jazzy/lib/`에 직접 복사하여 덮어씀:
```bash
sudo cp /usr/lib/aarch64-linux-gnu/libcamera.so.0.7.1 /opt/ros/jazzy/lib/libcamera.so.0.7.1
sudo ln -sf libcamera.so.0.7.1 /opt/ros/jazzy/lib/libcamera.so.0.7
sudo cp /usr/lib/aarch64-linux-gnu/libcamera-base.so.0.7.1 /opt/ros/jazzy/lib/libcamera-base.so.0.7.1
sudo ln -sf libcamera-base.so.0.7.1 /opt/ros/jazzy/lib/libcamera-base.so.0.7
```

검증:
```bash
strings /opt/ros/jazzy/lib/libcamera.so.0.7 | grep "^v0\."
# → v0.7.1+rpt20260429 ✓
```

---

### 문제 2: camera_node를 Docker 컨테이너에서 실행 불가

**증상**  
docker-compose에 camera-node를 포함시켰을 때:
```
[ERROR] no cameras available
```
호스트에서는 정상이지만 컨테이너 내부에서는 카메라 미인식.

**원인**  
컨테이너 내부의 libcamera는 apt v0.7.0이고, 호스트의 RPi fork IPA 모듈과
버전이 맞지 않아 ISP 파이프라인(pisp) 초기화 실패.
컨테이너 안에서 apt libcamera를 RPi fork로 교체하려면 전체 빌드가 필요하다.

**해결**  
camera-node를 docker-compose에서 제거하고 호스트에서 직접 실행.
`docker-compose.yml` 주석에 이유 명시:
```yaml
# camera-node는 호스트 libcamera RPi fork 버전과 컨테이너 버전 불일치로
# 호스트에서 직접 실행
```

---

### 문제 3: ROS_DOMAIN_ID 불일치

**증상**  
`ros2 topic list`에 토픽이 보이지 않음.

**원인**  
Docker 컨테이너는 `ROS_DOMAIN_ID=1`로 설정되어 있는데,
호스트에서 camera_node를 기본값(0)으로 실행함.

**해결**  
```bash
# 카메라 노드 실행 시 명시적으로 지정
ROS_DOMAIN_ID=1 ros2 run camera_ros camera_node ...

# 영구 설정 (.bashrc)
echo 'export ROS_DOMAIN_ID=1' >> ~/.bashrc
source ~/.bashrc
```

---

### 문제 4: inference-node가 /camera/image_raw 수신 불가 (FastDDS SHM)

**증상**  
```bash
# 컨테이너 내부에서
ros2 topic info /camera/image_raw
# Publisher count: 1, Subscription count: 2  ← 구독은 등록됨

ros2 topic hz /camera/image_raw
# no data  ← 데이터가 전혀 안 옴
```

호스트에서는 30 FPS로 정상 수신되지만 컨테이너에서는 0건.

**원인**  
ROS2 Jazzy의 기본 DDS 미들웨어인 FastDDS 3.x는 같은 머신 내 통신에
공유 메모리(SHM, `/dev/shm`) 전송을 우선 사용한다.
Docker 컨테이너는 기본적으로 호스트의 IPC 네임스페이스와 분리되어 있어
SHM 데이터에 접근할 수 없다.

`network_mode: host`는 네트워크만 공유하고 `/dev/shm`은 공유하지 않는다.
`ipc: host` 옵션을 추가해도 FastDDS의 SHM 소켓 경로 방식과 맞지 않아
여전히 데이터가 전달되지 않았다.

**해결**  
호스트와 컨테이너 양쪽에서 FastDDS 전송을 SHM 대신 UDPv4로 강제:

```bash
# 호스트 camera_node 실행 시
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_DOMAIN_ID=1 \
  ros2 run camera_ros camera_node ...
```

`docker-compose.yml` inference-node 환경변수에 추가:
```yaml
environment:
  - FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

결과: 7.1 FPS로 추론 및 `/cmd_vel` 발행 정상 확인.

---

### 문제 5: cv_bridge — bgra8 인코딩 변환 실패

**증상**  
inference-node가 실행되고 이미지도 수신되는데 `/cmd_vel`이 발행되지 않음.

**원인**  
camera_ros는 libcamera의 XRGB8888 포맷을 ROS2 이미지 메시지의 `bgra8` 인코딩으로 발행한다.
`lane_detect.py`의 `_cb_image`는 `desired_encoding="bgr8"`으로 변환을 시도했는데,
cv_bridge가 `bgra8 → bgr8` 직접 변환을 지원하지 않아 매 프레임 예외가 발생하고
`return`으로 조기 종료되었다. 따라서 이후의 `/cmd_vel` 발행 코드에 도달하지 못했다.

**해결**  
`passthrough` 인코딩으로 먼저 받은 뒤 cv2로 변환:

```python
if msg.encoding in ('bgra8', 'rgba8', 'xrgb8888'):
    raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    code = cv2.COLOR_RGBA2BGR if msg.encoding == 'rgba8' else cv2.COLOR_BGRA2BGR
    bgr = cv2.cvtColor(raw, code)
else:
    bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
```

---

### 문제 6: control-node 실행 실패 — LDS_MODEL 환경변수 누락

**증상**
```
KeyError: 'LDS_MODEL'
```

**원인**  
`turtlebot3_bringup` launch 파일이 `LDS_MODEL` 환경변수를 필수로 요구하는데
`docker-compose.yml`에 누락되어 있었다.

**해결**  
`docker-compose.yml` control-node 환경변수에 추가:
```yaml
- LDS_MODEL=ld08
```

---

## 4. 최종 구성 요약

### 호스트 실행 (매 부팅 후)

```bash
source /opt/ros/jazzy/setup.bash

# 카메라 노드 (포그라운드 또는 백그라운드)
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_DOMAIN_ID=1 \
  ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 &
```

### Docker 컨테이너 실행

```bash
cd ~/gyusama-project/docker
docker compose up inference-node &   # 추론 + /cmd_vel 발행
docker compose up control-node       # 모터 제어 (Dynamixel 연결 필요)
```

### 동작 확인

```bash
ros2 topic hz /camera/image_raw   # 30 FPS 확인
ros2 topic echo /cmd_vel          # 제어 명령 확인
ros2 topic hz /lane/debug_image   # 디버그 이미지 확인
```

---

## 5. 잔여 과제

- [ ] Dynamixel U2D2 연결 후 control-node 실제 구동 확인
- [ ] 실제 트랙 주행 테스트 및 파라미터 튜닝
- [ ] 부팅 자동 실행 스크립트 작성 (systemd 또는 rc.local)
- [ ] Sim-to-Real 갭 분석
