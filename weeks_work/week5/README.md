# 5주차 진행 보고 — 가상 IMX219 카메라 연동 및 /camera/image_raw 토픽 발행

> **목표**: Isaac Sim 에 가상 IMX219 카메라를 설정하고 `/camera/image_raw` 토픽을 ROS2로 발행한다.
> `omni.graph.image.core` 없이 동작하므로 RTX 5070 Ti(Blackwell) 에서 안전하다.

---

## 1. 핵심 설계 결정

### 문제 재정리 (4주차에서 이어짐)

4주차에서 `omni.graph.image.core` 확장이 RTX 5070 Ti cold-start 에서 segfault 를 일으켜 제외했다.
그 결과 OmniGraph 방식의 `/camera/image_raw` 토픽 발행이 불가능해졌다.

### 해결 방향: Replicator annotator + rclpy 직접 발행

| 항목 | OmniGraph 방식 (4주차 실패) | Replicator 방식 (5주차 채택) |
|------|---------------------------|------------------------------|
| 필요 확장 | `omni.graph.image.core` ← **segfault** | 불필요 (항상 제외 유지) |
| 데이터 경로 | 카메라 → OmniGraph → DDS → Python | 카메라 → Replicator annotator → numpy |
| rclpy 발행 | OmniGraph 노드가 자동 발행 | Python 코드에서 직접 발행 |
| RTX 5070 Ti 안전성 | ✗ | ✓ |

`omni.replicator.core` 는 합성 데이터 생성(`generate_synthetic_data.py`)에서 이미 사용 중이므로 추가 의존성 없이 적용 가능하다.

---

## 2. 구현 내용

### 2-1. IMX219 카메라 USD 프림 생성 — base_footprint 자식

**수정 파일**: [isaac_sim/run_track_sim.py](../../isaac_sim/run_track_sim.py)

```python
# base_footprint 자식으로 카메라 생성 → 물리 시뮬레이션 이동 시 자동 추종
# (주의: turtlebot3_burger 루트 프림은 물리 업데이트 시 USD Xform 이 갱신되지 않음)
cam_path = _ROBOT_PATH + "/base_footprint/imx219_camera"
cam_usd  = UsdGeom.Camera.Define(stage, cam_path)

# IMX219 3.04 mm 렌즈 + 1/4" 센서 (3.68 × 2.76 mm)
cam_usd.CreateFocalLengthAttr(3.04)         # H-FOV ≈ 62.2°
cam_usd.CreateHorizontalApertureAttr(3.68)
cam_usd.CreateVerticalApertureAttr(2.76)
cam_usd.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10.0))

# USD XYZ Euler (b=-90° 고정 시) image up = (0, cos(a-c), sin(a-c))
#   a=0°, b=-90°, c=-90° → view=+X_base(전방), image up=+Z_base(위) ✓
xf = UsdGeom.XformCommonAPI(cam_usd)
xf.SetTranslate(Gf.Vec3d(0.05, 0.0, 0.12))
xf.SetRotate(Gf.Vec3f(0.0, -90.0, -90.0))
```

**USD 자식 프림 추종 원리 — 핵심 발견**:

Isaac Sim 물리 엔진은 articulation root(`turtlebot3_burger`)의 USD Xform 을 직접 갱신하지 않는다.
실제로 움직이는 강체(rigid body)는 `base_footprint` 이며, 이 프림의 월드 변환이 매 스텝 갱신된다.
카메라를 `base_footprint` 자식으로 등록해야 USD 계층 상속에 의해 카메라 월드 좌표가 자동으로 따라온다.

```
/World/turtlebot_.../turtlebot3_burger/    ← articulation root (USD Xform 갱신 안 됨 ✗)
└── base_footprint/                        ← 실제 물리 강체 (USD Xform 갱신됨 ✓)
    └── imx219_camera                      ← 카메라 프림 (자동 추종 ✓)
```

**디버그로 확인된 동작**:
```
# 정지 중
[CAM][D] world=(0.011,-1.869,1.006)  pixel_mean=229.1
# teleop 전진 후
[CAM][D] world=(-0.306,-1.776,1.006)  pixel_mean=227.8
[CAM][D] world=(-0.183,-1.050,1.006)  pixel_mean=217.7
```
world 좌표가 변화 → 카메라가 로봇을 정상 추종 ✓

### 2-2. Replicator RGB annotator

```python
import omni.replicator.core as rep

rp  = rep.create.render_product(cam_path, (640, 480))
ann = rep.AnnotatorRegistry.get_annotator("rgb")
ann.attach([rp])
```

시뮬레이션 루프에서 `ann.get_data()` 호출로 `(H, W, 4)` RGBA uint8 배열을 즉시 획득한다.
DDS를 경유하지 않으므로 `omni.graph.image.core` 가 필요 없다.

### 2-3. rclpy 직접 발행 + CameraInfo

```python
import rclpy
from sensor_msgs.msg import Image as RosImage, CameraInfo

# use_sim_time=True: Isaac Sim 브리지의 /clock 을 구독하여
# TF 타임스탬프(시뮬 시간)와 이미지 타임스탬프를 동기화
cam_node = rclpy.create_node(
    "isaac_imx219_bridge",
    parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", ..., True)]
)
img_pub  = cam_node.create_publisher(RosImage,   "/camera/image_raw",    1)
info_pub = cam_node.create_publisher(CameraInfo, "/camera/camera_info",  1)
```

```python
# 시뮬레이션 루프 내 (60 Hz 시뮬 → 10 Hz 발행)
def _publish_cam_frame(ann, cam_node, img_pub, info_pub, cam_info_msg):
    simulation_app.update()           # Replicator render_product 갱신
    frame = ann.get_data()
    rgba  = frame["data"]             # (H, W, 4) RGBA
    rgb   = np.ascontiguousarray(rgba[:, :, :3])   # (H, W, 3) RGB

    stamp = cam_node.get_clock().now().to_msg()

    msg = RosImage()
    msg.header.stamp    = stamp
    msg.header.frame_id = "camera_rgb_optical_frame"
    msg.height = rgb.shape[0]; msg.width = rgb.shape[1]
    msg.encoding = "rgb8"; msg.step = rgb.shape[1] * 3
    msg.data = rgb.tobytes()

    cam_info_msg.header.stamp = stamp
    info_pub.publish(cam_info_msg)
    img_pub.publish(msg)
    rclpy.spin_once(cam_node, timeout_sec=0.0)
```

### 2-4. Static TF: base_link → camera_rgb_optical_frame

RViz2 Camera 디스플레이가 카메라 시점을 TF 트리에서 추적하려면 이 변환이 필요하다.

```python
# 표준 수평 전방 카메라 쿼터니언 (ROS optical frame 규약)
# +Z_cam = +X_base(전방), +X_cam = -Y_base(로봇 우측), +Y_cam = -Z_base(아래)
tf_msg.transform.rotation.x = -0.5
tf_msg.transform.rotation.y =  0.5
tf_msg.transform.rotation.z = -0.5
tf_msg.transform.rotation.w =  0.5
```

### 2-5. CameraInfo 내부 파라미터 (IMX219 핀홀 모델)

```python
fx = (3.04 / 3.68) * 640  # = 528.70 px
fy = (3.04 / 2.76) * 480  # = 529.04 px
cx = 320.0;  cy = 240.0
# K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
# D = [0, 0, 0, 0, 0]   (왜곡 없음 — 핀홀 완전 모델)
```

### 2-6. generate_synthetic_data.py FOV 수정

합성 학습 데이터의 카메라 FOV를 실제 카메라와 일치시켜 Sim-to-Real 도메인 갭을 줄였다.

| 파라미터 | 수정 전 (90° FOV) | 수정 후 (IMX219 정확) |
|---------|-------------------|----------------------|
| `focal_length` | 3.0 mm | **3.04 mm** |
| `horizontal_aperture` | 6.0 mm | **3.68 mm** |
| `vertical_aperture` | 4.5 mm | **2.76 mm** |
| H-FOV | ≈ 90° | **≈ 62.2°** |
| V-FOV | ≈ 73.7° | **≈ 48.8°** |

---

## 3. 5주차 디버깅 이력 — 카메라 추종 + 이미지 방향

### 버그 1: 카메라가 로봇 이동을 따라오지 않음 (해결 ✓)

**원인**: 카메라를 `turtlebot3_burger` articulation root 의 자식으로 붙였으나,
Isaac Sim 물리 엔진은 articulation root 의 USD Xform 을 갱신하지 않고 `base_footprint` 만 갱신한다.
결과적으로 카메라의 월드 좌표가 항상 초기 위치에 고정됐다.

**수정**: `cam_path = _ROBOT_PATH + "/base_footprint/imx219_camera"` 로 변경.

### 버그 2: RViz2 Camera 디스플레이 "queue is full" 오류 (해결 ✓)

**원인**: `cam_node` 가 wall clock 을 사용하는 반면 Isaac Sim 브리지의 TF 는 시뮬 시간을 사용.
타임스탬프 불일치로 RViz2 가 이미지를 버림.

**수정**: `cam_node` 에 `use_sim_time=True` 파라미터 추가.
RViz2 실행 시에도 `--ros-args -p use_sim_time:=true` 필요.

### 버그 3: 이미지 90° 회전 (진행 중 ⚠️)

**원인 분석**: USD XYZ Euler `SetRotate(a, b, c)` 에서 `b=-90°` 고정 시
image up 방향 = `(0, cos(a-c), sin(a-c))`.

| 설정 | image up | 결과 |
|------|---------|------|
| `(-15°, -90°, 0°)` | `(0, 0.966, -0.259)` ≈ +Y_base(로봇 우측) | **90° CW 회전** |
| `(0°, -90°, -90°)` | `(0, 0, 1)` = +Z_base(위) | **이론상 정상** ← 현재 설정 |

> 현재 `SetRotate(0.0, -90.0, -90.0)` 으로 설정했으나 재시작 후 검증 필요.

**참고 스크린샷**: 12-07-18 rqt_image_view — 90° CW 상태에서도 장애물·차선 마킹·트랙 내용은 정확히 캡처됨.
이미지 내용은 올바르고 회전만 수정이 필요한 상태.

---

## 4. IMX219 카메라 파라미터

| 항목 | 값 | 근거 |
|------|----|------|
| 센서 | Sony IMX219 1/4" | TurtleBot3 Burger 기본 카메라 |
| 센서 크기 | 3.68 × 2.76 mm | 1/4" 규격 |
| 렌즈 초점거리 | 3.04 mm | 광각 렌즈 (표준 구성) |
| H-FOV | ≈ 62.2° | 2·atan(3.68/(2·3.04)) |
| V-FOV | ≈ 48.8° | 2·atan(2.76/(2·3.04)) |
| 발행 해상도 | 640 × 480 | libcamera 기본값 |
| 발행 주기 | 10 Hz | 차선 검출 최소 요구 |
| 높이 (로봇 기준) | 12 cm | TurtleBot3 Burger 실측 |
| 전방 오프셋 | 5 cm | base_link 기준 |

---

## 5. 실행 방법

### 카메라 토픽 활성화 (터미널 1 — Isaac Sim)

```bash
ENABLE_CAMERA=1 bash isaac_sim/launch_sim.sh
```

### 카메라 이미지 확인 (터미널 2 — ROS2 제어)

```bash
source /opt/ros/jazzy/setup.bash

# 토픽 수신 확인
ros2 topic echo /camera/image_raw --no-arr    # 헤더만 출력

# 토픽 주기 확인 (10 Hz)
ros2 topic hz /camera/image_raw

# rqt_image_view 에서 이미지 확인 (권장)
ros2 run rqt_image_view rqt_image_view

# RViz2 에서 시각화 (use_sim_time 필수)
rviz2 --ros-args -p use_sim_time:=true
# → Add → By topic → /camera/image_raw → Image
```

### 터미널을 분리해야 하는 이유

Isaac Sim 터미널에서 `source /opt/ros/jazzy/setup.bash` 실행 시 `LD_LIBRARY_PATH` 오염으로 segfault 발생.
반드시 별도 터미널 사용. (ros2_terminal.sh 활용 권장)

---

## 6. 완료 기준 체크리스트

- [x] IMX219 등가 카메라 USD 프림 생성 (run_track_sim.py)
- [x] **`base_footprint` 자식 프림**으로 등록하여 물리 이동 시 자동 추종 (articulation root 버그 수정)
- [x] `omni.graph.image.core` 없이 Replicator annotator 방식으로 이미지 획득
- [x] rclpy 직접 발행 → `/camera/image_raw` 토픽 출력 (10 Hz)
- [x] `/camera/camera_info` 토픽 발행 (IMX219 내부 파라미터)
- [x] `use_sim_time=True` 로 RViz2 clock 동기화 ("queue is full" 해결)
- [x] Static TF `base_link → camera_rgb_optical_frame` 발행
- [x] 합성 데이터 생성 카메라 FOV를 IMX219 정확 값으로 수정 (90° → 62.2°)
- [ ] **이미지 90° 회전 수정 최종 확인** — `SetRotate(0°, -90°, -90°)` 재시작 후 검증 필요
- [ ] `/scan` + `/camera/image_raw` 동시 발행 정상 동작 확인

---

## 7. 주의사항

### omni.graph.image.core 는 항상 제외

5주차 이후에도 `run_track_sim.py` 에서 `omni.graph.image.core` 는 항상 제외한다.

```python
sys.argv += ["--/app/extensions/excluded/0=omni.graph.image.core"]
```

### 카메라 프림은 반드시 base_footprint 자식으로

`turtlebot3_burger` (articulation root) 의 자식으로 카메라를 붙이면 이동 추종이 안 된다.
Isaac Sim 에서 실제로 USD Xform 이 갱신되는 프림은 `base_footprint` (rigid body) 이다.

### rclpy 이중 초기화 방지

OmniGraph ROS2 브리지가 rcl 컨텍스트를 먼저 초기화할 수 있으므로 `rclpy.init()` 은 `try/except RuntimeError` 로 감싸 중복 초기화를 방지한다.

### RViz2 실행 시 use_sim_time 필수

```bash
rviz2 --ros-args -p use_sim_time:=true
```
이 옵션 없이 실행하면 TF 타임스탬프 불일치로 Camera 디스플레이에 "queue is full" 오류 발생.

---

## 8. 5주차 이후 단계 — 이미 구현된 기능

아래 기능들은 5주차 카메라 연동 완료를 전제로 구현되어 있거나 준비된 상태다.

### 8-1. LiDAR 기반 차선 추종 + 장애물 회피 (구현 완료)

**파일**: `isaac_sim/lane_follow_nav.py` (473 lines)

TF 트리(world → odom → base_link)로 절대 위치를 추적하는 Pure Pursuit 차선 추종 시스템.
카메라가 아닌 트랙 기하 좌표 + LiDAR `/scan` 만으로 동작한다.

| 상태 | 조건 | 동작 |
|------|------|------|
| `LANE` | 정상 주행 | Pure Pursuit 차선 추종 |
| `WARN` | 장애물 접근 | 감속 + 차선 유지 |
| `AVOID` | 장애물 근접 | 감속 + 회피 조향 |
| `STOP_OBS` | 긴급 | 정지 + 강한 회피 회전 |
| `STOP_LINE` | 정지선 감지 | 2초 정지 후 재출발 |

> **다음 단계**: 이 시스템에 `/camera/image_raw` 기반 차선 인식을 결합하면
> 트랙 기하 하드코딩 없이 실제 영상으로 차선을 인식하는 구조로 전환 가능.

### 8-2. YOLOv8 합성 데이터 생성 파이프라인 (구현 완료)

**파일**: `isaac_sim/generate_synthetic_data.py`

Isaac Replicator 로 트랙 이미지 + YOLOv8-seg 레이블을 자동 생성한다.
카메라 FOV 는 이미 IMX219 정확 값으로 수정됐다.

```
data/synthetic/
├── images/{train,val}/*.png   ← Replicator 캡처 이미지
├── labels/{train,val}/*.txt   ← YOLOv8 세그멘테이션 레이블
└── dataset.yaml               ← nc=2 (white_lane, stop_line)
```

### 8-3. YOLOv8 학습 스크립트 (준비 완료)

**파일**: `data/train_yolo_lane.py`

합성 데이터로 YOLOv8-seg 모델을 학습하는 스크립트.
학습 완료 후 모델은 `/camera/image_raw` 구독 → 차선 마스크 + 정지선 검출에 사용 예정.

### 8-4. 슬라롬 장애물 환경 (구현 완료)

트랙에 장애물 6개 배치 완료 (Top 직선 3개, Left 직선 3개 지그재그).
카메라로 장애물 인식 후 회피하는 비전 기반 제어로 확장 예정.
