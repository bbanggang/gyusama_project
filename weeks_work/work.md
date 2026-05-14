# gyusama-project 진행 정리 


## Isaac Sim 가상 카메라 연동

### 목적
Isaac Sim에서 TurtleBot3가 주행할 때 카메라 이미지를 ROS2 토픽(`/camera/image_raw`)으로 발행해야 합성 데이터 생성과 실시간 추론 파이프라인을 연결할 수 있다.

### 결과

**`run_track_sim.py` — IMX219 카메라 USD 프림 생성 및 이미지 발행**  
`omni.replicator.core` annotator로 렌더 이미지를 numpy 배열로 직접 획득하고, rclpy로 `/camera/image_raw`를 발행한다.
카메라는 로봇 이동 시 자동 추종을 위해 반드시 `base_footprint` 자식 프림으로 생성한다.

```python
import omni.replicator.core as rep  # Isaac Sim Replicator — 카메라 렌더 결과를 numpy 배열로 획득
import rclpy                         # 노드 생성 및 토픽 발행
from pxr import UsdGeom, Gf         # UsdGeom: USD 씬 프림(Camera, Xform) 정의 / Gf: 3D 벡터·행렬 연산
from sensor_msgs.msg import Image as RosImage  # camera/image_raw 발행

# 카메라 USD 프림 — base_footprint 자식으로 생성
# 즉, USD의 turtlebot3_burger 프림의 자식으로 카메라 프림을 생성하여 로봇을 따라가게 한다.
cam_path = _ROBOT_PATH + "/base_footprint/imx219_camera"
cam_usd  = UsdGeom.Camera.Define(stage, cam_path)
cam_usd.CreateFocalLengthAttr(_CAM_FL)           # IMX219 3.04mm (H-FOV 62.2°)
cam_usd.CreateHorizontalApertureAttr(_CAM_HA)    # 3.68mm
cam_usd.CreateVerticalApertureAttr(_CAM_VA)      # 2.76mm

xf = UsdGeom.XformCommonAPI(cam_usd)
xf.SetTranslate(Gf.Vec3d(_CAM_OFX, 0.0, _CAM_OFZ))
xf.SetRotate(Gf.Vec3f(75.0, 0.0, -90.0))   # 전방 하향 15° 시선

# Replicator annotator로 이미지 획득
rp  = rep.create.render_product(cam_path, (_CAM_W, _CAM_HT))
ann = rep.AnnotatorRegistry.get_annotator("rgb")
ann.attach([rp])

# rclpy로 /camera/image_raw 발행
img_pub = cam_node.create_publisher(RosImage, "/camera/image_raw", 1)
```

### 실행 결과 
![image1](https://github.com/user-attachments/assets/a5434694-b7cc-4264-8a94-c20f36b1b996)


### 핵심 시행착오

**① `omni.graph.image.core` segfault**  
RTX 5070 Ti(Blackwell)에서 해당 확장이 cold-start 시 segfault를 일으켜 OmniGraph 방식으로 카메라 토픽을 발행할 수 없었다. `run_track_sim.py` 실행 시 해당 확장을 명시적으로 제외하고 Replicator annotator 방식으로 전환했다.

```python
import sys  # Isaac Sim 실행 인수(argv) 조작 — 로드할 확장 목록에서 문제 확장 제외

# run_track_sim.py 상단 — 확장 제외 선언
sys.argv += ["--/app/extensions/excluded/0=omni.graph.image.core"]
```

**② 카메라가 로봇을 따라오지 않는 문제**  
`run_track_sim.py`에서 카메라 프림을 `turtlebot3_burger`(articulation root) 자식으로 붙였더니 로봇이 이동해도 카메라가 고정된 채로 남았다. Isaac Sim 물리 엔진은 articulation root의 USD Xform을 갱신하지 않고 `base_footprint`(rigid body)만 갱신하기 때문이었다. 카메라 경로를 `base_footprint` 자식으로 수정해 해결했다.

```python
# 수정 전 (카메라 고정됨)
cam_path = _ROBOT_PATH + "/turtlebot3_burger/imx219_camera"

# 수정 후 (로봇 이동 추종)
cam_path = _ROBOT_PATH + "/base_footprint/imx219_camera"
```

---

## 합성 데이터 생성 · YOLOv8 학습 · 추론 파이프라인 완성

### 목적
Isaac Sim 합성 이미지로 차선 검출 모델을 학습하고, 실시간으로 `/cmd_vel`을 발행하는 end-to-end 파이프라인을 완성한다.

### 결과

**`generate_synthetic_data.py` — 합성 데이터 생성**  
트랙을 렌더링하며 카메라를 웨이포인트를 따라 이동시켜 1000장 자동 캡처 (train 800 / val 200). 이미지에서 흰색 픽셀을 추출해 좌/우 bbox 레이블을 자동 생성한다.

```python
import random      # 조명 강도 랜덤 샘플링 — 이미지마다 다른 조명 조건 생성
from pxr import UsdLux  # USD 조명 프림(DomeLight, RectLight) 속성 제어

# 조명 랜덤화 — run_track_sim.py 씬 조건과 동일 범위로 맞춤
UsdLux.DomeLight(dome_prim).CreateIntensityAttr().Set(random.uniform(50, 200))
UsdLux.RectLight(rect_prim).CreateIntensityAttr().Set(random.uniform(1000, 5000))
```

- **합성 데이터 샘플**
![합성 데이터 샘플](https://github.com/user-attachments/assets/293b22cc-cc5d-4d13-9f16-f176e645aeeb)

- **val 예측 결과**
![val 예측 결과](https://github.com/user-attachments/assets/1db1ab17-ea30-4b16-9c98-538364627f1b)



**`train_yolo_lane.py` — YOLOv8n-detect 학습**  
합성 데이터로 100 epoch 학습. mAP50=0.763, `best.onnx` 생성 (11.7MB).

- **학습 loss·mAP 그래프**
![학습 loss·mAP 그래프](https://github.com/user-attachments/assets/595a9cc0-ae53-41e5-9af7-cb82f6e2585b)

**`lane_detect.py` — 차선 추종 알고리즘**  
YOLOv8 detect 모델의 출력(cx, cy, w, h, conf)을 이용해 차선 중심 오프셋을 계산한다.  
이미지를 좌(x<320) / 우(x≥320) 구역으로 분할하고 각 구역에서 confidence가 가장 높은 bbox 1개씩 선택(존 기반 중복 제거). 좌측 bbox 중심 x와 우측 bbox 중심 x의 평균을 목표 중심으로 삼아 이미지 중앙과의 차이를 오프셋(−1~1)으로 계산하고 `angular.z = −KP × offset` 비례 제어로 조향한다.

```python
import onnxruntime as ort                        # ONNX 모델 로드 및 추론 실행
import numpy as np                          
import cv2                                      
import rclpy                                    
from rclpy.node import Node                   
from sensor_msgs.msg import Image as RosImage    
from geometry_msgs.msg import Twist           
from cv_bridge import CvBridge                   # ROS2 Image 메시지 ↔ OpenCV numpy 배열 변환

# lane_detect.py — 존 기반 중복 제거 (NMS 후 구역별 최고 conf 1개만 유지)
zone_best: dict[int, dict] = {}
for box in results:
    zone = 0 if box["cx"] < w_orig / 2 else 1
    if zone not in zone_best or box["conf"] > zone_best[zone]["conf"]:
        zone_best[zone] = box
return list(zone_best.values())

# lane_detect.py — 오프셋 계산 (좌/우 bbox 중심 x의 평균 → 이미지 중앙 기준 정규화)
left_cx  = sorted_boxes[0]["cx"]
right_cx = sorted_boxes[-1]["cx"]
mid_cx   = (left_cx + right_cx) / 2.0
offset   = (mid_cx - img_w / 2.0) / (img_w / 2.0)   # [-1, 1]
```

- **/lane/debug_image 수신 화면**
![debug image](https://github.com/user-attachments/assets/b13e1b73-22f4-4580-8878-83bbb3e6dbb7)

### 핵심 시행착오

**① 조명 과노출로 인한 학습 데이터 품질 불량**  
`generate_synthetic_data.py`에서 DomeLight=600, RectLight=28000으로 생성한 이미지의 흰색 픽셀 비율이 99.3%에 달해 차선과 도로 구분이 불가능했다. 실제 시뮬 환경에서 YOLO가 max_conf=0.31로 아무것도 검출하지 못했다. `run_track_sim.py`와 `generate_synthetic_data.py` 모두 아래와 같이 수정해 흰색 픽셀 비율을 1~3%로 낮췄다.

```python
# run_track_sim.py
import random      # 조명 강도 랜덤 샘플링
from pxr import Gf, UsdLux  # Gf: 색상 벡터 생성 / UsdLux: DomeLight·RectLight 속성 제어

# run_track_sim.py — 씬 조명 수정
color=Gf.Vec3f(0.02, 0.02, 0.02)   # GroundPlane 거의 검정
dome.CreateIntensityAttr(120.0)     # DomeLight: 600 → 120
rect.CreateIntensityAttr(3000.0)    # RectLight: 28000 → 3000

# generate_synthetic_data.py — 학습 데이터 조명도 동일 범위로 맞춤
UsdLux.DomeLight(dome_prim).CreateIntensityAttr().Set(random.uniform(50, 200))
UsdLux.RectLight(rect_prim).CreateIntensityAttr().Set(random.uniform(1000, 5000))
```

**② Fallback 오프셋 이중 적용 버그**  
`lane_detect.py`의 흰색 픽셀 폴백은 전체 이미지 기준 마스크를 만든 뒤 우측 절반(x=320~639)에 `np.where()`를 적용한다. 이때 반환되는 x 좌표는 이미 이미지 전체 기준의 절대 좌표(320~639)다. 그런데 기존 코드가 이를 우측 구역 내 상대 좌표로 잘못 인식해 `+320`을 추가로 더했고, 결과적으로 우측 차선 중심이 항상 640~959(이미지 밖)로 계산되어 오프셋이 항상 +0.50으로 고정되는 문제였다.

```python
import numpy as np  # np.where()로 흰색 픽셀 좌표 추출 — 폴백 차선 중심 계산에 사용

# lane_detect.py — 수정 전 (이중 오프셋 버그)
boxes.append({"cx": rx + IMG_W // 2})   # rx(320~639) + 320 → 640~959 (이미지 밖)

# lane_detect.py — 수정 후
# rx 는 np.where(right) 의 결과 → 이미 절대 x 좌표 (320~639)
boxes.append({"cx": rx})
```

**③ FPS 저하 — 렌더 중복 및 GPU 경합**  
두 가지 원인이 순차적으로 발견됐다.

- **1차 (`run_track_sim.py`)**: 시뮬 루프에서 `sim_context.step(render=True)`가 렌더를 수행한 뒤 `simulation_app.update()`가 한 번 더 렌더를 실행해 ~3FPS가 나왔다. 중복 호출을 제거하자 ~5.7FPS로 개선됐다.

```python
# run_track_sim.py — 수정 후 (중복 렌더 제거)
while simulation_app.is_running():
    sim_context.step(render=True)          # 렌더 1회
    if _ENABLE_CAM and _cam_tick % _cam_interval == 0:
        _publish_cam_frame(*_cam_bridge)   # simulation_app.update() 제거
```

- **2차 (`lane_detect.py`)**: Isaac Sim(GPU 렌더링)과 ONNX 추론이 같은 GPU를 경합했다. ONNX provider를 `CUDAExecutionProvider`에서 `CPUExecutionProvider`로 전환해 GPU를 Isaac Sim에 전용하자 ~11.5FPS로 추가 개선됐다.

```python
import onnxruntime as ort  # ONNX 추론 엔진 — provider 선택으로 CPU/GPU 실행 장치 지정

# lane_detect.py — ONNX CPU 추론으로 전환
providers = ["CPUExecutionProvider"]
self.sess = ort.InferenceSession(onnx_path, providers=providers)
```

---

## RPi5 엣지 배포 및 INT8 양자화

### 목적
실제 로봇(RPi5, CUDA 없음)에서 ONNX 모델이 실시간(≥10Hz) 처리가 가능한지 검증하고, 미달 시 INT8 양자화로 속도를 확보한다.

### 결과

**`scripts/benchmark_rpi5.py` — RPi5 추론 속도 측정**  
RPi5 FP32 추론 결과 **186.4ms / 5.4Hz** (목표 10Hz 미달).

**`models/quantize_onnx.py` — 정적 INT8 양자화**  
`data/synthetic/images/val` 100장을 캘리브레이션 데이터로 사용해 QDQ 포맷 INT8 양자화를 적용했다. ARM CPU에서 INT8 연산이 FP32보다 효율적으로 처리된다.

```python
import numpy as np   # 캘리브레이션 이미지를 (1, 3, H, W) float32 텐서로 변환
import cv2           # 캘리브레이션 이미지 파일 로드 및 리사이즈(640×640)
from onnxruntime.quantization import (
    quantize_static,        # FP32 ONNX → INT8 ONNX 정적 양자화 실행 함수
    CalibrationDataReader,  # 캘리브레이션 데이터 공급 인터페이스 — 상속하여 구현
    QuantType,              # 양자화 비트 타입 지정 (QInt8)
    QuantFormat,            # 양자화 포맷 지정 (QDQ — ARM CPU 최적)
)

# models/quantize_onnx.py — 정적 INT8 양자화 실행
quantize_static(
    model_input=args.input,          # best.onnx (11.7MB)
    model_output=args.output,        # best_int8.onnx (3.2MB)
    calibration_data_reader=reader,  # val 이미지 100장
    quant_format=QuantFormat.QDQ,    # ARM CPU 최적 포맷
    weight_type=QuantType.QInt8,
    activation_type=QuantType.QInt8,
)
```

RPi5 INT8 추론: **67.4ms / 14.8Hz PASS** ✅ (모델 크기 11.7MB → 3.2MB, 3.6x 압축)


- **벤치마크 터미널 출력 — benchmark_rpi5.py 실행 결과**
![benchmark_rpi5](https://github.com/user-attachments/assets/99b3849f-8e57-4124-9ee3-db2a34b845eb)

### 핵심 시행착오

**① PC↔RPi5 네트워크 연결 불가**  
PC에 유선(enp4s0)과 무선(wlx705dccf3d9d3) 두 인터페이스가 활성화되어 있고, `192.168.0.0/24` 트래픽이 metric이 낮은 유선(metric=100)으로 라우팅되었다. RPi5는 WiFi로만 연결되어 있어 유선으로는 도달할 수 없었다. 아래 명령으로 RPi5 IP만 WiFi 인터페이스로 우선 라우팅하여 해결했다.

```bash
# 라우팅 테이블에서 유선이 우선 선택되는 구조
# 192.168.0.0/24 dev enp4s0  metric 100  ← 우선 (유선, RPi5 없음)
# 192.168.0.0/24 dev wlx...  metric 600  ← 후순위 (무선, RPi5 있음)

# RPi5 IP만 WiFi로 라우팅
sudo ip route add 192.168.0.153 dev wlx705dccf3d9d3
```
