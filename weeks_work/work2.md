# gyusama-project 진행 정리 2

work.md 이후 진행된 작업 기록 (Week 8~9)

---

## FP32 ONNX 배포 전환 + Dynamic INT8 시도

### 목적
Week 8 벤치마크에서 정적 INT8 양자화(14.8Hz)가 성능 기준을 통과했으나 실제 추론 시 mAP50=0.000으로 검출이 전혀 안 되는 문제가 발생했다. 대안으로 Dynamic INT8을 시도하고, 최종 배포 모델을 결정한다.

### 정적 INT8 vs 동적 INT8

| | 정적 INT8 | 동적 INT8 |
|---|---|---|
| 변환 대상 | 가중치 + 활성화 모두 INT8 | 가중치만 INT8, 활성화는 FP32 유지 |
| 캘리브레이션 | 필요 (대표 입력 데이터로 활성화 범위 측정) | 불필요 |
| 속도 | 더 빠름 | 정적보다 느림 |
| 정확도 | 캘리브레이션 품질에 따라 크게 달라짐 | 상대적으로 안정적 |

### 결과

**Dynamic INT8 양자화 (`quantize_onnx.py --mode dynamic`)**

캘리브레이션 데이터 없이 가중치만 INT8로 변환하는 방식으로 시도했다.

```python
from onnxruntime.quantization import quantize_dynamic, QuantType

quantize_dynamic(
    model_input=args.input,
    model_output=args.output,
    weight_type=QuantType.QInt8,   # 활성화는 FP32 유지, 가중치만 INT8
)
```

결과: mAP50=0.432 (FP32 0.707 대비 −38.7%) → 정확도 손실 과다로 미채택.

**최종 배포 모델: FP32 ONNX (`best.onnx`)**
- RPi5 추론 속도: ~2.8 FPS (294ms/frame)
- 정확도 손실 없음
- 속도 문제는 별도 개선 과제로 분리

---

## RPi5 실배포 파이프라인 완성

### 목적
IMX219 카메라 → YOLOv8 추론 → `/cmd_vel` → OpenCR → Dynamixel 모터까지 전체 end-to-end 파이프라인을 실제 TurtleBot3 Burger에서 동작시킨다.

### 결과

전체 파이프라인 동작 확인:

```
IMX219(RPi5 호스트) → /camera/image_raw
  → inference-node(Docker) → YOLO 추론 → /cmd_vel (TwistStamped)
  → control-node(Docker) → turtlebot3_node → OpenCR(/dev/ttyACM0)
  → Dynamixel TTL → Motor ID=1,2 (XL430-W250, 1,000,000 baud)
```

### 핵심 시행착오

**① OpenCR 펌웨어 버전 불일치 (ROS1 → ROS2)**

`turtlebot3_node`는 `/cmd_vel`을 수신해 차동 구동 공식으로 좌/우 바퀴 속도를 계산하고 DynamixelSDK로 OpenCR에 전달하는 ROS2 ↔ 하드웨어 브릿지 역할을 한다.

OpenCR에 ROS1 펌웨어가 올라가 있어 `turtlebot3_node`가 연결에 실패했다.
RPi5(aarch64)에서 32비트 ARM 바이너리 실행을 위한 compat 라이브러리 설치 후 ROS2 펌웨어를 플래시했다.

```bash
sudo dpkg --add-architecture armhf
sudo apt install -y libc6:armhf libstdc++6:armhf
export OPENCR_PORT=/dev/ttyACM0 && export OPENCR_MODEL=burger
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr   # V230127R1
```

**② `/cmd_vel` 타입 불일치 (Isaac Sim: Twist / RPi5: TwistStamped)**

`lane_detect.py`는 Isaac Sim 연동을 위해 `geometry_msgs/msg/Twist`로 발행하고 있었다.
RPi5에 배포하니 ROS2 Jazzy의 `turtlebot3_node` 2.3.6이 `/cmd_vel`을 `geometry_msgs/msg/TwistStamped`로 구독해 모터가 전혀 반응하지 않았다.

Isaac Sim은 `Twist` 유지, RPi5만 `TwistStamped` 사용해야 하므로 환경변수로 분기 처리했다.

```python
# CMD_VEL_STAMPED=1 (RPi5 docker-compose) → TwistStamped
# 미설정 (Isaac Sim) → 기존 Twist 유지
_USE_STAMPED = os.environ.get('CMD_VEL_STAMPED', '0') == '1'
```

**③ Dynamixel 토크 비활성화 상태**

스택 스매싱(stack smashing)은 C++ 프로그램이 스택 메모리 경계를 초과해 쓰기를 수행할 때 컴파일러 canary 값이 변조된 것을 감지하고 프로세스를 강제 종료하는 메모리 오류다. OpenCR 펌웨어 교체 과정에서 ROS1/ROS2 펌웨어 불일치 상태로 `turtlebot3_node`를 반복 재시작하면서 패킷 포맷 불일치로 크래시가 반복됐다. Dynamixel이 이를 에러로 인식해 토크를 자동 비활성화하는 보호 모드로 전환됐고, 재기동 후 수동으로 활성화해야 했다.

```bash
ros2 service call /motor_power std_srvs/srv/SetBool '{data: true}'
```

펌웨어 정상 설치 이후에는 크래시가 재발하지 않았다.

---

## PC 원격 모니터링 (FastDDS 크로스 머신)

### 목적
RPi5에서 실행 중인 `/lane/debug_image`를 PC에서 실시간으로 확인한다.

### 결과

FastDDS는 기본적으로 멀티캐스트로 같은 네트워크의 참여자를 자동 탐색한다. RPi5와 PC가 같은 로컬 네트워크에 있었지만 공유기가 멀티캐스트 패킷을 차단해 자동 탐색이 실패했다.

대신 유니캐스트 피어 설정으로 RPi5의 IP와 포트를 직접 명시해, FastDDS가 멀티캐스트 없이 해당 주소로 직접 discovery 패킷을 보내도록 했다. `ROS_DOMAIN_ID=1` 기준으로 포트가 7650, 7660~7664로 계산되므로 이를 모두 명시해야 모든 참여자가 발견된다.

```bash
# PC에서 FastDDS 유니캐스트 피어 설정 (도메인 1 포트 명시)
cat > /tmp/fastdds_peer.xml << 'XMLEOF'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="default_participant" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator><udpv4><address>192.168.0.155</address><port>7650</port></udpv4></locator>
          <locator><udpv4><address>192.168.0.155</address><port>7660</port></udpv4></locator>
          <locator><udpv4><address>192.168.0.155</address><port>7662</port></udpv4></locator>
          <locator><udpv4><address>192.168.0.155</address><port>7664</port></udpv4></locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
XMLEOF

export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_peer.xml
export ROS_DOMAIN_ID=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 daemon stop && ros2 daemon start
ros2 run rqt_image_view rqt_image_view   # /lane/debug_image 선택
```

---

## 잔여 과제 — 장애물 회피 완성

블록 다이어그램 기준으로 inference 통합 노드가 차선 추종 제어 토픽을 발행하고, LiDAR 데이터로 장애물 감지 시 behavior tree가 장애물 회피를 우선 처리하는 구조다.

현재 구현은 behavior tree 없이 `lane_detect.py` 단일 노드 내부에서 LiDAR 상태(clear/warn/avoid/stop)를 판단해 차선 조향과 블렌딩하는 방식이다. 결과적으로 차선 추종이 지배적으로 동작하며 장애물 회피가 제대로 우선처리되지 않는다. 또한 `FRONT_DEG=22`로 전방각을 좁혔음에도 좁은 차선에서 차선 벽을 장애물로 오인하는 경우가 남아 있다.

**추후 개선 필요 사항**
- behavior tree 기반 별도 노드 분리: lane_follow 노드와 obstacle_avoid 노드를 독립적으로 구현하고 BT로 우선순위 제어
- 장애물/차선 벽 구분 로직 개선

---

## 잔여 과제 — FPS 개선

RPi5 CPU에서 FP32 YOLOv8n 추론 속도가 **~2 FPS (294ms/frame)** 로 실시간 차선 추종이 어렵다. 추론 속도가 개선되면 차선 추종 알고리즘도 함께 최적화할 필요가 있다. 아래 방법 중 하나 이상을 적용해야 한다.

### 방법 1: YOLO 입력 해상도 축소 (즉시 적용 가능, 권장)

`YOLO_SZ = 640 → 320` 으로 변경하면 추론 속도 **3~4배 향상 (~6~8 FPS)** 예상.

```python
# lane_detect.py, test_lane_cam.py 동일 수정
YOLO_SZ = 320   # 640 → 320
```

정확도 다소 감소 예상. 합성 데이터로 320×320 재학습하면 정확도 회복 가능.

### 방법 2: 정적 INT8 양자화 재시도 (캘리브레이션 개선)

Week 8 벤치마크에서 정적 INT8은 **14.8Hz** 통과했으나 실제 검출이 mAP50=0.000으로 붕괴됐다. 원인은 캘리브레이션 데이터셋 분포 불일치로 추정된다.

개선 방향:
- 캘리브레이션 이미지를 실제 카메라 촬영본으로 교체
- `QuantFormat.QDQ` → `QuantFormat.QOperator` 전환 시도
- activation 양자화 제외 (`activation_type` 미설정)

### 방법 3: ncnn 변환 (ARM 최적화 추론 엔진)

Tencent ncnn은 ARM CPU에 고도 최적화된 추론 엔진으로 ONNX Runtime보다 ARM에서 2~3배 빠르다.

```bash
# ONNX → ncnn 변환
pip install onnx2ncnn
onnx2ncnn best.onnx best.param best.bin
```

YOLOv8 → ncnn 변환은 별도 후처리 코드 수정 필요.

### 방법 4: Hailo-8L AI Kit 활용

RPi5 공식 AI 가속기 (26 TOPS). YOLOv8n 기준 **~30 FPS** 달성 가능.  
Hailo Model Zoo에서 YOLOv8n HEF(Hailo 실행 포맷) 변환 도구 제공.

```bash
hailortcli fw-control identify   # Hailo-8L 연결 확인
hailo convert best.onnx --target-device hailo8l
```

하드웨어 구매 필요. 가장 근본적인 해결책.

### 방법 5: 모델 재학습 (320×320 입력)

YOLOv8n을 처음부터 320×320 입력으로 재학습하면 추론 속도와 정확도를 동시에 확보할 수 있다.

```bash
# train_yolo_lane.py 수정
yolo train model=yolov8n.pt imgsz=320 epochs=100 data=data/lane.yaml
```

합성 데이터 생성 시 카메라 해상도도 320×320으로 맞춰야 한다.
