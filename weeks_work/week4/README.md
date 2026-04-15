# 4주차 진행 보고 — Isaac Sim 환경 구성 및 TurtleBot3 제어 검증

> **목표**: TurtleBot3 USD 모델을 Isaac Sim에서 로드하고, 간단한 제어 코드로 물리 시뮬레이션 동작을 검증한다. ROS2 브리지를 통해 `/scan`, `/odom`, `/cmd_vel` 등 핵심 토픽이 정상 발행되는지 확인한다.

---

## 1. 진행한 작업 목록

### 4-1. TurtleBot3 USD 에셋 확보

URDF → USD 직접 변환 대신 NVIDIA 공식 isaac-sim-mobile-robot-rtab-map 레포지터리(`isaac_test`)에서 ROS2 OmniGraph가 내장된 `ros2-turtlebot.usd`를 확보하여 gyusama-project에 복사했다.

```bash
# isaac_test 레포 클론 및 USD 복사
cp ~/isaac_test/isaac-sim/ros2-turtlebot.usd \
   ~/gyusama-project/isaac_sim/assets/ros2-turtlebot.usd
```

해당 USD 파일에 내장된 구성 요소:

| 구성 요소 | 내용 |
|-----------|------|
| `turtlebot3_burger` | 로봇 형상 + 물리 (바퀴, IMU, LiDAR) |
| `simple_room` | 기본 제공 실내 환경 (바닥·벽·조명) |
| OmniGraph 노드 | `/scan`, `/imu`, `/odom`, `/cmd_vel`, `/tf` ROS2 발행 연결 |

---

### 4-2. Isaac Sim 실행 스크립트 작성

Isaac Sim 실행에 필요한 환경 변수 설정과 ROS2 브리지 경로를 통합한 셸 스크립트를 작성했다.

**작성 파일**: [isaac_sim/launch_sim.sh](../../isaac_sim/launch_sim.sh)

```bash
#!/bin/bash
BRIDGE_DIR=~/isaac_env/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$BRIDGE_DIR/jazzy/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$BRIDGE_DIR/jazzy/rclpy:$PYTHONPATH
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

rm -rf ~/isaac_env/.../isaacsim/kit/data/Kit/   # Kit 캐시 삭제
~/isaac_env/bin/python "$(dirname "$0")/run_track_sim.py"
```

**핵심 설계 결정**:

| 설정 | 이유 |
|------|------|
| `LD_LIBRARY_PATH`에 브리지 경로 추가 | Isaac Sim Python 3.11과 ROS2 Jazzy Python 3.12 버전 충돌 우회 |
| `VK_ICD_FILENAMES` 설정 | RTX 5070 Ti만 사용, Intel 내장 그래픽 배제 |
| Kit 캐시 삭제 | 이전 충돌로 손상된 USD 레이어 레지스트리 초기화 |
| ROS2 미리 source 금지 | `LD_LIBRARY_PATH` 충돌 → segfault 유발 |

---

### 4-3. RTX 5070 Ti Blackwell GPU 충돌 문제 해결

`isaacsim.exp.full.kit` 로드 시 `omni.graph.image.core` 확장이 초기화 단계에서 segfault를 일으키는 문제가 발생했다.

**충돌 위치**:
```
libomni.graph.image.core.plugin.so!_M_realloc_insert (hashtable emplace)
  ← libomni.kit.exec.core.plugin.so
  ← libomni.usd.so!reopenUsd
```

**원인**: RTX 5070 Ti (Blackwell 아키텍처, sm_120)에서 `omni.graph.image.core` 확장이 cold-start 시 메모리 재할당 버그 발생. 셰이더 캐시가 있을 때는 우회됐으나, 캐시 삭제 후 재현됨.

**해결**: `SimulationApp` 초기화 전 해당 확장을 제외 목록에 추가.

```python
import sys
sys.argv += ["--/app/extensions/excluded/0=omni.graph.image.core"]

simulation_app = SimulationApp({
    "headless": False,
    "experience": ".../isaacsim.exp.full.kit",
})
```

> **영향**: 카메라 이미지 토픽(`/camera/image_raw`) 발행 불가. LiDAR·IMU·Odometry·cmd_vel 토픽은 정상 동작.

---

### 4-4. IsaacLab 기반 제어 검증 (turtlebot3_scene.py)

Isaac Sim 직접 실행 전에 IsaacLab 프레임워크를 통해 TurtleBot3 USD의 물리 파라미터와 제어 인터페이스를 먼저 검증했다.

**작성 파일**: [isaac_sim/turtlebot3_scene.py](../../isaac_sim/turtlebot3_scene.py)

```bash
cd ~/IsaacLab
./isaaclab.sh -p ~/gyusama-project/isaac_sim/turtlebot3_scene.py
```

**검증 결과**:

| 항목 | 결과 |
|------|------|
| 휠 조인트 ID | [0, 1] 정상 탐지 |
| 전진 (wheel_vel = [2.0, 2.0]) | x축 양방향 이동 확인 |
| 후진 (wheel_vel = [-2.0, -2.0]) | x축 음방향 이동 확인 |
| LiDAR 유효 포인트 | 720/1080 (벽 감지) |

출력 샘플:
```
[Step   50 | t=0.25s | 전진]
  로봇 위치  : x=0.048, y=0.000, z=0.098
  로봇 속도  : vx=0.193, vy=0.000
  LiDAR 유효 포인트: 720/1080
```

---

### 4-5. ROS2 브리지 연동 시뮬레이션 (run_ros2_sim.py)

별도 제어 터미널과 Isaac Sim이 ROS2 토픽으로 통신하는 구조를 구성했다.

**작성 파일**:
- [isaac_sim/run_ros2_sim.py](../../isaac_sim/run_ros2_sim.py) — Isaac Sim 런처
- [isaac_sim/control_scenario.py](../../isaac_sim/control_scenario.py) — ROS2 `/cmd_vel` 제어 시나리오

**시뮬레이션 실행 구조**:

```
터미널 1 (Isaac Sim)             터미널 2 (ROS2 제어)
────────────────────────         ──────────────────────────────
launch_sim.sh                    source /opt/ros/jazzy/setup.bash
  └→ run_ros2_sim.py             python3 control_scenario.py
       └→ ROS2 브리지              └→ /cmd_vel 발행
          발행: /scan
                /imu                 또는
                /odom
                /tf              ros2 run teleop_twist_keyboard \
          구독: /cmd_vel              teleop_twist_keyboard
```

**제어 시나리오 (control_scenario.py)**:

| 동작 | 시간 | linear.x | angular.z |
|------|------|----------|-----------|
| 전진 | 10s | 0.2 | 0.0 |
| 우회전 | 5s | 0.0 | -0.5 |
| 전진 | 8s | 0.2 | 0.0 |
| 좌회전 | 5s | 0.0 | 0.5 |
| 전진 | 8s | 0.2 | 0.0 |
| 정지 | 3s | 0.0 | 0.0 |

---

### 4-6. 차선 트랙 씬 구성 (run_track_sim.py)

다음 주차의 YOLO 차선 인식 학습 데이터 수집을 위해 차선 마킹이 있는 직사각형 루프 트랙을 코드로 구성했다.

**작성 파일**: [isaac_sim/run_track_sim.py](../../isaac_sim/run_track_sim.py)

**트랙 구조**:

```
 ┌──────────────────────────────────────┐  ← 외부 벽 (LiDAR 감지)
 │  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │  흰색 경계선
 │  ╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌  │  황색 중앙선
 │  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │  흰색 경계선
 │          ┌──────────────┐           │
 │  ━━━━━   │  내부 공터   │   ━━━━━  │
 │  ╌╌╌╌╌   │  (7m × 4m)  │   ╌╌╌╌╌  │
 │  ━━━━━   └──────────────┘   ━━━━━  │
 │  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │
 │  ╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌  │
 │  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │
 └──────────────────────────────────────┘
   외부 10m × 7m  /  트랙 폭 1.5m
```

**USD 구성 요소**:

| Prim 경로 | 내용 |
|-----------|------|
| `/World/Ground` | 30m × 30m 바닥 (물리 충돌 포함) |
| `/World/Track/{Top,Bot,Left,Right}` | 아스팔트 색 트랙 표면 |
| `/World/Markings/*` | 흰색 경계선 + 황색 중앙선 |
| `/World/Walls/Out{N,S,E,W}` | 외부 경계 벽 (높이 0.3m) |
| `/World/Walls/In{N,S,E,W}` | 내부 경계 벽 (높이 0.3m) |
| `/World/DomeLight` + `/World/RectLight` | 조명 |

---

## 2. 완료 기준 확인

- [x] Isaac Sim에서 TurtleBot3 USD 모델 로드 및 시각화 성공
- [x] 물리 시뮬레이션 — 전진/후진 동작 확인 (IsaacLab)
- [x] LiDAR 유효 포인트 감지 확인 (720/1080)
- [x] ROS2 브리지 토픽 발행 구조 구성 (`/scan`, `/imu`, `/odom`, `/cmd_vel`, `/tf`)
- [x] `control_scenario.py`로 `/cmd_vel` 명령 전송 구조 완성
- [x] 차선 마킹 트랙 씬 코드 구성 완료 (`run_track_sim.py`)

---

## 3. 주의사항

### Isaac Sim 실행 전 ROS2 source 금지
- `source /opt/ros/jazzy/setup.bash`를 Isaac Sim 실행 터미널에서 하면 `LD_LIBRARY_PATH`가 오염되어 segfault가 발생한다.
- Isaac Sim 터미널과 ROS2 제어 터미널을 반드시 분리해서 사용한다.

### omni.graph.image.core 확장 제외 필수 (RTX 5070 Ti)
- RTX 5070 Ti (Blackwell, sm_120)에서 `isaacsim.exp.full.kit` cold-start 시 해당 확장이 충돌한다.
- 모든 Isaac Sim Python 스크립트의 `SimulationApp` 초기화 전에 아래 코드를 추가해야 한다.
  ```python
  import sys
  sys.argv += ["--/app/extensions/excluded/0=omni.graph.image.core"]
  ```
- 제외 시 `/camera/image_raw` 토픽은 발행되지 않는다. (5주차 카메라 연동 시 별도 해결 필요)

### 셰이더 캐시 삭제 주의
- `~/.local/share/ov/cache/`는 Isaac Sim 셰이더 컴파일 캐시다. 삭제하면 첫 실행 시 10~20분이 소요되거나 cold-start 충돌이 재현된다.
- `launch_sim.sh`에서는 Kit 데이터 캐시(`isaacsim/kit/data/Kit/`)만 삭제하고, OV 셰이더 캐시는 건드리지 않는다.

### IsaacLab vs SimulationApp 선택 기준

| 용도 | 권장 방식 |
|------|-----------|
| 물리 파라미터 검증, 관절 제어 | IsaacLab (`./isaaclab.sh -p script.py`) |
| ROS2 토픽 발행 (브리지 포함) | SimulationApp + `launch_sim.sh` |
