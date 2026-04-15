# 4주차 진행 보고 - Isaac Sim 환경 구성 및 TurtleBot3 제어 검증

> **목표**: TurtleBot3 USD 모델을 Isaac Sim에서 로드하고, 간단한 제어 코드로 물리 시뮬레이션 동작을 검증한다. ROS2 브리지를 통해 `/scan`, `/odom`, `/cmd_vel` 등 핵심 토픽이 정상 발행되는지 확인한다.
> 

---

## 1. 진행한 작업 목록

### 4-1. TurtleBot3 USD 에셋 확보

> **문제**: URDF → USD 직접 변환 시 형상·물리 정보만 담겨 Isaac Sim 실행 후에도 `/scan`, `/odom` 등 ROS2 토픽이 발행되지 않음
>
> **해결**: OmniGraph(ROS2 브리지 노드 그래프)가 내장된 NVIDIA 공식 USD 확보
>
> **결과**: 씬 로드 시점에 `/scan`, `/imu`, `/odom`, `/cmd_vel`, `/tf` 토픽이 자동 활성화

URDF를 Isaac Sim의 변환 툴로 USD로 변환하면 로봇의 형상 + 물리 정보만 담긴다. 따라서 Isaac Sim을 실행해도 `/scan`, `/odom` 등과 같은 토픽은 발행되지 않는다.

이를 해결하려면 Python 스크립트에서 센서 값을 읽어 rclpy로 수동 발행하거나, Isaac Sim GUI에서 OmniGraph를 직접 배선해야 한다. 따라서 이미 OmniGraph가 내장된 USD인 `ros2-turtlebot.usd`를 사용하여 씬 로드 시점에 자동으로 ROS2 브리지 노드들이 활성화되게 하였다.


**URDF vs USD**:

| 항목 | URDF | USD |
| --- | --- | --- |
| 풀네임 | Unified Robot Description Format | Universal Scene Description |
| 개발 주체 | ROS 커뮤니티 | Pixar (NVIDIA가 로보틱스에 채택) |
| 목적 | 로봇의 링크·조인트·센서 구조 기술 | 3D 씬 전체(형상, 물리, 조명, 애니메이션 등) 기술 |
| 표현 범위 | 로봇 단일 개체 | 로봇 + 환경 + 센서 + 데이터 흐름까지 통합 |
| 확장성 | ROS 생태계 전용 | 렌더링, 물리 엔진, 외부 툴 연동 가능 |

**Isaac Sim에서 USD를 사용하는 이유**:

Isaac Sim은 NVIDIA Omniverse 플랫폼 위에서 동작하며, Omniverse의 기본 씬 포맷이 USD다. URDF는 로봇 구조만 기술할 수 있지만 USD는 로봇, 환경, 조명, 물리 파라미터, 센서, 데이터 흐름 그래프까지 하나의 파일에 통합할 수 있어 Isaac Sim의 시뮬레이션 전체를 단일 파일로 정의하고 재사용할 수 있다.

**OmniGraph**:

OmniGraph는 Isaac Sim에 내장된 노드 기반 데이터 흐름 그래프 시스템이다. 코드 없이 노드를 연결하는 방식으로 센서 데이터를 읽고 가공하여 ROS2 토픽으로 발행하는 파이프라인을 구성할 수 있다.

```
카메라 / LiDAR / IMU 센서 (Isaac Sim 내부)
        ↓
OmniGraph 노드 (데이터 읽기 → 가공 → 포맷 변환)
        ↓
ROS2 토픽 발행 (/scan, /imu, /odom, /tf)
ROS2 토픽 구독 (/cmd_vel → 모터 제어)
```

OmniGraph가 USD에 내장되면 씬 로드 시점에 자동으로 노드 그래프가 활성화되어 별도의 Python 코드 없이 ROS2 브리지가 동작한다.




---

### 4-2. Isaac Sim 실행 스크립트 작성

> **문제**: Isaac Sim Python 3.11과 ROS2 Jazzy Python 3.12 버전 충돌, Intel 내장 그래픽 간섭, 비정상 종료로 인한 Kit 캐시 손상으로 Isaac Sim 실행 자체가 불안정
>
> **해결**: 충돌 원인별로 필요한 환경변수만 선별하여 단일 셸 스크립트로 통합
>
> **결과**: 환경변수 충돌 없이 Isaac Sim 안정적 실행 가능


**Kit 캐시** : Isaac Sim은 내부적으로 NVIDIA Omniverse Kit 위에서 동작하는데 Kit은 실행 시 USD scene 구성 정보를 캐시로 저장하여 Isaac Sim이 빠르게 scene을 열도록 Kit 캐시를 참조한다.

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
| --- | --- |
| `LD_LIBRARY_PATH`에 브리지 경로 추가 | Isaac Sim Python 3.11과 ROS2 Jazzy Python 3.12 버전 충돌 우회 |
| `VK_ICD_FILENAMES` 설정 | RTX 5070 Ti만 사용, Intel 내장 그래픽 배제 |
| Kit 캐시 삭제 | 이전 충돌로 손상된 USD 레이어 레지스트리 초기화 |
| ROS2 미리 source 금지 | `LD_LIBRARY_PATH` 충돌 → segfault 유발 |



---

### 4-3. RTX 5070 Ti Blackwell GPU 충돌 문제 해결

> **문제**: RTX 5070 Ti (Blackwell, sm_120)에서 `omni.graph.image.core` 확장이 cold-start 시 메모리 재할당 버그로 segfault 발생 → Isaac Sim 실행 불가
>
> **해결**: `SimulationApp` 초기화 전 `omni.graph.image.core`를 제외 목록에 추가하여 해당 확장의 초기화 자체를 건너뜀
>
> **결과**: Isaac Sim 정상 실행. 단, `/camera/image_raw` 토픽 발행 불가 → 5주차에 Isaac Sim Python API 직접 접근 방식으로 우회 예정

`isaacsim.exp.full.kit` 로드 시 `omni.graph.image.core` 확장이 초기화 단계에서 segfault를 일으키는 문제가 발생했다.

**용어 설명 :**

`cold-start` :   프로그램이 캐시나 사전 준비 없이 처음부터 모든 초기화를 수행하며 실행되는 상태

`isaacsim.exp.full.kit`  : 전체 기능 패키지로 Isaac Sim을 실행하기 위한 설정 파일 → 렌더링, 물리, ros2 bridge, 카메라 LiDAR 등 모든 확장을 적용

`omni.graph.image.core` : 카메라 이미지 데이터를 Omnigraph 노드로 처리하는 확장으로 Isaac Sim 내부 카메라 센서 데이터를 토픽으로 발행

segfault(segmentation fault) : 프로그램이 접근하면 안 되는 메모리 주소를 건드렸을 때 OS가 강제로 프로세서를 죽이는 오류

`SimulationApp` : Isaac Sim 실행 환경을 초기화하고 관리하는 클래스. Kit 런타임 시작, `.kit` 파일 파싱, 확장 초기화, USD 씬 로드, 시뮬레이션 루프 시작 순서로 동작함

**충돌 위치**:

```cpp
libomni.graph.image.core.plugin.so!_M_realloc_insert (hashtable emplace) #_M_realloc_insert에서 메모리 재할당
  ← libomni.kit.exec.core.plugin.so
  ← libomni.usd.so!reopenUsd
```


```python
import sys
sys.argv += ["--/app/extensions/excluded/0=omni.graph.image.core"]

simulation_app = SimulationApp({
    "headless": False,
    "experience": ".../isaacsim.exp.full.kit",
})
```


**카메라 데이터 확보 방향**:

`omni.graph.image.core` 제외로 `/camera/image_raw` ROS2 토픽은 발행되지 않지만, Isaac Sim Python API를 통해 카메라 버퍼를 직접 읽는 방식으로 우회한다. ROS2 토픽을 거치지 않고 Isaac Sim이 렌더링한 이미지를 메모리에서 numpy 배열로 직접 가져오기 때문에 해당 확장 없이도 동작한다.

현재 `ros2-turtlebot.usd`에는 카메라 프림이 포함되어 있지 않으므로, 시뮬레이션 실행 시 Python 코드로 카메라를 런타임에 씬에 추가한다. 이때 실제 TurtleBot3에 장착할 카메라의 물리적 위치, 해상도, 초점거리, 센서 크기를 동일하게 설정해야 시뮬레이션에서 학습한 차선 인식 모델이 실제 환경에서도 동작한다.


| 항목 | ROS2 토픽 방식 | Isaac Sim API 직접 방식 |
| --- | --- | --- |
| `omni.graph.image.core` 필요 여부 | 필요 | 불필요 |
| 데이터 경로 | 센서 → OmniGraph → DDS → Python 구독 | 센서 → numpy 배열 직접 |
| Isaac Sim 외부 노드와 이미지 공유 | 가능 | 불가 |
| YOLO 추론 연동 | 별도 ROS2 노드 필요 | Isaac Sim 내부에서 직접 처리 |

차선 인식 학습 데이터 수집과 YOLO 추론은 Isaac Sim 내부에서 완결되므로 API 직접 방식으로 충분하다.

---

### 4-4. IsaacLab 기반 제어 검증 (turtlebot3_scene.py)

> **문제**: Isaac Sim 실행 환경이 불안정한 상태에서 USD 물리 파라미터까지 동시에 검증하면 문제 발생 시 USD 자체의 문제인지 실행 환경 문제인지 원인 특정 불가
>
> **해결**: 최소 확장만 로드하는 IsaacLab으로 ROS2 브리지·환경변수 충돌 변수를 제거한 뒤 USD 물리만 격리하여 검증
>
> **결과**: 휠 조인트 ID, 전진/후진 동작, LiDAR 포인트 정상 확인 → USD 자체는 정상임을 보장, 이후 문제 발생 시 디버깅 범위 축소

**IsaacLab vs Isaac Sim**:

IsaacLab은 Isaac Sim과 별도의 실행 프로그램이 아닌, Isaac Sim을 내부적으로 실행하는 로보틱스/강화학습 프레임워크다. Isaac Gym의 후속으로 Isaac Sim 위에 올라가 강화학습, 모방학습, 관절 제어 연구 환경을 제공한다.

| 항목 | Isaac Sim | IsaacLab |
| --- | --- | --- |
| 성격 | 시뮬레이터 본체 | Isaac Sim 위의 로보틱스/RL 프레임워크 |
| 실행 방식 | `SimulationApp()` + `.kit` 파일 | `./isaaclab.sh -p script.py` |
| 로드하는 확장 범위 | full.kit — 렌더링, ROS2 브리지, 카메라 등 전체 | 최소한의 확장만 로드 |
| ROS2 브리지 | 포함 | 미포함 |
| 주 용도 | 시각화, ROS2 연동, 씬 구성 | 강화학습, 모방학습, 관절 제어 검증 |

**이유** : Isaac Sim 실행 환경이 불안정한 상황에서 IsaacLab은 로드하는 확장 범위가 더 적어 `omni.graph.image.core` segfault 발생 구간 자체를 지나지 않는다. ROS2 브리지, 환경변수 충돌 문제를 모두 제거한 상태에서 USD 물리 파라미터만 순수하게 검증할 수 있기 때문에 IsaacLab 프레임워크를 통해 검증을 진행했다.

**작성 파일**: [isaac_sim/turtlebot3_scene.py](../../isaac_sim/turtlebot3_scene.py)

```bash
cd ~/IsaacLab
./isaaclab.sh -p ~/gyusama-project/isaac_sim/turtlebot3_scene.py
```

**검증 결과**:

| 항목 | 결과 |
| --- | --- |
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

> **문제**: Isaac Sim과 native ROS2를 같은 터미널에서 실행 시 Python 버전 충돌, LD_LIBRARY_PATH 오염 등으로 segfault 발생
>
> **해결**: Isaac Sim 전용 터미널과 ROS2 제어 터미널을 분리하여 환경변수 충돌 원천 차단
>
> **결과**: `/scan`, `/imu`, `/odom`, `/tf` 토픽 발행 및 `/cmd_vel` 수신 정상 동작 확인

별도 제어 터미널과 Isaac Sim이 ROS2 토픽으로 통신하는 구조를 구성

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

**터미널을 반드시 분리해야 하는 이유**:

Isaac Sim과 native ROS2는 같은 환경변수 공간을 공유하도록 설계되지 않았다. Isaac Sim 실행 터미널에서 `source /opt/ros/jazzy/setup.bash`를 실행하면 아래 문제들이 복합적으로 발생한다.

| 문제 | 원인 | 증상 |
| --- | --- | --- |
| `LD_LIBRARY_PATH` 충돌 | Isaac Sim 브리지 라이브러리와 native ROS2 라이브러리 혼재 | segfault |
| Python 버전 충돌 | Isaac Sim Python 3.11 vs native ROS2 Jazzy Python 3.12 | import 오류, 런타임 크래시 |
| RMW 구현체 충돌 | Isaac Sim은 `rmw_fastrtps_cpp` 고정, native source 시 덮어씌워짐 | 토픽 미발행, DDS 통신 불가 |
| 환경변수 누적 오염 | 같은 터미널에서 Isaac Sim 재실행 시 `LD_LIBRARY_PATH`에 경로 중복 누적 | 의도하지 않은 버전의 라이브러리 로드 |

터미널은 각자 독립된 환경변수를 가지기 때문에, 터미널 2에서 native ROS2를 source해도 터미널 1의 Isaac Sim 환경에 영향을 주지 않는다.

conda 같은 가상환경으로 하나의 환경에 통합하는 방법도 있으나 아래 문제로 현실적이지 않다.

| 문제 | 내용 |
| --- | --- |
| Python 버전 강제 통일 | conda 환경을 Python 3.11로 고정하면 native ROS2 Jazzy의 모든 패키지를 Python 3.11용으로 소스 빌드해야 함 |
| 의존성 충돌 | rclpy, rclcpp 등 수십 개 ROS2 패키지의 의존성이 Isaac Sim 패키지와 충돌 가능 |
| 유지보수 부담 | Isaac Sim 또는 ROS2 업데이트 시 통합 환경 전체를 재구성해야 함 |
| 공식 미지원 | NVIDIA 공식 문서에서 Isaac Sim 실행 터미널의 ROS2 source를 명시적으로 금지하며 터미널 분리를 권장 |

**제어 시나리오 (control_scenario.py)**:

| 동작 | 시간 | linear.x | angular.z |
| --- | --- | --- | --- |
| 전진 | 10s | 0.2 | 0.0 |
| 우회전 | 5s | 0.0 | -0.5 |
| 전진 | 8s | 0.2 | 0.0 |
| 좌회전 | 5s | 0.0 | 0.5 |
| 전진 | 8s | 0.2 | 0.0 |
| 정지 | 3s | 0.0 | 0.0 |

---

### 4-6. 차선 트랙 씬 구성 (run_track_sim.py)

YOLO 차선 인식 학습 데이터 수집을 위해 차선 마킹이 있는 루프 트랙을 코드로 구성해야 한다.
- 현재 임의로 만들어 turtlebot3의 주행을 확인 중

**작성 파일**: [isaac_sim/run_track_sim.py](../../isaac_sim/run_track_sim.py)


---


## 2. 주의사항

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

