"""
TurtleBot3 Burger 로봇 설정 (IsaacLab ArticulationCfg)

- 에셋: NVIDIA Nucleus 클라우드 (ISAAC_NUCLEUS_DIR/Robots/Turtlebot/...)
- 액추에이터: Dynamixel XL430-W250 (속도 제어 모드)
- 센서: IMX219 카메라 + LDS-02 LiDAR
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.sensors import CameraCfg, RayCasterCfg, patterns
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

# ── TurtleBot3 Burger USD 경로 ────────────────────────────────────────────────
TURTLEBOT3_USD_PATH = (
    f"{ISAAC_NUCLEUS_DIR}/Robots/Turtlebot/turtlebot3_burger/turtlebot3_burger.usd"
)

# ── Dynamixel XL430-W250 액추에이터 설정 ──────────────────────────────────────
# TurtleBot3 Burger는 속도 제어 모드로 동작
DYNAMIXEL_XL430_CFG = DCMotorCfg(
    joint_names_expr=["wheel_left_joint", "wheel_right_joint"],
    saturation_effort=1.4,   # XL430 스톨 토크 (Nm)
    effort_limit=1.4,
    velocity_limit=4.82,     # XL430 최대 속도 (rad/s, 46 RPM)
    stiffness={".*": 0.0},   # 속도 제어 모드 → stiffness=0
    damping={".*": 0.5},
)

# ── TurtleBot3 Burger 로봇 설정 ───────────────────────────────────────────────
TURTLEBOT3_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=TURTLEBOT3_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.01),   # 지면 위 1cm
        joint_pos={".*": 0.0},
        joint_vel={".*": 0.0},
    ),
    actuators={"wheel_drive": DYNAMIXEL_XL430_CFG},
)

# ── IMX219 카메라 설정 ────────────────────────────────────────────────────────
# 실제 IMX219: 3280x2464, f=3.04mm / 시뮬레이션: 640x480 다운샘플
IMX219_CAMERA_CFG = CameraCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base_link/front_camera",
    update_period=0.1,       # 10Hz
    height=480,
    width=640,
    data_types=["rgb", "distance_to_image_plane"],
    spawn=sim_utils.PinholeCameraCfg(
        focal_length=3.04,
        focus_distance=1.0,
        horizontal_aperture=3.68,   # IMX219 센서 폭 (mm)
        clipping_range=(0.05, 20.0),
    ),
    offset=CameraCfg.OffsetCfg(
        pos=(0.069, 0.0, 0.107),    # base_link 기준 카메라 마운트 위치
        rot=(0.5, -0.5, 0.5, -0.5),
        convention="ros",
    ),
)

# ── LDS-02 LiDAR 설정 (RayCaster) ────────────────────────────────────────────
# LDS-02 스펙: 360° / 1° 해상도 / 0.12~3.5m 범위 / 5Hz
LDS02_LIDAR_CFG = RayCasterCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base_link",
    mesh_prim_paths=["/World/Ground"],
    pattern_cfg=patterns.LidarPatternCfg(
        channels=1,
        vertical_fov_range=(-0.0, 0.0),
        horizontal_fov_range=(-180.0, 180.0),
        horizontal_res=1.0,         # 1° 해상도 → 360 포인트
    ),
    max_distance=3.5,
    drift_range=(-0.005, 0.005),
    offset=RayCasterCfg.OffsetCfg(
        pos=(0.0, 0.0, 0.18),       # 라이다 마운트 높이
    ),
)
