"""
4주차 — TurtleBot3 Isaac Sim 환경 구성 스크립트

실행 방법:
    source ~/isaac_env/bin/activate
    cd ~/IsaacLab
    ./isaaclab.sh -p ~/gyusama-project/isaac_sim/turtlebot3_scene.py --enable_cameras

기능:
    1. TurtleBot3 Burger 에셋 로드 (로컬 USD)
    2. 주행 환경 구성 (평탄 격자 바닥 + 벽 장애물)
    3. IMX219 카메라 + LDS-02 LiDAR 센서 시뮬레이션
    4. 센서 데이터 콘솔 출력으로 물리 파라미터 검증
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="TurtleBot3 Isaac Sim 환경 구성")
parser.add_argument("--num_envs", type=int, default=1, help="시뮬레이션 환경 수")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── Isaac Sim 초기화 후 임포트 ────────────────────────────────────────────────
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.sensors.ray_caster import RayCasterCfg, patterns
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass

from turtlebot3_cfg import TURTLEBOT3_CFG, IMX219_CAMERA_CFG, LDS02_LIDAR_CFG


# ── 씬 구성 ──────────────────────────────────────────────────────────────────
@configclass
class TurtleBot3SceneCfg(InteractiveSceneCfg):
    """TurtleBot3 주행 환경 씬 설정 — 평탄 바닥 + 벽 장애물"""

    # 평탄 격자 바닥 (GroundPlaneCfg → default_environment.usd)
    # RayCaster가 자동으로 Plane/Mesh를 탐지하여 LiDAR 사용 가능
    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.GroundPlaneCfg(color=(0.15, 0.15, 0.15)),
    )

    # 조명
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )

    # 벽 장애물 (주행 환경 구성, LiDAR가 감지할 물리 오브젝트)
    wall_n = AssetBaseCfg(
        prim_path="/World/WallN",
        spawn=sim_utils.CuboidCfg(
            size=(4.0, 0.2, 0.5),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.6, 0.6, 0.6)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 2.0, 0.25)),
    )
    wall_s = AssetBaseCfg(
        prim_path="/World/WallS",
        spawn=sim_utils.CuboidCfg(
            size=(4.0, 0.2, 0.5),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.6, 0.6, 0.6)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, -2.0, 0.25)),
    )
    wall_e = AssetBaseCfg(
        prim_path="/World/WallE",
        spawn=sim_utils.CuboidCfg(
            size=(0.2, 4.0, 0.5),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.6, 0.6, 0.6)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(2.0, 0.0, 0.25)),
    )
    wall_w = AssetBaseCfg(
        prim_path="/World/WallW",
        spawn=sim_utils.CuboidCfg(
            size=(0.2, 4.0, 0.5),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.6, 0.6, 0.6)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-2.0, 0.0, 0.25)),
    )

    # TurtleBot3 Burger 로봇
    robot: ArticulationCfg = TURTLEBOT3_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # LDS-02 LiDAR
    lidar: RayCasterCfg = LDS02_LIDAR_CFG

    # IMX219 카메라 (--enable_cameras 플래그 필요)
    # camera: CameraCfg = IMX219_CAMERA_CFG  # 카메라 없이 실행 시 주석 유지


def run_simulation(sim: SimulationContext, scene: InteractiveScene):
    """시뮬레이션 루프"""
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    # 휠 조인트 ID 사전 조회
    wheel_joint_ids, _ = scene["robot"].find_joints(
        ["a__namespace_wheel_left_joint", "a__namespace_wheel_right_joint"]
    )
    print(f"[INFO] 휠 조인트 ID: {wheel_joint_ids}")

    print("\n" + "=" * 60)
    print("TurtleBot3 Isaac Sim 환경 구성 완료")
    print(f"  시뮬레이션 dt : {sim_dt:.4f}s")
    print(f"  바닥           : 평탄 격자 (GroundPlaneCfg)")
    print(f"  LiDAR         : LDS-02 360°")
    print(f"  카메라        : IMX219 640x480")
    print("=" * 60 + "\n")

    while simulation_app.is_running():

        # ── 500 스텝마다 리셋 ─────────────────────────────────────────────────
        if count % 500 == 0:
            root_state = scene["robot"].data.default_root_state.clone()
            root_state[:, :3] += scene.env_origins
            scene["robot"].write_root_pose_to_sim(root_state[:, :7])
            scene["robot"].write_root_velocity_to_sim(root_state[:, 7:])
            joint_pos = scene["robot"].data.default_joint_pos.clone()
            joint_vel = scene["robot"].data.default_joint_vel.clone()
            scene["robot"].write_joint_state_to_sim(joint_pos, joint_vel)
            scene.reset()
            print("[INFO] 로봇 상태 리셋")

        # ── 휠 속도 명령 ──────────────────────────────────────────────────────
        # URDF: 두 바퀴 모두 joint rpy="-1.57 0 0" + axis="0 0 1"
        # → 월드 Y축이 회전축 → 동일 부호 = 전진, 반대 부호 = 후진
        if count % 500 < 300:
            wheel_vel = torch.tensor([[2.0, 2.0]], device=sim.device)   # 전진
        else:
            wheel_vel = torch.tensor([[-2.0, -2.0]], device=sim.device) # 후진

        scene["robot"].set_joint_velocity_target(wheel_vel, joint_ids=wheel_joint_ids)

        # ── 씬 업데이트 ───────────────────────────────────────────────────────
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)
        sim_time += sim_dt
        count += 1

        # ── 50 스텝마다 센서 출력 ─────────────────────────────────────────────
        if count % 50 == 0:
            robot_pos = scene["robot"].data.root_pos_w[0]
            robot_vel = scene["robot"].data.root_lin_vel_w[0]

            lidar_hits = scene["lidar"].data.ray_hits_w
            total = lidar_hits.shape[1]
            valid = (lidar_hits[0, :, 0] != float("inf")).sum().item()

            phase = "전진" if count % 500 < 300 else "후진"
            print(f"[Step {count:4d} | t={sim_time:.2f}s | {phase}]")
            print(f"  로봇 위치  : x={robot_pos[0]:.3f}, y={robot_pos[1]:.3f}, z={robot_pos[2]:.3f}")
            print(f"  로봇 속도  : vx={robot_vel[0]:.3f}, vy={robot_vel[1]:.3f}")
            print(f"  LiDAR 유효 포인트: {valid}/{total}")
            if args_cli.enable_cameras and "camera" in scene.keys():
                cam_shape = scene["camera"].data.output["rgb"].shape
                print(f"  카메라 출력: {cam_shape}")
            print()


def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[3.0, -3.0, 2.0], target=[0.0, 0.0, 0.0])

    scene_cfg = TurtleBot3SceneCfg(num_envs=args_cli.num_envs, env_spacing=6.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO] 시뮬레이션 초기화 완료")

    run_simulation(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
