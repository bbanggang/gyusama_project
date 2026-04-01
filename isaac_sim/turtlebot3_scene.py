"""
4주차 — TurtleBot3 Isaac Sim 환경 구성 스크립트

실행 방법:
    cd ~/IsaacLab
    ./isaaclab.sh -p ~/gyusama-project/isaac_sim/turtlebot3_scene.py --enable_cameras

기능:
    1. TurtleBot3 Burger 에셋 로드 (NVIDIA Nucleus)
    2. 주행 환경 구성 (지면 + 직사각형 트랙 벽)
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
from isaaclab.sensors import CameraCfg, RayCasterCfg, patterns
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass

from turtlebot3_cfg import IMX219_CAMERA_CFG, LDS02_LIDAR_CFG, TURTLEBOT3_CFG


# ── 씬 구성 ──────────────────────────────────────────────────────────────────
@configclass
class TurtleBot3SceneCfg(InteractiveSceneCfg):
    """TurtleBot3 주행 환경 씬 설정"""

    # 지면 (RayCaster mesh_prim_paths와 경로 일치 필수)
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    # 조명 (돔 라이트 — 균일한 환경광)
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.9, 0.9)),
    )

    # 방향 조명 (그림자 생성 → 카메라 데이터 품질 향상)
    distant_light = AssetBaseCfg(
        prim_path="/World/DistantLight",
        spawn=sim_utils.DistantLightCfg(intensity=500.0, color=(1.0, 1.0, 0.9)),
    )

    # TurtleBot3 Burger 로봇
    robot: ArticulationCfg = TURTLEBOT3_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # IMX219 카메라 센서
    camera: CameraCfg = IMX219_CAMERA_CFG

    # LDS-02 LiDAR 센서
    lidar: RayCasterCfg = LDS02_LIDAR_CFG


def build_track(sim: SimulationContext):
    """직사각형 트랙 벽 생성 (2m x 3m)"""
    wall_cfg = sim_utils.CuboidCfg(
        size=(0.05, 3.0, 0.15),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        mass_props=sim_utils.MassPropertiesCfg(mass=0.0),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.4, 0.4, 0.9)),
    )
    # 좌벽
    wall_cfg.func("/World/track/WallLeft", wall_cfg, translation=(-1.0, 0.0, 0.075))
    # 우벽
    wall_cfg.func("/World/track/WallRight", wall_cfg, translation=(1.0, 0.0, 0.075))

    wall_cfg2 = sim_utils.CuboidCfg(
        size=(2.1, 0.05, 0.15),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        mass_props=sim_utils.MassPropertiesCfg(mass=0.0),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.4, 0.4, 0.9)),
    )
    # 앞벽
    wall_cfg2.func("/World/track/WallFront", wall_cfg2, translation=(0.0, 1.5, 0.075))
    # 뒷벽
    wall_cfg2.func("/World/track/WallBack", wall_cfg2, translation=(0.0, -1.5, 0.075))


def run_simulation(sim: SimulationContext, scene: InteractiveScene):
    """시뮬레이션 루프 — 센서 데이터 출력 및 물리 파라미터 검증"""
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    step_count = 0

    print("\n" + "=" * 60)
    print("TurtleBot3 Isaac Sim 환경 구성 완료")
    print(f"  시뮬레이션 dt : {sim_dt:.4f}s")
    print(f"  로봇 에셋     : NVIDIA Nucleus TurtleBot3 Burger")
    print(f"  카메라        : IMX219 640x480 @ 10Hz")
    print(f"  LiDAR         : LDS-02 360° @ 5Hz")
    print("=" * 60 + "\n")

    # 휠 조인트 ID 사전 조회 (루프 밖에서 한 번만)
    wheel_joint_ids, _ = scene["robot"].find_joints(
        ["a__namespace_wheel_left_joint", "a__namespace_wheel_right_joint"]
    )
    print(f"[INFO] 휠 조인트 ID: {wheel_joint_ids}")

    while simulation_app.is_running():
        # ── 전진 속도 명령 (직선 주행 테스트) ──────────────────────────────
        if step_count < 200:
            # TurtleBot3: 오른쪽 바퀴 조인트 축이 반전 → 전진 시 오른쪽 음수
            wheel_vel = torch.tensor([[2.0, -2.0]], device=sim.device)  # rad/s
        else:
            wheel_vel = torch.zeros(1, 2, device=sim.device)           # 정지

        scene["robot"].set_joint_velocity_target(wheel_vel, joint_ids=wheel_joint_ids)

        # ── 씬 업데이트 ─────────────────────────────────────────────────────
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        # ── 센서 데이터 출력 (50 스텝마다) ────────────────────────────────
        if step_count % 50 == 0:
            robot_pos = scene["robot"].data.root_pos_w[0]
            robot_vel = scene["robot"].data.root_lin_vel_w[0]

            lidar_data = scene["lidar"].data.ray_hits_w
            valid_hits = (lidar_data[0, :, 2] < 10.0).sum().item()

            print(f"[Step {step_count:4d} | t={sim_time:.2f}s]")
            print(f"  로봇 위치  : x={robot_pos[0]:.3f}, y={robot_pos[1]:.3f}, z={robot_pos[2]:.3f}")
            print(f"  로봇 속도  : vx={robot_vel[0]:.3f}, vy={robot_vel[1]:.3f}")
            print(f"  LiDAR 유효 포인트: {valid_hits}/360")

            if args_cli.enable_cameras:
                cam_shape = scene["camera"].data.output["rgb"].shape
                print(f"  카메라 출력: {cam_shape} (H x W x C)")
            print()

        sim_time += sim_dt
        step_count += 1


def main():
    sim_cfg = sim_utils.SimulationCfg(
        dt=0.005,           # 200Hz 물리 시뮬레이션
        render_interval=4,  # 50Hz 렌더링
        device=args_cli.device,
    )
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[2.0, -2.0, 1.5], target=[0.0, 0.0, 0.0])

    scene_cfg = TurtleBot3SceneCfg(num_envs=args_cli.num_envs, env_spacing=3.0)
    scene = InteractiveScene(scene_cfg)

    build_track(sim)

    sim.reset()
    print("[INFO] 시뮬레이션 초기화 완료")

    run_simulation(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
