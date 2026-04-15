"""
gyusama-project — TurtleBot3 ROS2 시뮬레이션 런처

ros2-turtlebot-robot-only.usd (로봇+ROS2 OmniGraph)를 로드하고
월드 씬(바닥, 조명, 벽)을 코드로 구성합니다.

실행:
    ~/gyusama-project/isaac_sim/launch_sim.sh
"""
import os
import sys
from isaacsim import SimulationApp

# omni.graph.image.core: RTX 5070 Ti(Blackwell)에서 cold-start 시 충돌 → 제외
sys.argv += ["--/app/extensions/excluded/0=omni.graph.image.core"]

simulation_app = SimulationApp({
    "headless": False,
    "experience": "/home/linux/isaac_env/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.full.kit",
})

import omni.usd
import omni.kit.commands
import omni.timeline
from pxr import UsdGeom, UsdLux, UsdPhysics, PhysxSchema, Gf, Sdf

# ── USD 로드 ──────────────────────────────────────────────────────────────────
ASSETS_DIR = os.path.join(os.path.dirname(__file__), "assets")
ROBOT_USD  = os.path.join(ASSETS_DIR, "ros2-turtlebot-robot-only.usd")

print(f"[INFO] 로봇 USD 로드 중: {ROBOT_USD}")
omni.usd.get_context().open_stage(ROBOT_USD)

for _ in range(20):
    simulation_app.update()


# ── 월드 씬 구성 ──────────────────────────────────────────────────────────────
def build_world_scene():
    stage = omni.usd.get_context().get_stage()

    # 1) Physics Scene
    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/PhysicsScene"))

    # 2) 바닥 (GroundPlane)
    omni.kit.commands.execute(
        "AddGroundPlaneCommand",
        stage=stage,
        planePath="/World/GroundPlane",
        axis="Z",
        size=20.0,
        position=Gf.Vec3f(0.0, 0.0, 0.0),
        color=Gf.Vec3f(0.3, 0.3, 0.3),
    )

    # 3) 조명 — DomeLight
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(800.0)
    dome.CreateColorAttr(Gf.Vec3f(0.9, 0.9, 1.0))

    # 4) 벽 4개 (LiDAR 감지용 직사각형 방, 6m x 6m)
    walls = [
        # (prim_path,   size_xyz,           position_xyz)
        ("/World/WallN", (6.0, 0.2, 0.8), ( 0.0,  3.0, 0.4)),
        ("/World/WallS", (6.0, 0.2, 0.8), ( 0.0, -3.0, 0.4)),
        ("/World/WallE", (0.2, 6.0, 0.8), ( 3.0,  0.0, 0.4)),
        ("/World/WallW", (0.2, 6.0, 0.8), (-3.0,  0.0, 0.4)),
    ]
    for path, size, pos in walls:
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        xform = UsdGeom.XformCommonAPI(cube)
        xform.SetTranslate(Gf.Vec3d(*pos))
        xform.SetScale(Gf.Vec3f(*size))
        # 충돌 활성화
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(path))
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(path))
        rb = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path))
        rb.CreateKinematicEnabledAttr(True)

    print("[INFO] 월드 씬 구성 완료: 바닥 + 조명 + 벽(N/S/E/W)")


build_world_scene()

for _ in range(10):
    simulation_app.update()

# ── 시뮬레이션 시작 ───────────────────────────────────────────────────────────
timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("[INFO] 시뮬레이션 시작")
print("[INFO] 발행 중인 ROS2 토픽: /scan, /imu, /odom, /cmd_vel, /tf")
print("[INFO] 제어 시나리오: python3 ~/gyusama-project/isaac_sim/control_scenario.py")
print("[INFO] 수동 제어:    ros2 run teleop_twist_keyboard teleop_twist_keyboard")
print("[INFO] 종료: Ctrl+C")

try:
    while simulation_app.is_running():
        simulation_app.update()
except KeyboardInterrupt:
    print("\n[INFO] 종료")

timeline.stop()
simulation_app.close()
