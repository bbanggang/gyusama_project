"""
gyusama-project — TurtleBot3 차선 트랙 시뮬레이션 런처

ros2-turtlebot-robot-only.usd (로봇 + ROS2 OmniGraph)를 로드하고
직사각형 루프 트랙 월드를 코드로 구성합니다.

트랙 구조:
  외부: 10m x 7m  /  내부 공터: 7m x 4m  /  트랙 폭: 1.5m
  마킹: 흰색(외/내 경계선) + 황색(중앙선)
  벽  : 외부 4면 + 내부 4면 (LiDAR 감지)

실행:
    ~/gyusama-project/isaac_sim/launch_sim.sh track
    또는 launch_sim.sh 마지막 줄을 run_track_sim.py 로 변경
"""
import os
import sys
from isaacsim import SimulationApp

sys.argv += ["--/app/extensions/excluded/0=omni.graph.image.core"]

simulation_app = SimulationApp({
    "headless": False,
    "experience": "/home/linux/isaac_env/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.full.kit",
})

import omni.usd
import omni.timeline
from pxr import UsdGeom, UsdLux, UsdPhysics, UsdShade, Gf, Sdf

# ── USD 로드 ──────────────────────────────────────────────────────────────────
ASSETS_DIR = os.path.join(os.path.dirname(__file__), "assets")
ROBOT_USD  = os.path.join(ASSETS_DIR, "ros2-turtlebot-robot-only.usd")

print(f"[INFO] 로봇 USD 로드 중: {ROBOT_USD}")
omni.usd.get_context().open_stage(ROBOT_USD)
for _ in range(20):
    simulation_app.update()


# ── 헬퍼 함수 ─────────────────────────────────────────────────────────────────
def _make_material(stage, mat_path, color):
    mat    = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness",    Sdf.ValueTypeNames.Float).Set(0.85)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _bind_color(stage, prim, color, mat_cache):
    if color not in mat_cache:
        idx = len(mat_cache)
        mat_cache[color] = _make_material(stage, f"/World/Materials/M{idx}", color)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat_cache[color])


def _slab(stage, path, cx, cy, cz, sx, sy, sz, color, mat_cache, physics=False):
    """얇은 직사각형 슬랩 (트랙 표면 / 마킹 / 벽 공용)"""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    api  = UsdGeom.XformCommonAPI(cube)
    api.SetTranslate(Gf.Vec3d(cx, cy, cz))
    api.SetScale(Gf.Vec3f(sx, sy, sz))
    prim = cube.GetPrim()
    if physics:
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(True)
    _bind_color(stage, prim, color, mat_cache)
    return prim


# ── 트랙 씬 구성 ──────────────────────────────────────────────────────────────
def build_track_scene():
    stage     = omni.usd.get_context().get_stage()
    mat_cache = {}
    UsdGeom.Scope.Define(stage, "/World/Materials")

    # 색상
    ASPHALT = (0.13, 0.13, 0.13)
    GROUND  = (0.28, 0.28, 0.28)
    WHITE   = (1.00, 1.00, 1.00)
    YELLOW  = (0.95, 0.85, 0.00)
    WALL_C  = (0.55, 0.55, 0.55)

    # 트랙 치수
    OX = 5.0   # 외부 반폭 x  → 전체 10m
    OY = 3.5   # 외부 반폭 y  → 전체 7m
    TW = 1.5   # 트랙 폭
    IX = OX - TW   # 내부 반폭 x = 3.5
    IY = OY - TW   # 내부 반폭 y = 2.0

    # ── 1) Physics Scene ──────────────────────────────────────────────────────
    phys = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    phys.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    phys.CreateGravityMagnitudeAttr(9.81)

    # ── 2) 바닥 (마찰 재질 포함) ──────────────────────────────────────────────
    _slab(stage, "/World/Ground", 0, 0, -0.025, 30, 30, 0.05, GROUND, mat_cache, physics=True)

    # 바닥 마찰 재질 — 바퀴가 헛돌지 않도록 staticFriction, dynamicFriction 설정
    from pxr import PhysxSchema
    ground_prim = stage.GetPrimAtPath("/World/Ground")
    phys_mat = UsdShade.Material.Define(stage, "/World/GroundPhysicsMaterial")
    phys_mat_prim = phys_mat.GetPrim()
    UsdPhysics.MaterialAPI.Apply(phys_mat_prim).CreateStaticFrictionAttr(0.7)
    UsdPhysics.MaterialAPI.Apply(phys_mat_prim).CreateDynamicFrictionAttr(0.5)
    UsdPhysics.MaterialAPI.Apply(phys_mat_prim).CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(ground_prim).Bind(
        phys_mat, UsdShade.Tokens.weakerThanDescendants, "physics"
    )

    # ── 3) 조명 ───────────────────────────────────────────────────────────────
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(500.0)
    dome.CreateColorAttr(Gf.Vec3f(0.9, 0.9, 1.0))

    rect = UsdLux.RectLight.Define(stage, "/World/RectLight")
    rect.CreateIntensityAttr(25000.0)
    rect.CreateWidthAttr(14.0)
    rect.CreateHeightAttr(10.0)
    UsdGeom.XformCommonAPI(rect).SetTranslate(Gf.Vec3d(0, 0, 6.0))
    UsdGeom.XformCommonAPI(rect).SetRotate(Gf.Vec3f(-90, 0, 0))

    # ── 4) 트랙 표면 ─────────────────────────────────────────────────────────
    #
    #   ┌────────────── Top ───────────────┐  y = +2.0 ~ +3.5
    #   L                                  R
    #   e  (공터: 7m x 4m)                 i
    #   f                                  g
    #   t                                  h
    #                                      t
    #   └────────────── Bot ───────────────┘  y = -3.5 ~ -2.0
    #
    UsdGeom.Scope.Define(stage, "/World/Track")
    TZ, TH = 0.003, 0.004

    for name, cx, cy, sx, sy in [
        ("Top",   0,            OY - TW/2,  OX*2, TW  ),
        ("Bot",   0,          -(OY - TW/2), OX*2, TW  ),
        ("Left",  -(OX-TW/2), 0,            TW,   IY*2),
        ("Right",  (OX-TW/2), 0,            TW,   IY*2),
    ]:
        _slab(stage, f"/World/Track/{name}", cx, cy, TZ, sx, sy, TH, ASPHALT, mat_cache)

    # ── 5) 차선 마킹 ─────────────────────────────────────────────────────────
    #   각 직선 3선: 외부 흰선 / 황색 중앙선 / 내부 흰선
    UsdGeom.Scope.Define(stage, "/World/Markings")
    MZ, MH  = 0.007, 0.002
    LW      = 0.07
    EDGE    = TW/2 - 0.07   # 트랙 중심에서 경계선까지 오프셋

    # 상/하 직선 (x 방향 선)
    yc = OY - TW/2
    for seg, y_sign in [("Top", +1), ("Bot", -1)]:
        yp = y_sign * yc
        for mname, y_off, color in [
            ("Out", y_sign *  EDGE, WHITE ),
            ("Ctr", 0,              YELLOW),
            ("In",  y_sign * -EDGE, WHITE ),
        ]:
            _slab(stage, f"/World/Markings/{seg}_{mname}",
                  0, yp + y_off, MZ, OX*2, LW, MH, color, mat_cache)

    # 좌/우 직선 (y 방향 선)
    xc = OX - TW/2
    for seg, x_sign in [("Left", -1), ("Right", +1)]:
        xp = x_sign * xc
        for mname, x_off, color in [
            ("Out", x_sign * -EDGE, WHITE ),
            ("Ctr", 0,              YELLOW),
            ("In",  x_sign *  EDGE, WHITE ),
        ]:
            _slab(stage, f"/World/Markings/{seg}_{mname}",
                  xp + x_off, 0, MZ, LW, IY*2, MH, color, mat_cache)

    # ── 6) 벽 (LiDAR 감지 + 충돌) ────────────────────────────────────────────
    UsdGeom.Scope.Define(stage, "/World/Walls")
    WH, WT, WZ = 0.30, 0.10, 0.15

    for name, pos, size in [
        ("OutN", ( 0,      OY,  WZ), (OX*2+WT, WT,       WH)),
        ("OutS", ( 0,     -OY,  WZ), (OX*2+WT, WT,       WH)),
        ("OutW", (-OX,     0,   WZ), (WT,       OY*2+WT, WH)),
        ("OutE", ( OX,     0,   WZ), (WT,       OY*2+WT, WH)),
        ("InN",  ( 0,      IY,  WZ), (IX*2+WT, WT,       WH)),
        ("InS",  ( 0,     -IY,  WZ), (IX*2+WT, WT,       WH)),
        ("InW",  (-IX,     0,   WZ), (WT,       IY*2+WT, WH)),
        ("InE",  ( IX,     0,   WZ), (WT,       IY*2+WT, WH)),
    ]:
        _slab(stage, f"/World/Walls/{name}",
              pos[0], pos[1], pos[2],
              size[0], size[1], size[2],
              WALL_C, mat_cache, physics=True)

    print("[INFO] 트랙 씬 구성 완료")
    print("  크기  : 외부 10m x 7m  /  트랙 폭 1.5m  /  내부 공터 7m x 4m")
    print("  마킹  : 흰색(외·내 경계선) + 황색(중앙선)")
    print("  벽    : 외부 4면 + 내부 4면 (높이 0.3m)")


build_track_scene()
for _ in range(10):
    simulation_app.update()


# ── 로봇 위치: 트랙 상단 직선 중앙에 배치 ────────────────────────────────────
def reposition_robot():
    stage = omni.usd.get_context().get_stage()
    # turtlebot3_burger 자체 xform 수정
    # 원본: translate=(0.06, -1.87, -0.712) — simple_room 바닥 기준으로 z가 지하에 묻혀 있음
    robot_path = "/World/turtlebot_tutorial_multi_sensor_publish_rates/turtlebot3_burger"
    prim = stage.GetPrimAtPath(robot_path)

    if not prim.IsValid():
        print(f"[WARN] 로봇 prim 없음: {robot_path}")
        return

    # ClearXformOpOrder() + AddTranslateOp() 조합은 USD 시각 위치와 물리 엔진 body 위치가
    # 불일치하여 teleop 명령 시 바퀴는 돌지만 로봇이 이동하지 않는 문제를 유발한다.
    # 기존 xformOp:translate 속성을 직접 수정하면 물리 엔진이 동일 속성을 참조하므로
    # 시각·물리 위치가 동기화된 채로 teleport된다.
    translate_attr = prim.GetAttribute("xformOp:translate")
    if translate_attr.IsValid():
        orig = translate_attr.Get()
        # x는 원본 유지(0.06), y=2.75(트랙 상단 직선 중앙), z=0.0(바닥면)
        translate_attr.Set(Gf.Vec3d(orig[0], 2.75, 0.0))
        print(f"[INFO] 로봇 위치 수정: ({orig[0]:.3f}, {orig[1]:.3f}, {orig[2]:.3f}) → ({orig[0]:.3f}, 2.75, 0.0)")
    else:
        # 속성이 없으면 XformOp으로 새로 추가
        xformable = UsdGeom.Xformable(prim)
        op = xformable.AddTranslateOp()
        op.Set(Gf.Vec3d(0.06, 2.75, 0.0))
        print(f"[INFO] 로봇 translate op 추가 → (0.06, 2.75, 0.0)")


# ── 카메라: 트랙 전체가 보이는 3/4 뷰 ───────────────────────────────────────
def setup_camera():
    try:
        import omni.kit.viewport.utility as vu
        viewport = vu.get_active_viewport()
        if not viewport:
            return
        cam_path = viewport.camera_path
        stage    = omni.usd.get_context().get_stage()
        cam_prim = stage.GetPrimAtPath(cam_path)
        if cam_prim.IsValid():
            xf = UsdGeom.XformCommonAPI(cam_prim)
            xf.SetTranslate(Gf.Vec3d(0.0, -14.0, 13.0))  # 남쪽 뒤에서 위로
            xf.SetRotate(Gf.Vec3f(-45.0, 0.0, 0.0))       # 45° 내려봄
            print("[INFO] 카메라 설정 완료: (0, -14, 13) → 트랙 전체 조망")
    except Exception as e:
        print(f"[WARN] 카메라 설정 실패: {e}")


reposition_robot()
setup_camera()
for _ in range(5):
    simulation_app.update()

# ── 시뮬레이션 시작 ───────────────────────────────────────────────────────────
timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("[INFO] 시뮬레이션 시작")
print("[INFO] 발행 토픽: /scan, /imu, /odom, /cmd_vel, /tf")
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
