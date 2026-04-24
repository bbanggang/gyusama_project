"""
gyusama-project — TurtleBot3 AutoRace 스타일 2차선 트랙

트랙 구조 (TurtleBot3 AutoRace 기반):
  외부: 10m × 5m  /  트랙 폭: 1.0m (2차선, 각 ~0.45m)
  마킹: 흰색 경계선(외·내) + 황색 중앙선 → 카메라 차선 검출용
  건설 구간(Construction Zone): 상단 직선 외측 차선 오렌지 콘 5개 → 차선 변경 유도
  벽 : 외부 4면 + 내부 4면 (LiDAR 감지 + 충돌)
  바닥: AddGroundPlaneCommand (PhysX 무한 평면)

  로봇 원본 위치 (0.06, -1.87, -0.712):
    하단 직선 중심(y=-2.0) 근방 → 재배치 없이 물리 평면이 z=0 으로 자동 올림

실행:
    ~/gyusama-project/isaac_sim/launch_sim.sh
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
import omni.kit.commands
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
    shader.CreateInput("roughness",    Sdf.ValueTypeNames.Float).Set(0.8)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _bind_color(stage, prim, color, mat_cache):
    if color not in mat_cache:
        idx = len(mat_cache)
        mat_cache[color] = _make_material(stage, f"/World/Materials/M{idx}", color)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat_cache[color])


def _slab(stage, path, cx, cy, cz, sx, sy, sz, color, mat_cache, physics=False):
    """직사각형 슬랩 — 트랙 표면·마킹·벽·콘 공용"""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    api = UsdGeom.XformCommonAPI(cube)
    api.SetTranslate(Gf.Vec3d(cx, cy, cz))
    api.SetScale(Gf.Vec3f(sx, sy, sz))
    prim = cube.GetPrim()
    if physics:
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(True)
    _bind_color(stage, prim, color, mat_cache)
    return prim


# ── AutoRace 스타일 트랙 씬 ───────────────────────────────────────────────────
def build_track_scene():
    stage     = omni.usd.get_context().get_stage()
    mat_cache = {}
    UsdGeom.Scope.Define(stage, "/World/Materials")

    # ── 색상 (카메라 차선 검출 대비 최대화)
    ASPHALT = (0.10, 0.10, 0.10)   # 짙은 아스팔트
    WHITE   = (1.00, 1.00, 1.00)   # 경계선
    YELLOW  = (0.95, 0.85, 0.00)   # 중앙 분리선
    ORANGE  = (1.00, 0.38, 0.00)   # 건설 구간 콘
    WALL_C  = (0.55, 0.55, 0.55)   # 외/내 벽

    # ── 트랙 치수
    # 로봇 원본 y = -1.87 → 하단 직선(y: -2.5 ~ -1.5) 내에 위치
    OX  = 5.0    # 외부 반폭 x  → 전체 10m
    OY  = 2.5    # 외부 반폭 y  → 전체  5m
    TW  = 1.0    # 트랙 폭 (2차선 합계)
    IX  = OX - TW          # 내부 반폭 x = 4.0
    IY  = OY - TW          # 내부 반폭 y = 1.5
    LW  = 0.05             # 차선 마킹 폭 (5 cm — AutoRace 테이프 너비)
    EDGE_OFF = TW/2 - LW/2 # 트랙 중심 → 경계선 중심 오프셋 = 0.475 m

    # 각 직선 중심 좌표
    BOT_CY = -(OY - TW/2)   # 하단 y = -2.0  (로봇 y=-1.87 ✓)
    TOP_CY =  (OY - TW/2)   # 상단 y = +2.0
    LFT_CX = -(OX - TW/2)   # 좌측 x = -4.5
    RGT_CX =  (OX - TW/2)   # 우측 x = +4.5

    # ── 1) Physics Scene
    phys = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    phys.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    phys.CreateGravityMagnitudeAttr(9.81)

    # ── 2) 바닥 — AddGroundPlaneCommand (PhysX 무한 평면)
    # 커스텀 박스 슬랩은 로봇(z=-0.712)이 슬랩 안에 박혀 물리 폭발을 유발함
    # 무한 평면은 로봇을 자동으로 z=0 위로 올려주므로 안전
    omni.kit.commands.execute(
        "AddGroundPlaneCommand",
        stage=stage, planePath="/World/GroundPlane",
        axis="Z", size=30.0,
        position=Gf.Vec3f(0.0, 0.0, 0.0),
        color=Gf.Vec3f(0.22, 0.22, 0.22),
    )

    # ── 3) 조명
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(600.0)
    dome.CreateColorAttr(Gf.Vec3f(0.95, 0.95, 1.0))

    rect = UsdLux.RectLight.Define(stage, "/World/RectLight")
    rect.CreateIntensityAttr(30000.0)
    rect.CreateWidthAttr(12.0)
    rect.CreateHeightAttr(8.0)
    UsdGeom.XformCommonAPI(rect).SetTranslate(Gf.Vec3d(0, 0, 6.0))
    UsdGeom.XformCommonAPI(rect).SetRotate(Gf.Vec3f(-90, 0, 0))

    # ── 4) 트랙 표면 — 아스팔트 (시각용, 물리 없음)
    #
    #  OutN ─────────────────────────── OutN
    #  │  ══════ Top ══════             │
    #  │  ─ ─ ─ ─ ─ ─ ─ ─ ─ ─  (중앙선)│
    #  │  ══════════════════════        │
    #  InN ────────────────────── InN  │
    #  ║  (공터 8m×3m)            ║    │
    #  InS ────────────────────── InS  │
    #  │  ══════════════════════        │
    #  │  ─ ─ ─ ─ ─ ─ ─ ─ ─ ─  (중앙선)│
    #  │  ══════ Bot ══════  ← 로봇 시작│
    #  OutS ─────────────────────────── OutS
    #
    UsdGeom.Scope.Define(stage, "/World/Track")
    TZ, TH = 0.005, 0.003  # 시각 슬랩 z 중심 / 두께

    for name, cx, cy, sx, sy in [
        ("Top",   0,      TOP_CY, OX*2, TW  ),
        ("Bot",   0,      BOT_CY, OX*2, TW  ),
        ("Left",  LFT_CX, 0,      TW,   IY*2),
        ("Right", RGT_CX, 0,      TW,   IY*2),
    ]:
        _slab(stage, f"/World/Track/{name}", cx, cy, TZ, sx, sy, TH, ASPHALT, mat_cache)

    # ── 5) 2차선 마킹 (시각용, 물리 없음)
    # 구조: [외측 흰선 5cm] [외측 차선 ~0.45m] [황색 중앙선 5cm] [내측 차선 ~0.45m] [내측 흰선 5cm]
    UsdGeom.Scope.Define(stage, "/World/Markings")
    MZ, MH = 0.009, 0.002

    # 상/하 직선 마킹 (x 방향 선)
    for seg, cy, y_sign in [("Top", TOP_CY, +1), ("Bot", BOT_CY, -1)]:
        for mname, y_off, color in [
            ("Out",  y_sign * EDGE_OFF,  WHITE ),
            ("Ctr",  0.0,                YELLOW),
            ("In",   y_sign * -EDGE_OFF, WHITE ),
        ]:
            _slab(stage, f"/World/Markings/{seg}_{mname}",
                  0, cy + y_off, MZ, OX*2, LW, MH, color, mat_cache)

    # 좌/우 직선 마킹 (y 방향 선)
    for seg, cx, x_sign in [("Left", LFT_CX, -1), ("Right", RGT_CX, +1)]:
        for mname, x_off, color in [
            ("Out",  x_sign * EDGE_OFF,  WHITE ),
            ("Ctr",  0.0,                YELLOW),
            ("In",   x_sign * -EDGE_OFF, WHITE ),
        ]:
            _slab(stage, f"/World/Markings/{seg}_{mname}",
                  cx + x_off, 0, MZ, LW, IY*2, MH, color, mat_cache)

    # ── 6) 시작선 (Start/Finish) — 하단 직선 x=0 에 흰색 가로선
    _slab(stage, "/World/Markings/StartLine",
          0, BOT_CY, MZ, LW*2, TW, MH, WHITE, mat_cache)

    # ── 7) 벽 (LiDAR 감지 + 충돌)
    UsdGeom.Scope.Define(stage, "/World/Walls")
    WH, WT, WZ = 0.30, 0.10, 0.15   # 높이 30cm, 두께 10cm, z 중심 15cm

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
              pos[0], pos[1], pos[2], size[0], size[1], size[2],
              WALL_C, mat_cache, physics=True)

    # ── 8) 건설 구간 (Construction Zone) ─────────────────────────────────────
    # 상단 직선(y≈+2.0)을 로봇이 -x 방향으로 주행할 때
    # 외측 차선(y > 2.0, y≈2.25) 에 오렌지 콘 5개 배치
    #
    #  외벽(y=+2.5) ─────────────────────────────
    #  외측 차선 [🔶콘] [🔶콘] [🔶콘] [🔶콘] [🔶콘]  ← 로봇이 이 차선으로 오면 회피 필요
    #  중앙선(y=+2.0) ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
    #  내측 차선                                      ← 로봇이 이 차선으로 변경하여 통과
    #  내벽(y=+1.5) ─────────────────────────────
    #
    UsdGeom.Scope.Define(stage, "/World/Construction")
    CONE_W  = 0.12   # 콘 단면 폭 (LiDAR 감지 가능한 최소 크기)
    CONE_H  = 0.25   # 콘 높이
    CONE_CZ = CONE_H / 2   # z 중심 (바닥 기준)

    # 외측 차선 중심: TOP_CY + TW/4 = 2.0 + 0.25 = 2.25
    OUTER_Y = TOP_CY + TW / 4

    for i, x_pos in enumerate([-2.0, -1.0, 0.0, 1.0, 2.0]):
        _slab(stage, f"/World/Construction/Cone{i}",
              x_pos, OUTER_Y, CONE_CZ,
              CONE_W, CONE_W, CONE_H,
              ORANGE, mat_cache, physics=True)

    # 건설 구간 진입/퇴출 안내선 (중앙선 위 오렌지 마커)
    for i, x_pos in enumerate([-2.5, 2.5]):
        _slab(stage, f"/World/Construction/Marker{i}",
              x_pos, TOP_CY, MZ, LW*3, TW, MH, ORANGE, mat_cache)

    print("[INFO] AutoRace 스타일 2차선 트랙 구성 완료")
    print(f"  외부    : {OX*2:.0f}m × {OY*2:.0f}m  /  트랙 폭: {TW:.1f}m  /  차선: 2 × ~0.45m")
    print(f"  하단 직선: y = {-(OY):.1f} ~ {-(OY-TW):.1f}  (로봇 원본 y≈-1.87 포함 ✓)")
    print(f"  건설 구간: 상단 직선 외측 차선 (y≈{OUTER_Y:.2f}) 콘 5개")
    print(f"  차선 마킹: 흰색 {LW*100:.0f}cm + 황색 {LW*100:.0f}cm (카메라 검출용)")


build_track_scene()
for _ in range(10):
    simulation_app.update()


# ── 카메라: 트랙 전체 조망 ────────────────────────────────────────────────────
def setup_camera():
    try:
        import omni.kit.viewport.utility as vu
        viewport = vu.get_active_viewport()
        if not viewport:
            return
        stage    = omni.usd.get_context().get_stage()
        cam_prim = stage.GetPrimAtPath(viewport.camera_path)
        if cam_prim.IsValid():
            xf = UsdGeom.XformCommonAPI(cam_prim)
            xf.SetTranslate(Gf.Vec3d(0.0, -10.0, 10.0))
            xf.SetRotate(Gf.Vec3f(-45.0, 0.0, 0.0))
            print("[INFO] 카메라: (0, -10, 10) → 트랙 전체 조망")
    except Exception as e:
        print(f"[WARN] 카메라 설정 실패: {e}")


setup_camera()
for _ in range(5):
    simulation_app.update()

# ── 시뮬레이션 시작 ───────────────────────────────────────────────────────────
timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("[INFO] 시뮬레이션 시작")
print("[INFO] 토픽: /scan, /imu, /odom, /cmd_vel, /tf")
print("[INFO] 수동 제어: ros2 run teleop_twist_keyboard teleop_twist_keyboard")
print("[INFO] 제어 시나리오: python3 ~/gyusama-project/isaac_sim/control_scenario.py")
print("[INFO] 종료: Ctrl+C")

try:
    while simulation_app.is_running():
        simulation_app.update()
except KeyboardInterrupt:
    print("\n[INFO] 종료")

timeline.stop()
simulation_app.close()
