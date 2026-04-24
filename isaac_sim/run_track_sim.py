"""
gyusama-project — TurtleBot3 AutoRace 스타일 복잡 루프 트랙

Gazebo AutoRace 트랙을 참고한 불규칙 루프 구조:
  하단 직선(8m) → 우측(3m) → 상단 우측(2.5m) → 내부 우측 수직(2m)
  → 상단 중앙(3m, 건설 구간) → 내부 좌측 수직(2m) → 상단 좌측(2.5m)
  → 좌측(3m) → 하단으로 복귀

  내부 공터: H 자 형태 (로봇이 H 외곽 트랙 위를 달림)
  2차선: 황색 중앙선 + 흰색 경계선 (카메라 차선 검출용)
  건설 구간(Construction Zone): 상단 중앙 외측 차선 오렌지 콘 3개

  로봇 시작: 하단 직선 (y≈-1.87, AddGroundPlaneCommand 가 z=0 자동 배치)

실행: ~/gyusama-project/isaac_sim/launch_sim.sh
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
    """직사각형 슬랩 공용 함수 (트랙·마킹·벽·콘)"""
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

    # ── 색상 (카메라 대비 최대화)
    ASPHALT = (0.09, 0.09, 0.09)
    WHITE   = (1.00, 1.00, 1.00)
    YELLOW  = (0.95, 0.85, 0.00)
    ORANGE  = (1.00, 0.38, 0.00)
    WALL_C  = (0.50, 0.50, 0.50)

    # ── 핵심 치수
    TW      = 1.0          # 트랙 폭 (2차선)
    LW      = 0.05         # 마킹 라인 폭
    EDGE    = TW/2 - LW/2  # 트랙 중심 → 경계선 중심 오프셋 = 0.475
    TZ, TH  = 0.005, 0.003 # 트랙 표면 z 중심 / 두께
    MZ, MH  = 0.009, 0.002 # 마킹 z 중심 / 두께
    WH, WT, WZ = 0.30, 0.10, 0.15  # 벽 높이 / 두께 / z 중심

    # ── 트랙 세그먼트 정의 ────────────────────────────────────────────────────
    #
    # AutoRace 참고 복잡 루프 (로봇 원본 위치 y≈-1.87 → 하단 직선 포함):
    #
    #   OutW    OutTopL      OutN       OutTopR   OutE
    #    │  ┌──────────┐  ┌──────┐  ┌──────────┐  │
    #    │  │ ULH      │  │ Top  │  │ URH      │  │
    #    │  │ (2.5m)   │  │(3m)  │  │ (2.5m)   │  │
    #    │  └──┐  InMidW│  │InN  │ InMidE┌──┘  │
    #    │Left │  ILV   │  │     │   IRV │     │ Right
    #    │(3m) │  (2m)  │  │     │  (2m) │     │ (3m)
    #    │  ┌──┘  InTopW│  │InS  │ InTopE└──┐  │
    #    │  │          InW   InE          │  │
    #    │  └─────────────────────────────┘  │
    #    │           InS (7m)                 │
    #    │  ══════════════ Bot (8m) ═══════   │  ← 로봇(y≈-1.87)
    #   OutS ─────────────────────────────── OutS
    #
    # 세그먼트 중심 좌표 (cx, cy) 및 크기 (sx, sy):
    #   Bot    : (0,    -2.0), sx=9.0, sy=TW   ← 하단(로봇 시작)
    #   Right  : (4.0,  -0.5), sx=TW,  sy=4.0  ← 우측
    #   URH    : (2.75,  1.0), sx=2.5, sy=TW   ← 상단 우측 수평
    #   IRV    : (1.5,   2.0), sx=TW,  sy=2.0  ← 내부 우측 수직
    #   Top    : (0,     3.0), sx=3.0, sy=TW   ← 상단 중앙 (건설 구간)
    #   ILV    : (-1.5,  2.0), sx=TW,  sy=2.0  ← 내부 좌측 수직
    #   ULH    : (-2.75, 1.0), sx=2.5, sy=TW   ← 상단 좌측 수평
    #   Left   : (-4.0, -0.5), sx=TW,  sy=4.0  ← 좌측

    # ── 1) Physics Scene
    phys = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    phys.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    phys.CreateGravityMagnitudeAttr(9.81)

    # ── 2) 바닥 — AddGroundPlaneCommand (PhysX 무한 평면)
    omni.kit.commands.execute(
        "AddGroundPlaneCommand",
        stage=stage, planePath="/World/GroundPlane",
        axis="Z", size=30.0,
        position=Gf.Vec3f(0.0, 0.0, 0.0),
        color=Gf.Vec3f(0.20, 0.20, 0.20),
    )

    # ── 3) 조명
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(700.0)
    dome.CreateColorAttr(Gf.Vec3f(0.95, 0.95, 1.0))

    rect = UsdLux.RectLight.Define(stage, "/World/RectLight")
    rect.CreateIntensityAttr(35000.0)
    rect.CreateWidthAttr(14.0)
    rect.CreateHeightAttr(10.0)
    UsdGeom.XformCommonAPI(rect).SetTranslate(Gf.Vec3d(0, 0.5, 7.0))
    UsdGeom.XformCommonAPI(rect).SetRotate(Gf.Vec3f(-90, 0, 0))

    # ── 4) 트랙 표면 (시각용, 물리 없음) ─────────────────────────────────────
    UsdGeom.Scope.Define(stage, "/World/Track")

    segs = [
        ("Bot",   0.0,   -2.0, 9.0, TW ),
        ("Right", 4.0,   -0.5, TW,  4.0),
        ("URH",   2.75,   1.0, 2.5, TW ),
        ("IRV",   1.5,    2.0, TW,  2.0),
        ("Top",   0.0,    3.0, 3.0, TW ),
        ("ILV",  -1.5,    2.0, TW,  2.0),
        ("ULH",  -2.75,   1.0, 2.5, TW ),
        ("Left", -4.0,   -0.5, TW,  4.0),
    ]
    for name, cx, cy, sx, sy in segs:
        _slab(stage, f"/World/Track/{name}", cx, cy, TZ, sx, sy, TH, ASPHALT, mat_cache)

    # 코너 패치 (방향 전환 지점 이음새 채우기)
    corners = [
        ("CornerSE",  4.0,  -2.0),
        ("CornerE",   4.0,   1.0),
        ("CornerURH", 1.5,   1.0),
        ("CornerNE",  1.5,   3.0),
        ("CornerNW", -1.5,   3.0),
        ("CornerULH",-1.5,   1.0),
        ("CornerW",  -4.0,   1.0),
        ("CornerSW", -4.0,  -2.0),
    ]
    for name, cx, cy in corners:
        _slab(stage, f"/World/Track/{name}", cx, cy, TZ, TW, TW, TH, ASPHALT, mat_cache)

    # ── 5) 2차선 마킹 (시각용, 물리 없음) ─────────────────────────────────────
    # 구조: 외측흰선(5cm) │ 외측 차선(~0.45m) │ 황색중앙선(5cm) │ 내측 차선(~0.45m) │ 내측흰선(5cm)
    UsdGeom.Scope.Define(stage, "/World/Markings")

    # (prim_name, cx, cy, sx, sy, color)
    marks = [
        # ── 하단 직선 (Bot): outer=-y, inner=+y
        ("Bot_Out", 0.0, -2.0-EDGE, 9.0, LW, WHITE ),
        ("Bot_Ctr", 0.0, -2.0,      9.0, LW, YELLOW),
        ("Bot_In",  0.0, -2.0+EDGE, 9.0, LW, WHITE ),
        # ── 우측 수직 (Right): outer=+x, inner=-x
        ("Rgt_Out",  4.0+EDGE, -0.5, LW, 4.0, WHITE ),
        ("Rgt_Ctr",  4.0,      -0.5, LW, 4.0, YELLOW),
        ("Rgt_In",   4.0-EDGE, -0.5, LW, 4.0, WHITE ),
        # ── 상단 우측 수평 (URH): outer=+y, inner=-y
        ("URH_Out",  2.75,  1.0+EDGE, 2.5, LW, WHITE ),
        ("URH_Ctr",  2.75,  1.0,      2.5, LW, YELLOW),
        ("URH_In",   2.75,  1.0-EDGE, 2.5, LW, WHITE ),
        # ── 내부 우측 수직 (IRV): outer=-x(상단 트랙 방향), inner=+x(H 공터 방향)
        ("IRV_Out",  1.5-EDGE,  2.0, LW, 2.0, WHITE ),
        ("IRV_Ctr",  1.5,       2.0, LW, 2.0, YELLOW),
        ("IRV_In",   1.5+EDGE,  2.0, LW, 2.0, WHITE ),
        # ── 상단 중앙 (Top): outer=+y, inner=-y
        ("Top_Out",  0.0,  3.0+EDGE, 3.0, LW, WHITE ),
        ("Top_Ctr",  0.0,  3.0,      3.0, LW, YELLOW),
        ("Top_In",   0.0,  3.0-EDGE, 3.0, LW, WHITE ),
        # ── 내부 좌측 수직 (ILV): outer=+x, inner=-x
        ("ILV_Out", -1.5+EDGE,  2.0, LW, 2.0, WHITE ),
        ("ILV_Ctr", -1.5,       2.0, LW, 2.0, YELLOW),
        ("ILV_In",  -1.5-EDGE,  2.0, LW, 2.0, WHITE ),
        # ── 상단 좌측 수평 (ULH): outer=+y, inner=-y
        ("ULH_Out", -2.75,  1.0+EDGE, 2.5, LW, WHITE ),
        ("ULH_Ctr", -2.75,  1.0,      2.5, LW, YELLOW),
        ("ULH_In",  -2.75,  1.0-EDGE, 2.5, LW, WHITE ),
        # ── 좌측 수직 (Left): outer=-x, inner=+x
        ("Lft_Out", -4.0-EDGE, -0.5, LW, 4.0, WHITE ),
        ("Lft_Ctr", -4.0,      -0.5, LW, 4.0, YELLOW),
        ("Lft_In",  -4.0+EDGE, -0.5, LW, 4.0, WHITE ),
    ]
    for name, cx, cy, sx, sy, col in marks:
        _slab(stage, f"/World/Markings/{name}", cx, cy, MZ, sx, sy, MH, col, mat_cache)

    # 시작/종료선 — 하단 직선 x=0 에 흰색 가로선
    _slab(stage, "/World/Markings/StartLine", 0.0, -2.0, MZ, LW*3, TW, MH, WHITE, mat_cache)

    # ── 6) 벽 — 외부 경계 + 내부 H 공터 경계 (LiDAR 감지 + 충돌) ────────────
    UsdGeom.Scope.Define(stage, "/World/Walls")

    # 외벽 8개 (트랙 외측 경계)
    outer_walls = [
        ("OutS",    0.0,  -2.5,  9.2,  WT ),  # 남 (하단 외곽)
        ("OutE",    4.5,  -0.5,  WT,   4.2),  # 동 (우측 외곽)
        ("OutTopE", 3.25,  1.5,  2.6,  WT ),  # 상단-우 수평 외곽
        ("OutMidE", 2.0,   2.5,  WT,   2.2),  # 상단-우 수직 외곽
        ("OutN",    0.0,   3.5,  4.2,  WT ),  # 북 (상단 외곽)
        ("OutMidW",-2.0,   2.5,  WT,   2.2),  # 상단-좌 수직 외곽
        ("OutTopW",-3.25,  1.5,  2.6,  WT ),  # 상단-좌 수평 외곽
        ("OutW",   -4.5,  -0.5,  WT,   4.2),  # 서 (좌측 외곽)
    ]
    # 내벽 8개 (H자 공터 경계)
    inner_walls = [
        ("InS",     0.0,  -1.5,  7.2,  WT ),  # 남 (하단 내측)
        ("InE",     3.5,  -0.5,  WT,   2.2),  # 동 (우측 내측)
        ("InTopE",  2.75,  0.5,  1.6,  WT ),  # 상단-우 수평 내측
        ("InMidE",  2.0,   1.5,  WT,   2.2),  # 상단-우 수직 내측
        ("InN",     0.0,   2.5,  4.2,  WT ),  # 북 (상단 내측)
        ("InMidW", -2.0,   1.5,  WT,   2.2),  # 상단-좌 수직 내측
        ("InTopW", -2.75,  0.5,  1.6,  WT ),  # 상단-좌 수평 내측
        ("InW",    -3.5,  -0.5,  WT,   2.2),  # 서 (좌측 내측)
    ]
    for name, cx, cy, sx, sy in outer_walls + inner_walls:
        _slab(stage, f"/World/Walls/{name}", cx, cy, WZ, sx, sy, WH, WALL_C, mat_cache, physics=True)

    # ── 7) 건설 구간 (Construction Zone) ─────────────────────────────────────
    # 상단 중앙(Top, y=3.0)을 로봇이 -x 방향 주행 시
    # 외측 차선(y > 3.0, y≈3.25)에 오렌지 콘 3개 배치 → 내측 차선으로 변경 유도
    #
    #  외벽(y=3.5)  ─────────────────────────
    #  외측 차선  [🔶 -1.0] [🔶 0.0] [🔶 1.0]  ← 로봇이 이 차선에 접근 시 회피
    #  중앙선(y=3.0) ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
    #  내측 차선                               ← 로봇이 이 차선으로 변경하여 통과
    #  내벽(y=2.5)  ─────────────────────────
    #
    UsdGeom.Scope.Define(stage, "/World/Construction")
    CONE_W, CONE_H = 0.12, 0.25
    CONE_CZ = CONE_H / 2
    OUTER_Y = 3.0 + TW / 4   # 외측 차선 중심 y = 3.25

    for i, xp in enumerate([-1.0, 0.0, 1.0]):
        _slab(stage, f"/World/Construction/Cone{i}",
              xp, OUTER_Y, CONE_CZ, CONE_W, CONE_W, CONE_H, ORANGE, mat_cache, physics=True)

    # 건설 구간 진입/퇴출 오렌지 마커 (세로선)
    for i, xp in enumerate([-1.5, 1.5]):
        _slab(stage, f"/World/Construction/Marker{i}",
              xp, 3.0, MZ, LW*2, TW, MH, ORANGE, mat_cache)

    print("[INFO] AutoRace 복잡 루프 트랙 구성 완료")
    print("  세그먼트 : 하단(8m) + 우측(3m) + 상단우(2.5m) + 내부우(2m)")
    print("            + 상단중(3m) + 내부좌(2m) + 상단좌(2.5m) + 좌측(3m)")
    print("  내부 공터: H자형 (좌우 2개 + 중앙 공간)")
    print("  2차선    : 황색 중앙선 + 흰색 경계선 (카메라 검출용)")
    print("  건설 구간: 상단 중앙 외측 차선 (y≈3.25) 콘 3개")
    print(f"  로봇 시작: 하단 직선 (y≈-1.87 → 하단 직선 y:-2.5~-1.5 내 포함 ✓)")


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
            xf.SetTranslate(Gf.Vec3d(0.0, -5.0, 14.0))
            xf.SetRotate(Gf.Vec3f(-55.0, 0.0, 0.0))
            print("[INFO] 카메라: (0, -5, 14) → 트랙 전체 조망")
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