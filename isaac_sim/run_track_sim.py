"""
TurtleBot3 AutoRace 2020 트랙  —  Isaac Sim Standalone
========================================================
맵    : 6 m × 6 m 아스팔트 바닥판 (시각)
루프  : 3.6 m × 3.3 m 직사각형 클로즈드 루프 (2차선, LANE=0.38 m)
차선  : 흰색 3선 (외측/중앙/내측 모두 백색) 0.02 m @ z=0.005 m

장애물: Cube/Cube_01/Cube_02 — LANE×LANE 정사각형 흰 블록 (Top 직선 슬라롬)
QIV   : 터널/게이트 (Right 직선 하단)

로봇 원점: USD 내 (0.06, -1.87, -0.712) → AddGroundPlaneCommand 가 z=0 배치
실행  : ~/gyusama-project/isaac_sim/launch_sim.sh
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

ASSETS_DIR = os.path.join(os.path.dirname(__file__), "assets")
ROBOT_USD  = os.path.join(ASSETS_DIR, "ros2-turtlebot-robot-only.usd")

print(f"[INFO] 로봇 USD 로드 중: {ROBOT_USD}")
omni.usd.get_context().open_stage(ROBOT_USD)
for _ in range(20):
    simulation_app.update()


# ─── 재료 캐시 ────────────────────────────────────────────────────────────────
_mats: dict = {}


def _mat(stage, key: str, rgb, rough: float = 0.8, emit=None):
    if key in _mats:
        return _mats[key]
    idx  = len(_mats)
    p    = f"/World/Mats/M{idx:03d}"
    mat  = UsdShade.Material.Define(stage, p)
    sh   = UsdShade.Shader.Define(stage, f"{p}/Sh")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor",  Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness",     Sdf.ValueTypeNames.Float  ).Set(rough)
    if emit is not None:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emit))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    _mats[key] = mat
    return mat


def _bind(stage, prim, key: str, rgb, rough: float = 0.8, emit=None):
    _mat(stage, key, rgb, rough, emit)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(_mats[key])


def _box(stage, path: str,
         cx: float, cy: float, cz: float,
         sx: float, sy: float, sz: float,
         mkey: str, rgb,
         rough: float = 0.8, emit=None, phys: bool = False):
    """Cube 프리미티브 (size=1 → scale(sx,sy,sz) → translate(cx,cy,cz))"""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.XformCommonAPI(cube)
    xf.SetTranslate(Gf.Vec3d(cx, cy, cz))
    xf.SetScale(Gf.Vec3f(sx, sy, sz))
    p = cube.GetPrim()
    if phys:
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.RigidBodyAPI.Apply(p).CreateKinematicEnabledAttr(True)
    _bind(stage, p, mkey, rgb, rough, emit)
    return p


def _cyl(stage, path: str,
         cx: float, cy: float, cz: float,
         r: float, h: float,
         mkey: str, rgb,
         rough: float = 0.8, phys: bool = False):
    """Cylinder 프리미티브 (axis=Z, 세로 원통)"""
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(r)
    c.CreateHeightAttr(h)
    c.CreateAxisAttr("Z")
    UsdGeom.XformCommonAPI(c).SetTranslate(Gf.Vec3d(cx, cy, cz))
    p = c.GetPrim()
    if phys:
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.RigidBodyAPI.Apply(p).CreateKinematicEnabledAttr(True)
    _bind(stage, p, mkey, rgb, rough)
    return p


# ─── 씬 구성 ──────────────────────────────────────────────────────────────────
def build_scene():
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Scope.Define(stage, "/World/Mats")

    # ── 색상 (AutoRace 2020)
    ASPHALT = (0.05, 0.05, 0.05)
    WHITE   = (1.00, 1.00, 1.00)
    ORANGE  = (1.00, 0.40, 0.00)
    GRAY    = (0.55, 0.55, 0.55)

    # ── 차선 치수 (spec 준수)
    LANE  = 0.38            # 단일 차선 폭 (spec: 0.35~0.40 m)
    TW    = LANE * 2        # 도로 총 폭 = 0.76 m
    LW    = 0.02            # 차선 표시 폭 (spec: 0.02 m)
    EDGE  = LANE - LW / 2   # 도로 중심→외측 차선 중심 = 0.37 m

    TH    = 0.003            # 트랙 슬랩 두께
    TZ    = TH / 2           # = 0.0015 (z=0 위에 올라앉도록)
    MH    = 0.001            # 차선 마킹 두께  (spec: scale z=0.001)
    MZ    = 0.005            # 차선 마킹 z 중심 (spec: z=0.005)

    # ── 루프 중심선
    BY = -1.80   # 하단(Bottom) y  ← 로봇(y=-1.87) 포함 ✓
    RX =  1.80   # 우측(Right)  x
    TY =  1.50   # 상단(Top)    y
    LX = -1.80   # 좌측(Left)   x
    CY = (BY + TY) / 2      # = -0.15 (수직 세그먼트 중심 y)
    BL = abs(RX - LX)       # = 3.60 m — 하단/상단 길이
    VL = abs(TY - BY)       # = 3.30 m — 우측/좌측 길이

    # ─── 1) Physics Scene + Ground ──────────────────────────────────────────
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)

    omni.kit.commands.execute(
        "AddGroundPlaneCommand",
        stage=stage, planePath="/World/GroundPlane",
        axis="Z", size=30.0,
        position=Gf.Vec3f(0, 0, 0),
        color=Gf.Vec3f(0.20, 0.20, 0.20),
    )

    # ─── 2) 조명 ────────────────────────────────────────────────────────────
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(600.0)

    rect = UsdLux.RectLight.Define(stage, "/World/RectLight")
    rect.CreateIntensityAttr(28000.0)
    rect.CreateWidthAttr(10.0)
    rect.CreateHeightAttr(10.0)
    UsdGeom.XformCommonAPI(rect).SetTranslate(Gf.Vec3d(0, 0, 7))
    UsdGeom.XformCommonAPI(rect).SetRotate(Gf.Vec3f(-90, 0, 0))

    # ─── 3) 5m × 5m 아스팔트 바닥판 (시각용) ──────────────────────────────
    _box(stage, "/World/Ground", 0, 0, -0.005, 6.0, 6.0, 0.01,
         "asphalt", ASPHALT, rough=0.9)

    # ─── 4) 트랙 표면 ───────────────────────────────────────────────────────
    # 루프: Bottom(y=-1.8) → Right(x=1.8) → Top(y=1.5) → Left(x=-1.8)
    # 로봇 y=-1.87 → Bottom y ∈ [-2.18, -1.42] 내 포함 ✓
    UsdGeom.Scope.Define(stage, "/World/Track")

    for nm, cx, cy, sx, sy in [
        ("Bot",   0.0, BY,  BL, TW),
        ("Right", RX,  CY,  TW, VL),
        ("Top",   0.0, TY,  BL, TW),
        ("Left",  LX,  CY,  TW, VL),
    ]:
        _box(stage, f"/World/Track/{nm}", cx, cy, TZ, sx, sy, TH, "track", ASPHALT, rough=0.9)

    for nm, cx, cy in [
        ("CSE", RX, BY), ("CNE", RX, TY),
        ("CNW", LX, TY), ("CSW", LX, BY),
    ]:
        _box(stage, f"/World/Track/{nm}", cx, cy, TZ, TW, TW, TH, "track", ASPHALT, rough=0.9)

    # ─── 5) 2차선 마킹 (spec: z=0.005, width=0.02, emissive) ───────────────
    UsdGeom.Scope.Define(stage, "/World/Marks")
    EW = (0.7, 0.7, 0.7)   # 흰선 약한 emissive

    for nm, cx, cy, sx, sy, col, em in [
        # Bottom — 남측백 / 중앙백 / 북측백
        ("BS", 0.0,       BY - EDGE, BL, LW, WHITE, EW),
        ("BC", 0.0,       BY,        BL, LW, WHITE, EW),
        ("BN", 0.0,       BY + EDGE, BL, LW, WHITE, EW),
        # Right — 동측백 / 중앙백 / 서측백
        ("RE", RX + EDGE, CY,        LW, VL, WHITE, EW),
        ("RC", RX,        CY,        LW, VL, WHITE, EW),
        ("RW", RX - EDGE, CY,        LW, VL, WHITE, EW),
        # Top — 북측백 / 중앙백 / 남측백
        ("TN", 0.0,       TY + EDGE, BL, LW, WHITE, EW),
        ("TC", 0.0,       TY,        BL, LW, WHITE, EW),
        ("TS", 0.0,       TY - EDGE, BL, LW, WHITE, EW),
        # Left — 서측백 / 중앙백 / 동측백
        ("LW", LX - EDGE, CY,        LW, VL, WHITE, EW),
        ("LC", LX,        CY,        LW, VL, WHITE, EW),
        ("LE", LX + EDGE, CY,        LW, VL, WHITE, EW),
    ]:
        _box(stage, f"/World/Marks/{nm}", cx, cy, MZ, sx, sy, MH,
             f"mk_{nm}", col, rough=0.05, emit=em)

    # 시작선 (x=0, 하단 직선)
    _box(stage, "/World/Marks/Start", 0.0, BY, MZ, LW * 4, TW, MH,
         "mk_start", WHITE, rough=0.05, emit=EW)

    # ─── 6) 장애물 큐브 3개 (LANE×LANE 정사각형, Top 직선 슬라롬) ──────────
    # Cube / Cube_01 / Cube_02: 차선 폭과 동일한 0.38×0.38×0.30 m 흰 블록
    UsdGeom.Scope.Define(stage, "/World/Obstacles")
    OBH = 0.30   # 큐브 높이
    for pname, ox, oy in [
        ("Cube",    -0.50, TY + LANE * 0.42),   # 외측(북) 차선
        ("Cube_01", -0.00, TY - LANE * 0.42),   # 내측(남) 차선
        ("Cube_02",  0.50, TY + LANE * 0.42),   # 외측(북) 차선
    ]:
        _box(stage, f"/World/Obstacles/{pname}",
             ox, oy, OBH / 2, LANE, LANE, OBH, "obs_cube", WHITE, phys=True)

    # ─── 7) QIV: 주차장 + 터널 — 하단/우측 우측 (x>0) ───────────────────
    UsdGeom.Scope.Define(stage, "/World/QIV")

    # 터널 (Right 수직 하단, y<0 구간)
    TUN_H  = 0.35    # 터널 내부 높이
    TUN_L  = 0.80    # 터널 길이 (y 방향)
    TUN_T  = 0.03    # 구조물 두께
    TUN_CY = -0.90   # 터널 중심 y

    LWX = RX - LANE - TUN_T / 2   # 좌(서)벽 중심 x = 1.405
    RWX = RX + LANE + TUN_T / 2   # 우(동)벽 중심 x = 2.195
    RFZ = TUN_H + TUN_T / 2       # 지붕 중심 z   = 0.365

    _box(stage, "/World/QIV/TunWL",
         LWX, TUN_CY, (TUN_H + TUN_T) / 2,
         TUN_T, TUN_L, TUN_H + TUN_T, "tun", GRAY, phys=True)
    _box(stage, "/World/QIV/TunWR",
         RWX, TUN_CY, (TUN_H + TUN_T) / 2,
         TUN_T, TUN_L, TUN_H + TUN_T, "tun", GRAY, phys=True)
    _box(stage, "/World/QIV/TunRoof",
         RX, TUN_CY, RFZ,
         TW + 2*TUN_T, TUN_L, TUN_T, "tun", GRAY, phys=True)

    # 터널 입구/출구 프레임 (도로를 막지 않는 기둥+보 구조)
    for suf, gy in [("F", TUN_CY + TUN_L/2), ("B", TUN_CY - TUN_L/2)]:
        _box(stage, f"/World/QIV/TunGate{suf}PL",
             LWX, gy, TUN_H/2, TUN_T, TUN_T*2, TUN_H, "tgate", ORANGE, phys=True)
        _box(stage, f"/World/QIV/TunGate{suf}PR",
             RWX, gy, TUN_H/2, TUN_T, TUN_T*2, TUN_H, "tgate", ORANGE, phys=True)
        _box(stage, f"/World/QIV/TunGate{suf}Top",
             RX,  gy, TUN_H + TUN_T/2,
             TW + 2*TUN_T, TUN_T*2, TUN_T, "tgate", ORANGE)

    print("[INFO] AutoRace 2020 트랙 구성 완료")
    print(f"  루프  : {BL:.1f} m × {VL:.1f} m 직사각형 클로즈드 루프")
    print(f"  도로폭: {TW:.2f} m  (단일 차선 {LANE:.2f} m)")
    print(f"  차선  : 흰색 3선 @ z={MZ} (외측/중앙/내측 모두 백색)")
    print(f"  장애물: Cube/Cube_01/Cube_02 — {LANE:.2f}×{LANE:.2f}×{OBH:.2f} m (Top 직선 슬라롬)")
    print(f"  QIV   : 터널 (y={TUN_CY}±{TUN_L/2:.2f})")
    print(f"  로봇 시작: y=-1.87 → Bot y∈[{BY-LANE:.2f}, {BY+LANE:.2f}] ✓")


build_scene()

for _ in range(10):
    simulation_app.update()


# ── 카메라 초기 시점 ──────────────────────────────────────────────────────────
def setup_camera():
    try:
        import omni.kit.viewport.utility as vu
        vp = vu.get_active_viewport()
        if not vp:
            return
        cp = omni.usd.get_context().get_stage().GetPrimAtPath(vp.camera_path)
        if cp.IsValid():
            xf = UsdGeom.XformCommonAPI(cp)
            xf.SetTranslate(Gf.Vec3d(0.0, -3.0, 9.0))
            xf.SetRotate(Gf.Vec3f(-70.0, 0.0, 0.0))
            print("[INFO] 카메라: (0, -3, 9) @ -70° — 트랙 전체 조망")
    except Exception as e:
        print(f"[WARN] 카메라 설정 실패: {e}")


setup_camera()
for _ in range(5):
    simulation_app.update()

# ── 시뮬레이션 시작 ───────────────────────────────────────────────────────────
timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("[INFO] 시뮬레이션 시작")
print("[INFO] 토픽: /scan /imu /odom /cmd_vel /tf")
print("[INFO] 수동 제어: ros2 run teleop_twist_keyboard teleop_twist_keyboard")
print("[INFO] 종료: Ctrl+C")

try:
    while simulation_app.is_running():
        simulation_app.update()
except KeyboardInterrupt:
    print("\n[INFO] 종료")

timeline.stop()
simulation_app.close()
