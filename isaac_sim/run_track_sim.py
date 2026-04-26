"""
TurtleBot3 AutoRace 2020 트랙  —  Isaac Sim Standalone
========================================================
맵    : 5 m × 5 m 아스팔트 바닥판
루프  : 3.6 m × 3.3 m 직사각형 클로즈드 루프 (2차선, LANE=0.38 m)
차선  : 외측 흰선 0.02 m + 황색 중앙선 0.02 m @ z=0.005 m

미션 구역 (4사분면):
  QI  (x>0, y>0) : S자 주황 콘 3개 + 신호등
  QII (x<0, y>0) : 장애물 회피 — 흰 원통 3개 (슬라롬)
  QIII(x<0, y<0) : 공사 구역 — 교통봉(빨/흰) 2개 + 흰 큐브 벽 3개
  QIV (x>0, y<0) : 주차장(점선 2칸) + 터널/게이트

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
    YELLOW  = (1.00, 0.85, 0.00)   # spec: RGB(1.0, 0.85, 0.0)
    RED     = (0.85, 0.10, 0.10)
    ORANGE  = (1.00, 0.40, 0.00)
    GRAY    = (0.55, 0.55, 0.55)
    GREEN   = (0.10, 0.72, 0.10)

    # ── 차선 치수 (spec 준수)
    LANE  = 0.38            # 단일 차선 폭 (spec: 0.35~0.40 m)
    TW    = LANE * 2        # 도로 총 폭 = 0.76 m
    LW    = 0.02            # 차선 표시 폭 (spec: 0.02 m)
    EDGE  = LANE - LW / 2   # 도로 중심→외측 차선 중심 = 0.37 m

    TH    = 0.003            # 트랙 슬랩 두께
    TZ    = TH / 2           # = 0.0015 (z=0 위에 올라앉도록)
    MH    = 0.001            # 차선 마킹 두께  (spec: scale z=0.001)
    MZ    = 0.005            # 차선 마킹 z 중심 (spec: z=0.005)

    WH    = 0.25             # 외벽 높이
    WT    = 0.05             # 외벽 두께

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
        # Bottom — 남측백 / 황중앙 / 북측백
        ("BS", 0.0,       BY - EDGE, BL, LW, WHITE,  EW  ),
        ("BC", 0.0,       BY,        BL, LW, YELLOW, None),
        ("BN", 0.0,       BY + EDGE, BL, LW, WHITE,  EW  ),
        # Right — 동측백 / 황중앙 / 서측백
        ("RE", RX + EDGE, CY,        LW, VL, WHITE,  EW  ),
        ("RC", RX,        CY,        LW, VL, YELLOW, None),
        ("RW", RX - EDGE, CY,        LW, VL, WHITE,  EW  ),
        # Top — 북측백 / 황중앙 / 남측백
        ("TN", 0.0,       TY + EDGE, BL, LW, WHITE,  EW  ),
        ("TC", 0.0,       TY,        BL, LW, YELLOW, None),
        ("TS", 0.0,       TY - EDGE, BL, LW, WHITE,  EW  ),
        # Left — 서측백 / 황중앙 / 동측백
        ("LW", LX - EDGE, CY,        LW, VL, WHITE,  EW  ),
        ("LC", LX,        CY,        LW, VL, YELLOW, None),
        ("LE", LX + EDGE, CY,        LW, VL, WHITE,  EW  ),
    ]:
        _box(stage, f"/World/Marks/{nm}", cx, cy, MZ, sx, sy, MH,
             f"mk_{nm}", col, rough=0.05, emit=em)

    # 시작선 (x=0, 하단 직선)
    _box(stage, "/World/Marks/Start", 0.0, BY, MZ, LW * 4, TW, MH,
         "mk_start", WHITE, rough=0.05, emit=EW)

    # ─── 6) QII: 장애물 회피 — 상단 좌측 (x<0, y≈TY) ─────────────────────
    # 흰색 원통 3개: Top 직선 x<0 구간에 슬라롬 배치
    UsdGeom.Scope.Define(stage, "/World/QII")
    for i, (ox, oy) in enumerate([
        (-0.50, TY + LANE * 0.42),   # 외측(북) 차선
        (-0.90, TY - LANE * 0.42),   # 내측(남) 차선
        (-1.30, TY + LANE * 0.42),   # 외측(북) 차선
    ]):
        _cyl(stage, f"/World/QII/Obs{i}",
             ox, oy, 0.125, 0.05, 0.25, "obs_w", WHITE, rough=0.4, phys=True)

    # ─── 8) QI: S자 콘 + 신호등 — 상단 우측 (x>0, y≈TY) ──────────────────
    UsdGeom.Scope.Define(stage, "/World/QI")

    # 주황 가이드 콘 3개: Top 직선 x>0 구간 슬라롬
    for i, (cx, cy) in enumerate([
        (0.40, TY + LANE * 0.42),
        (0.80, TY - LANE * 0.42),
        (1.20, TY + LANE * 0.42),
    ]):
        _cyl(stage, f"/World/QI/Cone{i}",
             cx, cy, 0.10, 0.04, 0.20, "cone_or", ORANGE, phys=True)

    # 신호등 — 동 외벽 바깥, QI 영역
    TLX = RX + EO + 0.05
    TLY = TY - 0.50
    _box(stage, "/World/QI/TLPole",   TLX, TLY, 0.25, 0.05, 0.05, 0.50, "tl_p", WHITE,  phys=True)
    _cyl(stage, "/World/QI/TLHead",   TLX, TLY, 0.57, 0.09, 0.16, "tl_h", GRAY )
    _cyl(stage, "/World/QI/TLRed",    TLX, TLY, 0.63, 0.04, 0.05, "tl_r", RED,   rough=0.2)
    _cyl(stage, "/World/QI/TLGreen",  TLX, TLY, 0.54, 0.04, 0.05, "tl_g", GREEN, rough=0.2)

    # ─── 9) QIII: 공사 구역 — 하단 좌측 (x<0, y≈BY) ──────────────────────
    UsdGeom.Scope.Define(stage, "/World/QIII")

    # 교통봉 (빨/흰): Bot 직선 x<0 구간 지그재그
    for i, (bx, by) in enumerate([
        (-0.55, BY - LANE * 0.40),   # 외측(남) 차선
        (-1.05, BY + LANE * 0.40),   # 내측(북) 차선
    ]):
        _cyl(stage, f"/World/QIII/BarR{i}", bx, by, 0.14,  0.025, 0.28, "bar_r", RED,   phys=True)
        _cyl(stage, f"/World/QIII/BarW{i}", bx, by, 0.265, 0.027, 0.07, "bar_w", WHITE)

    # ─── 10) QIV: 주차장 + 터널 — 하단/우측 우측 (x>0) ───────────────────
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
    print(f"  차선  : 외측 백선 {LW:.2f} m + 황색 중앙선 {LW:.2f} m @ z={MZ}")
    print(f"  QI  (우상): S자 주황 콘 3개 + 신호등")
    print(f"  QII (좌상): 장애물 흰 원통 3개 (슬라롬)")
    print(f"  QIII(좌하): 교통봉 2개")
    print(f"  QIV (우하): 터널 (y={TUN_CY}±{TUN_L/2:.2f})")
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
