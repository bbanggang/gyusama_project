"""
TurtleBot3 AutoRace 2020 트랙 — Isaac Sim Standalone (v2)
=========================================================
소스  : turtlebot3_simulations/turtlebot3_autorace_2020.world
좌표계: Gazebo world 파일과 동일
트랙  : 직사각형 루프 (도로 중심 ±1.65 m, 폭 0.55 m) + Gazebo 인라인 요소
차선  : 외측·내측 흰색 실선, 중앙 실선(기본) / 점선(공사구간 ↔ 동쪽 y=0.4~1.6)
인라인:
  tunnel_wall  4개 (world 파일 <state> 절대좌표 그대로)
  tunnel_obstacle 3개 실린더 (슬라롬)
  barrier_1/2/3  3개 박스 (공사구간)
"""
import os
import sys
import math
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

print(f"[INFO] 로봇 USD: {ROBOT_USD}")
omni.usd.get_context().open_stage(ROBOT_USD)
for _ in range(20):
    simulation_app.update()


# ─── 재료 캐시 ────────────────────────────────────────────────────────────────
_mats: dict = {}


def _mat(stage, key: str, rgb, rough: float = 0.8, emit=None):
    if key in _mats:
        return _mats[key]
    p   = f"/World/Mats/M{len(_mats):03d}"
    mat = UsdShade.Material.Define(stage, p)
    sh  = UsdShade.Shader.Define(stage, f"{p}/Sh")
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
         rough: float = 0.8, emit=None, phys: bool = False, rz=None):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.XformCommonAPI(cube)
    xf.SetTranslate(Gf.Vec3d(cx, cy, cz))
    if rz is not None:
        xf.SetRotate(Gf.Vec3f(0.0, 0.0, float(rz)))
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

    for _p in ["/World/Cube", "/World/Cube_01", "/World/Cube_02"]:
        if stage.GetPrimAtPath(_p).IsValid():
            stage.RemovePrim(_p)

    UsdGeom.Scope.Define(stage, "/World/Mats")

    # ── 색상
    ASPHALT = (0.06, 0.06, 0.06)
    WHITE   = (1.00, 1.00, 1.00)
    GRAY    = (0.50, 0.50, 0.50)
    EW      = (0.70, 0.70, 0.70)   # 차선 emissive

    # ── 두께/z
    TH, TZ = 0.003, 0.0015
    MH, MZ = 0.001, 0.005

    # ── AutoRace 2020 트랙 치수 (Gazebo world 좌표계 일치)
    # 도로 중심선 좌표 (traffic 표지판 위치에서 역산)
    #   traffic_light  (1.30, -1.95) → 외곽선 y = -1.925 (0.025 m 바깥)
    #   traffic_intersection (1.95, -1.0) → 외곽선 x = +1.925
    #   traffic_parking (0.74, +1.95) → 외곽선 y = +1.925
    BY = -1.65   # Bottom(남쪽) 도로 중심 y
    RX = +1.65   # Right(동쪽)  도로 중심 x
    TY = +1.65   # Top(북쪽)    도로 중심 y
    LX = -1.65   # Left(서쪽)   도로 중심 x
    CY =  0.00   # y 중심

    TW   = 0.55          # 도로 총 폭 (Autorace 2020 spec: 550 mm)
    LW   = 0.02          # 차선 마킹 폭
    LANE = TW / 2        # = 0.275 m (반폭)
    EDGE = LANE - LW / 2 # = 0.265 m (도로 중심 → 차선 마킹 중심)

    # 코너 아크 반지름 (도로 중심선 기준, R_CORNER = TW)
    R_CORNER = TW
    R_IN     = R_CORNER - EDGE   # = 0.285 m (내측 차선 아크)
    R_CTR    = R_CORNER          # = 0.55  m (중앙 차선 아크)
    R_OUT    = R_CORNER + EDGE   # = 0.815 m (외측 차선 아크)

    BL       = abs(RX - LX)          # = 3.30 m (동서 총 폭)
    VL       = abs(TY - BY)          # = 3.30 m (남북 총 높이)
    BL_inner = BL - 2 * R_CORNER     # = 2.20 m (Bottom/Top 직선 순수 길이)
    VL_inner = VL - 2 * R_CORNER     # = 2.20 m (Right/Left 직선 순수 길이)

    # 코너 pivot (CCW 주행: Bottom→동, Right→북, Top→서, Left→남 → 모두 좌회전)
    pvSE = (RX - R_CORNER, BY + R_CORNER)   # ( 1.10, -1.10)  270°→360°
    pvNE = (RX - R_CORNER, TY - R_CORNER)   # ( 1.10,  1.10)    0°→ 90°
    pvNW = (LX + R_CORNER, TY - R_CORNER)   # (-1.10,  1.10)   90°→180°
    pvSW = (LX + R_CORNER, BY + R_CORNER)   # (-1.10, -1.10)  180°→270°

    # 공사구간 (동쪽 직선, 배리어 y 범위 ± 여유)
    CONSTR_Y0 = 0.35    # 점선 시작 y
    CONSTR_Y1 = 1.55    # 점선 끝 y

    # 점선 파라미터
    DASH_ON  = 0.07
    DASH_OFF = 0.07

    N_ARC = 14   # 코너 아크 분할 수

    # ─── 1) Physics + Ground ────────────────────────────────────────────────
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)

    # Gazebo world: 8×8 m 바닥 (z=-0.1)
    omni.kit.commands.execute(
        "AddGroundPlaneCommand",
        stage=stage, planePath="/World/GroundPlane",
        axis="Z", size=16.0,
        position=Gf.Vec3f(0, 0, 0),
        color=Gf.Vec3f(0.20, 0.20, 0.20),
    )

    # ─── 2) 조명 (Gazebo world 참조) ─────────────────────────────────────────
    # DirectionalLight ← Gazebo sun (direction: -0.5, 0.5, -1)
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(400.0)

    sun = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    sun.CreateIntensityAttr(20000.0)
    UsdGeom.XformCommonAPI(sun).SetRotate(Gf.Vec3f(-50.0, 0.0, -25.0))

    # Point lights (Gazebo world 좌표 그대로, z=1 m)
    for i, (px, py) in enumerate([
        (-1.648,  0.605),
        ( 1.639,  1.696),
        ( 0.420, -3.310),
    ]):
        pl = UsdLux.SphereLight.Define(stage, f"/World/PointLight{i}")
        pl.CreateIntensityAttr(4000.0)
        pl.CreateRadiusAttr(0.05)
        UsdGeom.XformCommonAPI(pl).SetTranslate(Gf.Vec3d(px, py, 1.0))

    # ─── 3) 트랙 표면 (아스팔트) ──────────────────────────────────────────────
    UsdGeom.Scope.Define(stage, "/World/Track")

    # 직선 구간 (코너 제외)
    for nm, cx, cy, sx, sy in [
        ("Bottom", 0.0, BY, BL_inner, TW),
        ("Top",    0.0, TY, BL_inner, TW),
        ("Right",  RX,  CY, TW, VL_inner),
        ("Left",   LX,  CY, TW, VL_inner),
    ]:
        _box(stage, f"/World/Track/{nm}", cx, cy, TZ, sx, sy, TH,
             "asph", ASPHALT, rough=0.9, phys=True)

    # 코너 표면 (pivot 기준 R_OUT 정사각형 근사)
    for nm, pvx, pvy, sx_s, sy_s in [
        ("CSE", pvSE[0], pvSE[1],  1, -1),
        ("CNE", pvNE[0], pvNE[1],  1,  1),
        ("CNW", pvNW[0], pvNW[1], -1,  1),
        ("CSW", pvSW[0], pvSW[1], -1, -1),
    ]:
        _box(stage, f"/World/Track/{nm}",
             pvx + sx_s * R_OUT / 2,
             pvy + sy_s * R_OUT / 2,
             TZ, R_OUT, R_OUT, TH,
             "asph", ASPHALT, rough=0.9, phys=True)

    # ─── 4) 차선 마킹 helper ──────────────────────────────────────────────────
    UsdGeom.Scope.Define(stage, "/World/Marks")
    _idx = [0]

    def _solid(path, cx, cy, sx, sy):
        _box(stage, path, cx, cy, MZ, sx, sy, MH,
             "white", WHITE, rough=0.05, emit=EW)

    def _dash_line(scope, x0, y0, x1, y1, thick=LW, dash=DASH_ON, gap=DASH_OFF):
        dx, dy = x1 - x0, y1 - y0
        total  = math.sqrt(dx * dx + dy * dy)
        if total < 1e-6:
            return
        ux, uy = dx / total, dy / total
        rz = math.degrees(math.atan2(dy, dx)) - 90.0
        s, i = 0.0, 0
        while s + dash <= total:
            sc = s + dash / 2
            _box(stage, f"{scope}/d{i:04d}",
                 x0 + sc * ux, y0 + sc * uy, MZ,
                 thick, dash, MH,
                 "white", WHITE, rough=0.05, emit=EW, rz=rz)
            s += dash + gap
            i += 1

    def _dash_arc(scope, pvx, pvy, R, a0_deg, a1_deg,
                  thick=LW, dash=0.08, gap=0.08):
        total = R * math.radians(abs(a1_deg - a0_deg))
        if total < 1e-6:
            return
        s, i = 0.0, 0
        while s + dash <= total:
            sc = s + dash / 2
            t      = sc / total
            a_mid  = math.radians(a0_deg + t * (a1_deg - a0_deg))
            cx_a   = pvx + R * math.cos(a_mid)
            cy_a   = pvy + R * math.sin(a_mid)
            _box(stage, f"{scope}/d{i:04d}",
                 cx_a, cy_a, MZ, thick, dash, MH,
                 "white", WHITE, rough=0.05, emit=EW, rz=math.degrees(a_mid))
            s += dash + gap
            i += 1

    def _corner_3arcs(scope, pvx, pvy, a0_deg, a1_deg):
        span_deg = a1_deg - a0_deg
        # 내측/외측: 실선 세그먼트
        for r, key in [(R_IN, "in"), (R_OUT, "out")]:
            for i in range(N_ARC + 1):
                t     = i / N_ARC
                ang_d = a0_deg + t * span_deg
                ang_r = math.radians(ang_d)
                sl    = max(r * math.radians(abs(span_deg)) / N_ARC * 1.2, LW * 2)
                _box(stage, f"{scope}/{key}_{i:02d}",
                     pvx + r * math.cos(ang_r),
                     pvy + r * math.sin(ang_r),
                     MZ, LW, sl, MH,
                     "white", WHITE, rough=0.05, emit=EW, rz=ang_d)
        # 중앙: 점선 호
        UsdGeom.Scope.Define(stage, f"{scope}/ctr")
        _dash_arc(f"{scope}/ctr", pvx, pvy, R_CTR, a0_deg, a1_deg)

    # ─── 5) 직선 구간 3선 마킹 ────────────────────────────────────────────────
    UsdGeom.Scope.Define(stage, "/World/Marks/Straight")

    # 직선 x/y 범위 (코너 pivot 사이)
    x_bot0, x_bot1 = pvSW[0], pvSE[0]   # -1.10 ~ +1.10
    x_top0, x_top1 = pvNW[0], pvNE[0]   # -1.10 ~ +1.10
    y_rit0, y_rit1 = pvSE[1], pvNE[1]   # -1.10 ~ +1.10
    y_lft0, y_lft1 = pvSW[1], pvNW[1]   # -1.10 ~ +1.10

    # ── Bottom (남쪽, x 방향): 장애물 없음 → 중앙선 실선
    _solid("/World/Marks/Straight/BO",  0.0, BY - EDGE, BL_inner, LW)  # 외측
    _solid("/World/Marks/Straight/BC",  0.0, BY,        BL_inner, LW)  # 중앙
    _solid("/World/Marks/Straight/BI",  0.0, BY + EDGE, BL_inner, LW)  # 내측

    # 시작선
    _solid("/World/Marks/Straight/Start", 1.30, BY, LW * 4, TW)

    # ── Top (북쪽, x 방향): 장애물 없음 → 중앙선 실선
    _solid("/World/Marks/Straight/TI",  0.0, TY - EDGE, BL_inner, LW)  # 내측
    _solid("/World/Marks/Straight/TC",  0.0, TY,        BL_inner, LW)  # 중앙
    _solid("/World/Marks/Straight/TO",  0.0, TY + EDGE, BL_inner, LW)  # 외측

    # ── Right (동쪽, y 방향): 공사구간(y=CONSTR_Y0~Y1) → 중앙선 점선
    _solid("/World/Marks/Straight/RO", RX + EDGE, CY, LW, VL_inner)   # 외측 실선
    _solid("/World/Marks/Straight/RI", RX - EDGE, CY, LW, VL_inner)   # 내측 실선
    # 중앙선 구간 분리
    UsdGeom.Scope.Define(stage, "/World/Marks/Straight/RC")
    if CONSTR_Y0 > y_rit0:
        _solid("/World/Marks/Straight/RC_lo",
               RX, (y_rit0 + CONSTR_Y0) / 2, LW, CONSTR_Y0 - y_rit0)
    _dash_line("/World/Marks/Straight/RC",
               RX, CONSTR_Y0, RX, CONSTR_Y1)
    if CONSTR_Y1 < y_rit1:
        _solid("/World/Marks/Straight/RC_hi",
               RX, (CONSTR_Y1 + y_rit1) / 2, LW, y_rit1 - CONSTR_Y1)

    # ── Left (서쪽, y 방향): 터널 구간 포함 → 중앙선 실선
    _solid("/World/Marks/Straight/LO", LX - EDGE, CY, LW, VL_inner)   # 외측
    UsdGeom.Scope.Define(stage, "/World/Marks/Straight/LC")
    _dash_line("/World/Marks/Straight/LC",
               LX, y_lft0, LX, y_lft1)                                 # 중앙 점선
    _solid("/World/Marks/Straight/LI", LX + EDGE, CY, LW, VL_inner)   # 내측

    # ─── 6) 코너 3선 원호 마킹 ────────────────────────────────────────────────
    UsdGeom.Scope.Define(stage, "/World/Marks/Corners")

    for cname, (pvx, pvy), a0, a1 in [
        ("SE", pvSE, 270.0, 360.0),
        ("NE", pvNE,   0.0,  90.0),
        ("NW", pvNW,  90.0, 180.0),
        ("SW", pvSW, 180.0, 270.0),
    ]:
        sp = f"/World/Marks/Corners/{cname}"
        UsdGeom.Scope.Define(stage, sp)
        _corner_3arcs(sp, pvx, pvy, a0, a1)

    # ─── 7) Tunnel 벽체 (world 파일 <state> 절대좌표) ─────────────────────────
    # 출처: turtlebot3_autorace_2020.world  tunnel_wall model
    # rot_z=90°: box 가 y축 방향으로 연장됨 (Gazebo rot=1.5708 동일)
    # wall size: (Gazebo size_x, size_y, size_z) → Isaac (sx, sy, sz)
    #   rot=0/180: sx=length(x방향), sy=thick(y방향)
    #   rot=90   : sx=length(y방향), sy=thick(x방향)  ← Isaac T·R·S 기준
    UsdGeom.Scope.Define(stage, "/World/Tunnel")

    tunnel_walls = [
        # (name,  cx,        cy,       cz,    sx_len, sy_thi, sz_h,  rz)
        ("W1", -1.91548, -1.00586, 0.125, 1.800,  0.05,  0.25,  90.0),
        ("W2", -0.99470, -1.92118, 0.125, 1.800,  0.05,  0.25,   0.0),
        ("W3", -0.02130, -0.83207, 0.125, 1.537,  0.05,  0.25,  90.0),
        ("W4", -0.81470, -0.03074, 0.125, 1.580,  0.05,  0.25, 180.0),
    ]
    for nm, cx, cy, cz, sl, st, sh, rz in tunnel_walls:
        _box(stage, f"/World/Tunnel/{nm}",
             cx, cy, cz, sl, st, sh,
             "gray", GRAY, rough=0.7, phys=True, rz=rz)

    # ─── 8) Tunnel 슬라롬 장애물 실린더 (world 파일 <state> 절대좌표) ──────────
    # r=0.10 m, h=0.25 m, cz=0.125 m (실린더 중심 = 바닥+h/2)
    UsdGeom.Scope.Define(stage, "/World/TunnelObs")

    tunnel_obs = [
        ("O1", -0.61823, -1.33874),
        ("O2", -1.36518, -1.41427),
        ("O3", -1.14094, -0.62712),
    ]
    for nm, cx, cy in tunnel_obs:
        _cyl(stage, f"/World/TunnelObs/{nm}",
             cx, cy, 0.125, 0.10, 0.25,
             "gray", GRAY, rough=0.7, phys=True)

    # ─── 9) 공사구간 Barrier 박스 (world 파일 <state> 절대좌표) ─────────────────
    # size: 0.25(x) × 0.10(y) × 0.30(z), cz=0.15 (바닥에 안착)
    UsdGeom.Scope.Define(stage, "/World/Barriers")

    barriers = [
        ("B1", 1.490, 0.540, 0.15),
        ("B2", 1.740, 1.000, 0.15),
        ("B3", 1.490, 1.464, 0.15),
    ]
    for nm, cx, cy, cz in barriers:
        _box(stage, f"/World/Barriers/{nm}",
             cx, cy, cz, 0.25, 0.10, 0.30,
             "barrier", (0.60, 0.60, 0.60), rough=0.7, phys=True)

    # ─── 완료 ────────────────────────────────────────────────────────────────
    print("[INFO] AutoRace 2020 씬 구성 완료")
    print(f"  루프    : {BL:.2f} m × {VL:.2f} m, 코너 R={R_CORNER:.2f} m")
    print(f"  도로폭  : {TW:.3f} m  (LANE={LANE:.3f} m × 2)")
    print(f"  차선폭  : {LW:.3f} m")
    print(f"  외곽선  : y=±{BY-EDGE:.3f}, x=±{RX+EDGE:.3f} ← traffic 표지판과 일치")
    print(f"  공사구간: y={CONSTR_Y0}~{CONSTR_Y1} (동쪽 중앙선 점선)")
    print(f"  터널벽  : 4개 (world 좌표 그대로)")
    print(f"  슬라롬  : 실린더 3개")
    print(f"  배리어  : 박스 3개")


# ─── 씬 빌드 + 카메라 + 시뮬레이션 ───────────────────────────────────────────
build_scene()

for _ in range(10):
    simulation_app.update()


def setup_camera():
    try:
        import omni.kit.viewport.utility as vu
        vp = vu.get_active_viewport()
        if not vp:
            return
        cp = omni.usd.get_context().get_stage().GetPrimAtPath(vp.camera_path)
        if cp.IsValid():
            xf = UsdGeom.XformCommonAPI(cp)
            xf.SetTranslate(Gf.Vec3d(0.0, 0.0, 7.0))
            xf.SetRotate(Gf.Vec3f(-90.0, 0.0, 0.0))
            print("[INFO] 카메라: (0, 0, 7) 수직하향")
    except Exception as e:
        print(f"[WARN] 카메라 설정 실패: {e}")


setup_camera()
for _ in range(5):
    simulation_app.update()

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
