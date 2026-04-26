"""
TurtleBot3 AutoRace 2020 트랙  —  Isaac Sim Standalone (v4)
============================================================
맵     : 6 m × 6 m 아스팔트 바닥판
루프   : 3.6 m × 3.3 m 직사각형 + R=0.76 m 원호 코너 (4 코너)
차선   : 3선 시스템 @ z=0.005 m
         ┌ 최외곽 실선 (흰색, 0.02 m 폭)
         ├ 중앙선  실선 or 점선 (장애물 구간 = 점선 → 차선 변경 허용)
         └ 최내곽 실선 (흰색)
내부망 : 십자형(+) 내부 연결로 — 수평(y=CY) + 수직(x=0) + 4-way 교차점
로봇   : USD 원점 (0.06, -1.87, -0.712)
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

print(f"[INFO] 로봇 USD 로드 중: {ROBOT_USD}")
omni.usd.get_context().open_stage(ROBOT_USD)
for _ in range(20):
    simulation_app.update()


# ─── 재료 캐시 ────────────────────────────────────────────────────────────────
_mats: dict = {}


def _mat(stage, key: str, rgb, rough: float = 0.8, emit=None):
    if key in _mats:
        return _mats[key]
    idx = len(_mats)
    p   = f"/World/Mats/M{idx:03d}"
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
    """Cube 프리미티브. XformCommonAPI 순서: T · R · S"""
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
    """Cylinder 프리미티브 (axis=Z)"""
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
    ASPHALT = (0.05, 0.05, 0.05)
    WHITE   = (1.00, 1.00, 1.00)
    ORANGE  = (1.00, 0.40, 0.00)
    GRAY    = (0.55, 0.55, 0.55)

    # ── 기본 치수
    LANE = 0.38          # 단일 차선 폭
    TW   = LANE * 2      # 도로 총 폭 = 0.76 m
    LW   = 0.02          # 차선 마킹 폭
    EDGE = LANE - LW / 2 # 도로 중심→차선 중심 = 0.37 m

    TH = 0.003           # 트랙 슬랩 두께
    TZ = TH / 2
    MH = 0.001           # 차선 마킹 두께
    MZ = 0.005           # 차선 마킹 z 위치

    # ── 루프 중심선 좌표
    BY = -1.80   # Bottom y
    RX =  1.80   # Right  x
    TY =  1.50   # Top    y
    LX = -1.80   # Left   x
    CY = (BY + TY) / 2   # = -0.15
    BL = abs(RX - LX)    # = 3.60 m
    VL = abs(TY - BY)    # = 3.30 m

    # ── 코너 아크 반지름 (R_CORNER = TW → 내측 아크도 눈에 보이는 크기)
    R_CORNER = TW                  # = 0.76 m  (도로 중심선 코너 아크 반지름)
    R_IN     = R_CORNER - EDGE     # = 0.39 m  (내측 차선 아크)
    R_CTR    = R_CORNER            # = 0.76 m  (중앙 차선 아크)
    R_OUT    = R_CORNER + EDGE     # = 1.13 m  (외측 차선 아크)

    # ── R_CORNER 코너를 수용하기 위한 직선 구간 축소
    BL_inner = BL - 2 * R_CORNER  # = 2.08 m  (Bottom/Top 직선 순수 길이)
    VL_inner = VL - 2 * R_CORNER  # = 1.78 m  (Right/Left 직선 순수 길이)

    # ── 코너 pivot 좌표 (각 코너 호의 중심)
    # CCW 주행: Bottom→동, Right→북, Top→서, Left→남 → 모든 코너 좌회전
    pvSE = (RX - R_CORNER, BY + R_CORNER)  # = ( 1.04, -1.04)  arc 270→360
    pvNE = (RX - R_CORNER, TY - R_CORNER)  # = ( 1.04,  0.74)  arc   0→ 90
    pvNW = (LX + R_CORNER, TY - R_CORNER)  # = (-1.04,  0.74)  arc  90→180
    pvSW = (LX + R_CORNER, BY + R_CORNER)  # = (-1.04, -1.04)  arc 180→270

    # ── 내부 도로 치수
    IH_X0   = LX + TW / 2   # = -1.42  수평 연결로 서쪽 시작
    IH_X1   = RX - TW / 2   # = +1.42  수평 연결로 동쪽 끝
    CROSS_H = TW / 2         # = +0.38  교차점 반폭
    IH_SEG  = -CROSS_H - IH_X0  # = -0.38 - (-1.42) = 1.04  한 쪽 절반 길이

    IV_Y0   = BY + TW / 2   # = -1.42  수직 연결로 남쪽 시작
    IV_Y1   = TY - TW / 2   # = +1.12  수직 연결로 북쪽 끝
    CROSS_V = TW / 2         # = 0.38   교차점 반폭 (y 방향)
    IV_SEG_S = abs(CY - CROSS_V - IV_Y0)  # = abs(-0.53 - (-1.42)) = 0.89
    IV_SEG_N = IV_Y1 - (CY + CROSS_V)     # = 1.12 - 0.23 = 0.89

    # ── 점선 파라미터
    DASH_ON  = 0.06
    DASH_OFF = 0.06

    EW = (0.7, 0.7, 0.7)   # 흰 차선 약한 emissive

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

    # ─── 3) 아스팔트 바닥판 ─────────────────────────────────────────────────
    _box(stage, "/World/Ground", 0, 0, -0.005, 6.0, 6.0, 0.01,
         "asphalt", ASPHALT, rough=0.9)

    # ─── 4) 트랙 표면 ───────────────────────────────────────────────────────
    UsdGeom.Scope.Define(stage, "/World/Track")

    # 4-a) 직선 구간 (코너 아크 공간 제외)
    for nm, cx, cy, sx, sy in [
        ("Bot",   0.0, BY,  BL_inner, TW),
        ("Right", RX,  CY,  TW,       VL_inner),
        ("Top",   0.0, TY,  BL_inner, TW),
        ("Left",  LX,  CY,  TW,       VL_inner),
    ]:
        _box(stage, f"/World/Track/{nm}", cx, cy, TZ, sx, sy, TH,
             "track", ASPHALT, rough=0.9)

    # 4-b) 코너 표면 (bounding box: pivot에서 R_OUT × R_OUT 정사각형)
    for nm, pvx, pvy, sx_sign, sy_sign in [
        ("CSE", pvSE[0], pvSE[1],  1, -1),
        ("CNE", pvNE[0], pvNE[1],  1,  1),
        ("CNW", pvNW[0], pvNW[1], -1,  1),
        ("CSW", pvSW[0], pvSW[1], -1, -1),
    ]:
        cx_c = pvx + sx_sign * R_OUT / 2
        cy_c = pvy + sy_sign * R_OUT / 2
        _box(stage, f"/World/Track/{nm}", cx_c, cy_c, TZ,
             R_OUT, R_OUT, TH, "track", ASPHALT, rough=0.9)

    # 4-c) 내부 십자 도로 표면
    UsdGeom.Scope.Define(stage, "/World/InnerRoad")

    IH_CX_W = (IH_X0 + (-CROSS_H)) / 2   # = (-1.42 + -0.38)/2 = -0.90
    IH_CX_E = (CROSS_H  + IH_X1) / 2     # = ( 0.38 +  1.42)/2 =  0.90
    IV_CY_S = (IV_Y0 + (CY - CROSS_V)) / 2  # = (-1.42 + -0.53)/2 = -0.975
    IV_CY_N = ((CY + CROSS_V) + IV_Y1) / 2  # = ( 0.23 +  1.12)/2 =  0.675

    for nm, cx, cy, sx, sy in [
        ("HW",    IH_CX_W, CY,     IH_SEG,   TW),
        ("HE",    IH_CX_E, CY,     IH_SEG,   TW),
        ("VS",    0.0,     IV_CY_S, TW,      IV_SEG_S),
        ("VN",    0.0,     IV_CY_N, TW,      IV_SEG_N),
        ("Cross", 0.0,     CY,      TW,       TW),
    ]:
        _box(stage, f"/World/InnerRoad/{nm}", cx, cy, TZ, sx, sy, TH,
             "track", ASPHALT, rough=0.9)

    # ─── 5) 차선 마킹 ───────────────────────────────────────────────────────
    UsdGeom.Scope.Define(stage, "/World/Marks")

    # ── 헬퍼: 실선 1개
    def _solid(path, cx, cy, sx, sy):
        _box(stage, path, cx, cy, MZ, sx, sy, MH,
             "mk_white", WHITE, rough=0.05, emit=EW)

    # ── 헬퍼: x 방향 점선 (x0 → x1, y 고정)
    def _dash_x(scope, x0, x1, y):
        x, i = x0 + DASH_ON / 2, 0
        while x + DASH_ON / 2 <= x1:
            _box(stage, f"{scope}/d{i:03d}", x, y, MZ,
                 DASH_ON, LW, MH, "mk_white", WHITE, rough=0.05, emit=EW)
            x += DASH_ON + DASH_OFF
            i += 1

    # ── [신규] 헬퍼: 임의 방향 점선 직선 (누적 호장 기반)
    # spawn_lane_segments style="dashed" 의 직선 구현체
    def _dash_line(scope, x0, y0, x1, y1, thick=LW, dash=0.10, gap=0.10):
        dx, dy = x1 - x0, y1 - y0
        total = math.sqrt(dx * dx + dy * dy)
        if total < 1e-6:
            return
        ux, uy = dx / total, dy / total
        # rz: local-y 축이 (ux,uy) 방향을 향하도록 (= tang_angle − 90°)
        rz = math.degrees(math.atan2(dy, dx)) - 90.0
        s, i = 0.0, 0
        while s + dash <= total:
            sc = s + dash / 2
            _box(stage, f"{scope}/d{i:03d}",
                 x0 + sc * ux, y0 + sc * uy, MZ,
                 thick, dash, MH,
                 "mk_white", WHITE, rough=0.05, emit=EW, rz=rz)
            s += dash + gap
            i += 1

    # ── [신규] 헬퍼: 점선 원호 (CCW: a0<a1, CW: a0>a1)
    # spawn_lane_segments style="dashed" 의 곡선 구현체
    def _dash_arc(scope, pvx, pvy, R, a0_deg, a1_deg, thick=LW, dash=0.05, gap=0.05):
        span = abs(a1_deg - a0_deg)
        total = R * math.radians(span)
        if total < 1e-6:
            return
        is_ccw = (a1_deg >= a0_deg)
        s, i = 0.0, 0
        while s + dash <= total:
            sc = s + dash / 2
            t = sc / total
            a_mid = math.radians(a0_deg + t * (a1_deg - a0_deg))
            cx = pvx + R * math.cos(a_mid)
            cy = pvy + R * math.sin(a_mid)
            a_mid_deg = math.degrees(a_mid)
            # CCW: rz = a_mid_deg  /  CW: rz = a_mid_deg − 180°
            rz = a_mid_deg if is_ccw else (a_mid_deg - 180.0)
            _box(stage, f"{scope}/d{i:03d}",
                 cx, cy, MZ, thick, dash, MH,
                 "mk_white", WHITE, rough=0.4, emit=EW, rz=rz)
            s += dash + gap
            i += 1

    # 5-a) 직선 구간 3선 ─────────────────────────────────────────────────────
    UsdGeom.Scope.Define(stage, "/World/Marks/Straight")

    SL = BL_inner   # = 2.08 m  (Bottom/Top 직선 길이)
    VLS = VL_inner  # = 1.78 m  (Right/Left 직선 길이)

    # ── Bottom (y=BY=-1.80, x 방향): 장애물 없음 → 중앙선 실선
    _solid("/World/Marks/Straight/BS",  0.0, BY - EDGE, SL,  LW)   # 외측(남)
    UsdGeom.Scope.Define(stage, "/World/Marks/Straight/BC")        # 중앙 점선
    _dash_line("/World/Marks/Straight/BC", -SL / 2, BY, SL / 2, BY)
    _solid("/World/Marks/Straight/BN",  0.0, BY + EDGE, SL,  LW)   # 내측(북)

    # ── Right (x=RX=1.80, y 방향): 터널 있으나 차선 변경 불필요 → 중앙선 실선
    _solid("/World/Marks/Straight/RE",  RX + EDGE, CY,  LW, VLS)   # 외측(동)
    UsdGeom.Scope.Define(stage, "/World/Marks/Straight/RC")        # 중앙 점선
    _dash_line("/World/Marks/Straight/RC", RX, CY - VLS / 2, RX, CY + VLS / 2)
    _solid("/World/Marks/Straight/RW",  RX - EDGE, CY,  LW, VLS)   # 내측(서)

    # ── Top (y=TY=1.50, x 방향): 슬라롬 장애물 → 중앙선 점선 (차선 변경 허용)
    _solid("/World/Marks/Straight/TN",  0.0, TY + EDGE, SL,  LW)   # 외측(북) 실선
    _solid("/World/Marks/Straight/TS",  0.0, TY - EDGE, SL,  LW)   # 내측(남) 실선
    UsdGeom.Scope.Define(stage, "/World/Marks/Straight/TC")
    _dash_x("/World/Marks/Straight/TC",                              # 중앙 점선
            -SL / 2, SL / 2, TY)

    # ── Left (x=LX=-1.80, y 방향): 장애물 없음 → 중앙선 실선
    _solid("/World/Marks/Straight/LW",  LX - EDGE, CY,  LW, VLS)   # 외측(서)
    UsdGeom.Scope.Define(stage, "/World/Marks/Straight/LC")        # 중앙 점선
    _dash_line("/World/Marks/Straight/LC", LX, CY - VLS / 2, LX, CY + VLS / 2)
    _solid("/World/Marks/Straight/LE",  LX + EDGE, CY,  LW, VLS)   # 내측(동)

    # ── 시작선 (Bottom 중앙)
    _solid("/World/Marks/Straight/Start", 0.0, BY, LW * 4, TW)

    # 5-b) 코너 3선 원호 마킹 ─────────────────────────────────────────────────
    # rot_z = angle_deg 으로 설정해야 local+y 가 CCW 접선 방향과 일치
    # (rot_z = angle + 90 은 반지름 방향 → 오류)
    UsdGeom.Scope.Define(stage, "/World/Marks/Corners")

    N_ARC = 14   # 원호 분할 수

    def _corner_3arcs(scope, pvx, pvy, a0_deg, a1_deg):
        """pivot(pvx,pvy) 기준 a0→a1 도 범위의 3선 원호 마킹.
        내측(R_IN)/외측(R_OUT): 실선 (style='solid')
        중앙(R_CTR): 점선 (style='dashed', _dash_arc 사용)
        """
        span_deg = a1_deg - a0_deg
        # ── 내측 / 외측: 실선 세그먼트
        for r, key in [(R_IN, "in"), (R_OUT, "out")]:
            for i in range(N_ARC + 1):
                t     = i / N_ARC
                ang_d = a0_deg + t * span_deg
                ang_r = math.radians(ang_d)
                px    = pvx + r * math.cos(ang_r)
                py    = pvy + r * math.sin(ang_r)
                sl    = max(r * math.radians(abs(span_deg)) / N_ARC * 1.2,
                            LW * 2)
                _box(stage, f"{scope}/{key}_{i:02d}",
                     px, py, MZ, LW, sl, MH,
                     "mk_white", WHITE, rough=0.05, emit=EW, rz=ang_d)
        # ── 중앙: 점선 호 (style='dashed')
        UsdGeom.Scope.Define(stage, f"{scope}/ctr")
        _dash_arc(f"{scope}/ctr", pvx, pvy, R_CTR, a0_deg, a1_deg,
                  thick=LW, dash=0.08, gap=0.08)

    for cname, (pvx, pvy), a0, a1 in [
        ("SE", pvSE, 270.0, 360.0),
        ("NE", pvNE,   0.0,  90.0),
        ("NW", pvNW,  90.0, 180.0),
        ("SW", pvSW, 180.0, 270.0),
    ]:
        sp = f"/World/Marks/Corners/{cname}"
        UsdGeom.Scope.Define(stage, sp)
        _corner_3arcs(sp, pvx, pvy, a0, a1)

    # 5-c) 내부 십자 도로 차선 마킹 (3선, 모두 실선) ────────────────────────
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner")

    # 수평 연결로 서쪽 (IH_W): 외측(S)/내측(N) 실선, 중앙(C) 점선
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/HW")
    _solid("/World/Marks/Inner/HW/S", IH_CX_W, CY - EDGE, IH_SEG, LW)
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/HW/C")
    _dash_line("/World/Marks/Inner/HW/C",
               IH_CX_W - IH_SEG / 2, CY, IH_CX_W + IH_SEG / 2, CY)
    _solid("/World/Marks/Inner/HW/N", IH_CX_W, CY + EDGE, IH_SEG, LW)

    # 수평 연결로 동쪽 (IH_E): 외측(S)/내측(N) 실선, 중앙(C) 점선
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/HE")
    _solid("/World/Marks/Inner/HE/S", IH_CX_E, CY - EDGE, IH_SEG, LW)
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/HE/C")
    _dash_line("/World/Marks/Inner/HE/C",
               IH_CX_E - IH_SEG / 2, CY, IH_CX_E + IH_SEG / 2, CY)
    _solid("/World/Marks/Inner/HE/N", IH_CX_E, CY + EDGE, IH_SEG, LW)

    # 수직 연결로 남쪽 (IV_S): 외측(W)/내측(E) 실선, 중앙(C) 점선
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/VS")
    _solid("/World/Marks/Inner/VS/W", -EDGE, IV_CY_S, LW, IV_SEG_S)
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/VS/C")
    _dash_line("/World/Marks/Inner/VS/C",
               0.0, IV_CY_S - IV_SEG_S / 2, 0.0, IV_CY_S + IV_SEG_S / 2)
    _solid("/World/Marks/Inner/VS/E",  EDGE, IV_CY_S, LW, IV_SEG_S)

    # 수직 연결로 북쪽 (IV_N): 외측(W)/내측(E) 실선, 중앙(C) 점선
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/VN")
    _solid("/World/Marks/Inner/VN/W", -EDGE, IV_CY_N, LW, IV_SEG_N)
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/VN/C")
    _dash_line("/World/Marks/Inner/VN/C",
               0.0, IV_CY_N - IV_SEG_N / 2, 0.0, IV_CY_N + IV_SEG_N / 2)
    _solid("/World/Marks/Inner/VN/E",  EDGE, IV_CY_N, LW, IV_SEG_N)

    # 교차점(0, CY): 실제 도로 교차로 — 차선 마킹 없음 (의도적)

    # ─── 5-d) 정지선 (Stop Lines) ────────────────────────────────────────────
    # [신규] spawn_stop_lines: 교차로 4방향 진입부마다 굵은 흰색 정지선 1개씩
    # 위치: 교차로 경계(±CROSS_H)로부터 SL_OFF=0.10 m 바깥쪽
    # 두께: 0.04 m (일반 차선 0.02 m의 2배, 카메라 검출 라벨 구분)
    UsdGeom.Scope.Define(stage, "/World/Track/StopLines")
    SL_OFF = 0.10    # 정지선 → 교차로 경계 오프셋
    SL_THK = 0.04    # 정지선 두께

    for nm, cx, cy, sx, sy in [
        ("stop_N", 0.0,               CY + CROSS_H + SL_OFF, TW,    SL_THK),
        ("stop_S", 0.0,               CY - CROSS_H - SL_OFF, TW,    SL_THK),
        ("stop_E", CROSS_H + SL_OFF,  CY,                    SL_THK, TW   ),
        ("stop_W", -(CROSS_H + SL_OFF), CY,                  SL_THK, TW   ),
    ]:
        _box(stage, f"/World/Track/StopLines/{nm}",
             cx, cy, MZ, sx, sy, MH,
             "mk_white", WHITE, rough=0.4, emit=EW)

    # ─── 5-e) 교차로 내부 점선 십자 (정지선 사이 네모 안 가로/세로 점선) ────────
    # 정지선 사이 영역:
    #   x ∈ [-(CROSS_H+SL_OFF), +(CROSS_H+SL_OFF)]
    #   y ∈ [CY-(CROSS_H+SL_OFF), CY+(CROSS_H+SL_OFF)]
    # 가로선: y=CY, 서쪽정지선 → 동쪽정지선
    # 세로선: x=0,  남쪽정지선 → 북쪽정지선
    UsdGeom.Scope.Define(stage, "/World/Track/IntersectionCross")
    IX_HALF = CROSS_H + SL_OFF   # 정지선까지의 절반 거리 = 0.48 m

    UsdGeom.Scope.Define(stage, "/World/Track/IntersectionCross/H")
    _dash_line("/World/Track/IntersectionCross/H",
               -IX_HALF, CY, IX_HALF, CY,
               thick=LW, dash=0.06, gap=0.06)

    UsdGeom.Scope.Define(stage, "/World/Track/IntersectionCross/V")
    _dash_line("/World/Track/IntersectionCross/V",
               0.0, CY - IX_HALF, 0.0, CY + IX_HALF,
               thick=LW, dash=0.06, gap=0.06)

    # ─── 6) 슬라롬 장애물 큐브 (Top 직선) ───────────────────────────────────
    UsdGeom.Scope.Define(stage, "/World/Obstacles")
    OBH = 0.30
    for pname, ox, oy in [
        ("Obs0", -0.52, TY + LANE * 0.42),   # 외측 차선 (북)
        ("Obs1",  0.00, TY - LANE * 0.42),   # 내측 차선 (남) → 지그재그
        ("Obs2",  0.52, TY + LANE * 0.42),   # 외측 차선 (북)
    ]:
        _box(stage, f"/World/Obstacles/{pname}",
             ox, oy, OBH / 2, 0.2, 0.2, OBH,
             "obs_cube", WHITE, phys=True)

    # ─── 7) QIV: 터널 (Right 구간 하단) ────────────────────────────────────
    UsdGeom.Scope.Define(stage, "/World/QIV")

    TUN_H  = 0.35
    TUN_L  = 0.80
    TUN_T  = 0.03
    TUN_CY = -0.90

    LWX = RX - LANE - TUN_T / 2
    RWX = RX + LANE + TUN_T / 2
    RFZ = TUN_H + TUN_T / 2

    _box(stage, "/World/QIV/TunWL",
         LWX, TUN_CY, (TUN_H + TUN_T) / 2,
         TUN_T, TUN_L, TUN_H + TUN_T, "tun", GRAY, phys=True)
    _box(stage, "/World/QIV/TunWR",
         RWX, TUN_CY, (TUN_H + TUN_T) / 2,
         TUN_T, TUN_L, TUN_H + TUN_T, "tun", GRAY, phys=True)
    _box(stage, "/World/QIV/TunRoof",
         RX, TUN_CY, RFZ,
         TW + 2 * TUN_T, TUN_L, TUN_T, "tun", GRAY, phys=True)

    for suf, gy in [("F", TUN_CY + TUN_L / 2), ("B", TUN_CY - TUN_L / 2)]:
        _box(stage, f"/World/QIV/TunGate{suf}PL",
             LWX, gy, TUN_H / 2, TUN_T, TUN_T * 2, TUN_H, "tgate", ORANGE, phys=True)
        _box(stage, f"/World/QIV/TunGate{suf}PR",
             RWX, gy, TUN_H / 2, TUN_T, TUN_T * 2, TUN_H, "tgate", ORANGE, phys=True)
        _box(stage, f"/World/QIV/TunGate{suf}Top",
             RX, gy, TUN_H + TUN_T / 2,
             TW + 2 * TUN_T, TUN_T * 2, TUN_T, "tgate", ORANGE)

    # ─── 완료 로그 ──────────────────────────────────────────────────────────
    print("[INFO] AutoRace 트랙 구성 완료 (v5)")
    print(f"  루프   : {BL:.1f} × {VL:.1f} m, 코너 R={R_CORNER:.2f} m")
    print(f"  도로폭 : {TW:.2f} m  (차선 {LANE:.2f} m × 2)")
    print(f"  차선   : 외측/내측 실선 | 중앙 점선(d=0.10,g=0.10) | 코너 중앙 점선(0.08/0.08)")
    print(f"  정지선 : N/S/E/W 각 1개, 교차로 경계+{SL_OFF:.2f}m, 두께 {SL_THK:.2f}m")
    print(f"  교차로 : 정지선 사이 점선 십자(가로+세로, dash=0.06m, ±{IX_HALF:.2f}m)")
    print(f"  교차점 : (0, {CY:.2f}) — 4-way, 교차로 CROSS_H={CROSS_H:.2f} m")
    print(f"  터널   : y={TUN_CY}±{TUN_L/2:.2f}  (Right 구간)")
    print(f"  로봇   : y=-1.87 → Bottom y∈[{BY-LANE:.2f}, {BY+LANE:.2f}] ✓")


build_scene()

for _ in range(10):
    simulation_app.update()


# ── 카메라 초기 시점 (위에서 내려다보기) ─────────────────────────────────────
def setup_camera():
    try:
        import omni.kit.viewport.utility as vu
        vp = vu.get_active_viewport()
        if not vp:
            return
        cp = omni.usd.get_context().get_stage().GetPrimAtPath(vp.camera_path)
        if cp.IsValid():
            xf = UsdGeom.XformCommonAPI(cp)
            xf.SetTranslate(Gf.Vec3d(0.0, 0.0, 6.0))
            xf.SetRotate(Gf.Vec3f(-90.0, 0.0, 0.0))
            print("[INFO] 카메라: (0,0,6) 수직하향 — 교차로 정지선+가이드선 확인용")
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
