"""
autorace_lanefollow_v5.py — TurtleBot3 AutoRace LaneFollow
============================================================
맵    : 6.0 m × 9.0 m 아스팔트 바닥판
트랙  : 외곽 + 내부 굴곡 단일 루프 (S커브 1, U턴 2, 우회전 1, 완만 1)
차선  : 흰색 실선 2줄 (간격 0.60 m, 점선 없음)
장애물: 흰 큐브 2개(Bottom 직선) + 주황 콘 2개(Top 직선)

중심선: 15개 웨이포인트 → Catmull-Rom 스플라인 → 0.02 m 등간격 리샘플링
차선선: center ±0.30 m 법선 오프셋 → spawn_lane_segments() 로 얇은 Cube 배열
"""
import os
import sys
import math
from isaacsim import SimulationApp

sys.argv += ["--/app/extensions/excluded/0=omni.graph.image.core"]

simulation_app = SimulationApp({
    "headless": False,
    "experience": (
        "/home/linux/isaac_env/lib/python3.11/site-packages/"
        "isaacsim/apps/isaacsim.exp.full.kit"
    ),
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


# ═══════════════════════════════════════════════════════════════════════════════
# ─── 상수 ─────────────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

LANE_SEP = 0.60        # 두 흰선 중심 간 거리 (차선 폭, m)
HALF_SEP = LANE_SEP / 2  # = 0.30 m (중심선 → 각 흰선 오프셋)
LW       = 0.02        # 차선 마킹 폭 (m)
MH       = 0.001       # 차선 마킹 두께 (z scale, m)
MZ       = 0.005       # 차선 마킹 중심 z 위치 (m)

# 장애물 위치 상수 ── 실험 중 좌표 변경 시 여기만 수정 ──────────────────────
# 큐브: (이름, cx, cy, sx, sy, sz)
OBS_CUBES = [
    ("Obs1A", -0.6,  -3.6 + 0.18, 0.20, 0.20, 0.30),   # Bottom 직선 좌측 차선
    ("Obs1B", +0.6,  -3.6 - 0.18, 0.20, 0.20, 0.30),   # Bottom 직선 우측 차선
]
# 콘 실린더: (이름, cx, cy, radius, height)
OBS_CONES = [
    ("Obs2A", +0.5,  +3.6, 0.08, 0.30),   # Top 직선 우측 콘
    ("Obs2B", -0.5,  +3.6, 0.08, 0.30),   # Top 직선 좌측 콘
]


# ═══════════════════════════════════════════════════════════════════════════════
# ─── 재료 캐시 ─────────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

_mats: dict = {}


def _mat(stage, key: str, rgb, rough: float = 0.8, emit=None):
    """재료를 처음 생성한 뒤 캐시에 저장; 동일 key 는 캐시 반환."""
    if key in _mats:
        return _mats[key]
    p   = f"/World/Mats/M{len(_mats):03d}"
    mat = UsdShade.Material.Define(stage, p)
    sh  = UsdShade.Shader.Define(stage, f"{p}/Sh")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor",  Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness",     Sdf.ValueTypeNames.Float  ).Set(rough)
    if emit is not None:
        sh.CreateInput("emissiveColor",
                       Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emit))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    _mats[key] = mat
    return mat


def _bind(stage, prim, key: str, rgb, rough: float = 0.8, emit=None):
    """prim 에 재료 바인딩."""
    _mat(stage, key, rgb, rough, emit)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(_mats[key])


# ═══════════════════════════════════════════════════════════════════════════════
# ─── 프리미티브 헬퍼 ────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

def _box(stage, path: str,
         cx: float, cy: float, cz: float,
         sx: float, sy: float, sz: float,
         mkey: str, rgb,
         rough: float = 0.8, emit=None, phys: bool = False,
         rz: float = None):
    """Cube 프리미티브. 변환 순서: T · R · S.

    Args:
        rz: Z 축 회전 (도). None 이면 생략.
    """
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
    """Cylinder 프리미티브 (axis=Z)."""
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


# ═══════════════════════════════════════════════════════════════════════════════
# ─── 중심선 생성 함수 ───────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

def generate_centerline_waypoints():
    """트랙 중심선 웨이포인트 15개 반환 (CCW, 단위: m).

    Returns:
        List[Tuple[float, float]]: (x, y) 좌표 리스트.
    """
    # 외곽 x = ±1.8 m 로 설정 → CR 스플라인 최대 오버슈트 포함 후
    # 외곽선(±0.30 m 오프셋) 이 x ∈ [-2.6, +2.6] 에 안착.
    return [
        (-1.8, -3.6),   # P0  시작/종료점 — Bottom 직선 서쪽 끝
        (+1.8, -3.6),   # P1  우측 하단 U턴(좌회전 R≈0.5) 직전
        (+1.8, -2.6),   # P2  U턴 후 북진 시작
        (+1.8, -1.0),   # P3  내부 진입 좌회전(R≈0.8) 직전
        (+0.8, -0.3),   # P4  S커브 시작 (우→좌)
        (-0.6, +0.3),   # P5  S커브 끝
        (+0.6, +1.2),   # P6  내부 굴곡 ② 좌회전(R≈0.7)
        (+1.8, +2.0),   # P7  외곽 복귀 북진
        (+1.8, +3.6),   # P8  상단 좌회전 U턴(R≈0.5) 직전
        (-1.8, +3.6),   # P9  Top 직선 서쪽 끝
        (-1.8, +2.0),   # P10 내부 재진입 좌회전(R≈0.8)
        (-1.0, +0.8),   # P11 내부 굴곡 ③ 우회전(R≈0.6)
        (-1.5, -0.5),   # P12 완만한 좌회전(R≈1.2)
        (-1.8, -2.0),   # P13 좌측 직선 남진
        (-1.8, -3.6),   # P14 좌측 하단 좌회전 U턴(R≈0.5) → P0
    ]


def catmull_rom_segment(p0, p1, p2, p3, n: int):
    """균일 Catmull-Rom: p1 → p2 사이 n+1 개 점 반환.

    Args:
        p0, p1, p2, p3: (x, y) 제어점.
        n: 분할 수.

    Returns:
        List[Tuple[float, float]]: 보간된 점 리스트 (n+1개).
    """
    pts = []
    for i in range(n + 1):
        t  = i / n
        t2 = t * t
        t3 = t2 * t

        def _coord(a0, a1, a2, a3):
            return 0.5 * (
                2 * a1
                + (-a0 + a2) * t
                + (2*a0 - 5*a1 + 4*a2 - a3) * t2
                + (-a0 + 3*a1 - 3*a2 + a3) * t3
            )

        pts.append((_coord(p0[0], p1[0], p2[0], p3[0]),
                    _coord(p0[1], p1[1], p2[1], p3[1])))
    return pts


def _generate_centerline_raw(waypoints, pts_per_meter: int = 30):
    """Catmull-Rom 스플라인으로 닫힌 루프 원시 중심선 생성.

    Args:
        waypoints: 웨이포인트 리스트 (닫힘 없음).
        pts_per_meter: 미터당 보간 점 수.

    Returns:
        List[Tuple[float, float]]: 루프 닫기 없는 점 리스트.
    """
    N = len(waypoints)
    all_pts = []
    for i in range(N):
        p0 = waypoints[(i - 1) % N]
        p1 = waypoints[i]
        p2 = waypoints[(i + 1) % N]
        p3 = waypoints[(i + 2) % N]
        seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        n = max(4, round(seg_len * pts_per_meter))
        seg = catmull_rom_segment(p0, p1, p2, p3, n)
        all_pts.extend(seg[:-1])   # 마지막 점 제외 (다음 세그먼트와 중복)
    return all_pts                 # 열린 루프 (first ≠ last)


def resample_polyline(points, ds: float = 0.02):
    """폴리라인을 ds (m) 등간격으로 리샘플링.

    Args:
        points: (x, y) 리스트.
        ds: 샘플 간격 (m).

    Returns:
        List[Tuple[float, float]]: 리샘플된 점 리스트.
    """
    if len(points) < 2:
        return list(points)
    result = [points[0]]
    carry  = 0.0                     # 이전 세그먼트 잔여 거리
    for i in range(1, len(points)):
        x0, y0 = points[i - 1]
        x1, y1 = points[i]
        dx, dy  = x1 - x0, y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-9:
            continue
        d = ds - carry               # 이 세그먼트에서 첫 샘플까지 남은 거리
        while d <= seg_len:
            t = d / seg_len
            result.append((x0 + t * dx, y0 + t * dy))
            d += ds
        carry = d - seg_len          # 다음 세그먼트로 넘길 거리
    return result


def offset_polyline(points, offset: float):
    """닫힌 루프 폴리라인을 법선 방향으로 offset m 이동.

    법선 = 접선의 CCW 90° 회전 → offset > 0 = 좌측(내부 차선 기준 외측).

    Args:
        points: 열린 루프 (x, y) 리스트 (마지막 ≠ 첫 점).
        offset: 오프셋 거리 (m). 양수 = CCW 좌측.

    Returns:
        List[Tuple[float, float]]: 오프셋 점 리스트 (동일 길이).
    """
    N = len(points)
    out = []
    for i in range(N):
        prev = points[(i - 1) % N]
        curr = points[i]
        nxt  = points[(i + 1) % N]

        dx_in,  dy_in  = curr[0] - prev[0], curr[1] - prev[1]
        dx_out, dy_out = nxt[0]  - curr[0], nxt[1]  - curr[1]
        len_in  = math.hypot(dx_in,  dy_in)
        len_out = math.hypot(dx_out, dy_out)

        # 단위 접선 (입출 평균)
        if len_in < 1e-9:
            tx, ty = dx_out / len_out, dy_out / len_out
        elif len_out < 1e-9:
            tx, ty = dx_in / len_in, dy_in / len_in
        else:
            ux_in,  uy_in  = dx_in  / len_in,  dy_in  / len_in
            ux_out, uy_out = dx_out / len_out, dy_out / len_out
            tx = (ux_in + ux_out) * 0.5
            ty = (uy_in + uy_out) * 0.5
            nm = math.hypot(tx, ty)
            if nm > 1e-9:
                tx, ty = tx / nm, ty / nm

        # CCW 법선: 접선을 90° 반시계 회전
        nx, ny = -ty, tx
        out.append((curr[0] + offset * nx, curr[1] + offset * ny))
    return out


def spawn_lane_segments(stage, points, scope_path: str, material_key: str) -> int:
    """폴리라인 점들 사이를 얇은 Cube 세그먼트(yaw 회전)로 스폰.

    T·R·S 변환 순서:
        scale(seg_len, LW, MH) → rotate(0, 0, yaw) → translate(cx, cy, MZ)
    즉 local-X 축이 세그먼트 진행 방향과 일치 (yaw = atan2(dy, dx)).

    Args:
        stage: USD Stage.
        points: 닫힌 루프 (x, y) 리스트 (마지막 == 첫 점).
        scope_path: Scope USD 경로.
        material_key: 재료 캐시 키.

    Returns:
        int: 생성된 세그먼트 수.
    """
    WHITE = (1.0, 1.0, 1.0)
    EW    = (0.7, 0.7, 0.7)
    UsdGeom.Scope.Define(stage, scope_path)
    count = 0
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        dx, dy  = x1 - x0, y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-9:
            continue
        cx  = (x0 + x1) * 0.5
        cy  = (y0 + y1) * 0.5
        yaw = math.degrees(math.atan2(dy, dx))
        # sx = seg_len (진행방향), sy = LW (폭)
        _box(stage, f"{scope_path}/s{i:06d}",
             cx, cy, MZ,
             seg_len, LW, MH,
             material_key, WHITE, rough=0.4, emit=EW, rz=yaw)
        count += 1
    return count


def make_arc(center, radius: float, a0_deg: float, a1_deg: float,
             n: int = 20):
    """원호 점 리스트 헬퍼 (a0_deg → a1_deg).

    Args:
        center: (cx, cy) 호 중심.
        radius: 호 반지름 (m).
        a0_deg: 시작 각도 (도).
        a1_deg: 끝 각도 (도).
        n: 분할 수.

    Returns:
        List[Tuple[float, float]]: n+1 개 점.
    """
    cx, cy = center
    pts = []
    for i in range(n + 1):
        ang = math.radians(a0_deg + i / n * (a1_deg - a0_deg))
        pts.append((cx + radius * math.cos(ang),
                    cy + radius * math.sin(ang)))
    return pts


def validate_no_self_intersection(centerline, min_dist: float = 0.80) -> bool:
    """비인접 세그먼트 쌍의 최소 거리 검증.

    0.2 m 간격으로 다운샘플 후 O(N²) 거리 계산.
    위반(dist < min_dist) 발생 시 콘솔에 경고 출력.

    Args:
        centerline: 열린 루프 (x, y) 리스트.
        min_dist: 최소 허용 거리 (m).

    Returns:
        bool: True = 자기교차 없음.
    """
    def _pt2seg(p, a, b):
        """점 p 에서 선분 a-b 까지의 최단 거리."""
        ax, ay = a;  bx, by = b;  px, py = p
        dx, dy = bx - ax, by - ay
        len2   = dx*dx + dy*dy
        if len2 < 1e-9:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / len2))
        return math.hypot(px - ax - t*dx, py - ay - t*dy)

    # 0.2 m 간격 다운샘플
    step = max(1, round(0.20 / 0.02))
    thin = centerline[::step]
    N    = len(thin)
    warns = 0
    for i in range(N - 1):
        for j in range(i + 2, N - 1):
            # 루프 첫-마지막 연결 구간은 인접 취급
            if i == 0 and j == N - 2:
                continue
            d = min(
                _pt2seg(thin[i],   thin[j], thin[j+1]),
                _pt2seg(thin[i+1], thin[j], thin[j+1]),
                _pt2seg(thin[j],   thin[i], thin[i+1]),
                _pt2seg(thin[j+1], thin[i], thin[i+1]),
            )
            if d < min_dist:
                warns += 1

    if warns:
        print(f"[WARN] 자기교차 경고: {warns}개 세그먼트 쌍 거리 < {min_dist} m "
              f"(S커브·내부 굴곡 접근으로 인한 정상 범위 내 경고일 수 있음)")
    else:
        print(f"[INFO] 자기교차 없음 ✓  (최소 허용 거리 {min_dist} m)")
    return warns == 0


# ═══════════════════════════════════════════════════════════════════════════════
# ─── 씬 구성 ────────────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

def build_scene():
    """Isaac Sim 씬 전체 구성."""
    stage = omni.usd.get_context().get_stage()

    # 로봇 USD 기본 큐브 제거
    for _p in ["/World/Cube", "/World/Cube_01", "/World/Cube_02"]:
        if stage.GetPrimAtPath(_p).IsValid():
            stage.RemovePrim(_p)

    UsdGeom.Scope.Define(stage, "/World/Mats")

    ASPHALT = (0.05, 0.05, 0.05)
    ORANGE  = (1.00, 0.40, 0.00)
    WHITE   = (1.00, 1.00, 1.00)
    EW      = (0.70, 0.70, 0.70)

    TH = 0.010    # 아스팔트 슬랩 두께
    TZ = -TH / 2  # 슬랩 z 중심 (윗면 = z=0)

    # ─── 1) Physics Scene + GroundPlane ────────────────────────────────────
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)

    omni.kit.commands.execute(
        "AddGroundPlaneCommand",
        stage=stage, planePath="/World/GroundPlane",
        axis="Z", size=30.0,
        position=Gf.Vec3f(0, 0, 0),
        color=Gf.Vec3f(0.15, 0.15, 0.15),
    )

    # ─── 2) 조명 (기존 v4 그대로) ───────────────────────────────────────────
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(600.0)

    rect = UsdLux.RectLight.Define(stage, "/World/RectLight")
    rect.CreateIntensityAttr(28000.0)
    rect.CreateWidthAttr(10.0)
    rect.CreateHeightAttr(10.0)
    UsdGeom.XformCommonAPI(rect).SetTranslate(Gf.Vec3d(0, 0, 7))
    UsdGeom.XformCommonAPI(rect).SetRotate(Gf.Vec3f(-90, 0, 0))

    # ─── 3) 6 × 9 m 아스팔트 슬랩 ─────────────────────────────────────────
    _box(stage, "/World/Ground",
         0.0, 0.0, TZ, 6.0, 9.0, TH,
         "asphalt", ASPHALT, rough=0.9)

    # ─── 4) 중심선 생성 + 검증 ─────────────────────────────────────────────
    wps     = generate_centerline_waypoints()
    raw     = _generate_centerline_raw(wps, pts_per_meter=30)
    # 열린 루프 리샘플
    center  = resample_polyline(raw, ds=0.02)

    # 트랙 총 길이
    track_len = sum(
        math.hypot(center[i+1][0] - center[i][0],
                   center[i+1][1] - center[i][1])
        for i in range(len(center) - 1)
    )
    print(f"[INFO] 중심선 샘플 수: {len(center)}, 총 호장: {track_len:.2f} m")

    # 자기교차 검증
    validate_no_self_intersection(center, min_dist=0.80)

    # 트랙 마진 확인 (외곽선 포함)
    xs = [p[0] for p in center]
    ys = [p[1] for p in center]
    print(f"[INFO] 중심선 범위: x∈[{min(xs):.2f}, {max(xs):.2f}], "
          f"y∈[{min(ys):.2f}, {max(ys):.2f}]")
    print(f"[INFO] 외곽선 범위: x∈[{min(xs)-HALF_SEP:.2f}, {max(xs)+HALF_SEP:.2f}], "
          f"y∈[{min(ys)-HALF_SEP:.2f}, {max(ys)+HALF_SEP:.2f}]")

    # ─── 5) 차선 마킹: 좌/우 흰색 실선 2줄 ────────────────────────────────
    UsdGeom.Scope.Define(stage, "/World/Lanes")

    left_pts  = offset_polyline(center, +HALF_SEP)   # CCW 좌측 (+0.30 m)
    right_pts = offset_polyline(center, -HALF_SEP)   # CCW 우측 (-0.30 m)

    # 닫힌 루프: 마지막에 첫 점 추가
    left_closed  = left_pts  + [left_pts[0]]
    right_closed = right_pts + [right_pts[0]]

    # 재료 사전 등록
    _mat(stage, "lane_white", WHITE, rough=0.4, emit=EW)

    n_left  = spawn_lane_segments(
        stage, left_closed,  "/World/Lanes/Left",  "lane_white")
    n_right = spawn_lane_segments(
        stage, right_closed, "/World/Lanes/Right", "lane_white")
    print(f"[INFO] 차선 세그먼트: Left={n_left}, Right={n_right}, "
          f"합계={n_left + n_right}")

    # ─── 6) 출발선 (P0 부근, y ≈ -3.6, 굵은 가로선) ───────────────────────
    # 차선 진행 방향(동) ⊥ 출발선(남북) → sx=두께(0.04), sy=폭(0.60)
    _box(stage, "/World/Lanes/StartLine",
         -2.0, -3.6, MZ,
         0.04, LANE_SEP, MH,
         "lane_white", WHITE, rough=0.4, emit=EW)

    # ─── 7) 장애물 #1 — Bottom 직선 흰색 큐브 (지그재그) ────────────────────
    UsdGeom.Scope.Define(stage, "/World/Obstacles")
    for nm, ox, oy, sx, sy, sz in OBS_CUBES:
        _box(stage, f"/World/Obstacles/{nm}",
             ox, oy, sz / 2,  # cz = 절반 높이 (바닥 접지)
             sx, sy, sz,
             "obs_cube_white", WHITE, rough=0.5, phys=True)

    # ─── 8) 장애물 #2 — Top 직선 주황 PVC 콘 ────────────────────────────────
    for nm, ox, oy, r, h in OBS_CONES:
        _cyl(stage, f"/World/Obstacles/{nm}",
             ox, oy, h / 2,   # cz = 절반 높이
             r, h,
             "obs_cone_orange", ORANGE, rough=0.5, phys=True)

    # ─── 완료 로그 ──────────────────────────────────────────────────────────
    print(f"[INFO] 장애물: 큐브 {len(OBS_CUBES)}개, 콘 {len(OBS_CONES)}개")
    print(f"[INFO] 씬 구성 완료 ─ "
          f"트랙 {track_len:.1f} m | "
          f"차선 마킹 {n_left+n_right}개 세그먼트 | "
          f"장애물 {len(OBS_CUBES)+len(OBS_CONES)}개")


# ═══════════════════════════════════════════════════════════════════════════════
# ─── 카메라 + 시뮬레이션 시작 ─────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

build_scene()

for _ in range(10):
    simulation_app.update()


def setup_camera():
    """카메라를 (0, 0, 9) 수직하향으로 설정 — 6×9 m 맵 전체 가시."""
    try:
        import omni.kit.viewport.utility as vu
        vp = vu.get_active_viewport()
        if not vp:
            return
        cp = omni.usd.get_context().get_stage().GetPrimAtPath(vp.camera_path)
        if cp.IsValid():
            xf = UsdGeom.XformCommonAPI(cp)
            xf.SetTranslate(Gf.Vec3d(0.0, 0.0, 9.0))
            xf.SetRotate(Gf.Vec3f(-90.0, 0.0, 0.0))
            print("[INFO] 카메라: (0, 0, 9) 수직하향")
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
