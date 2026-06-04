"""
generate_synthetic_data.py — Isaac Replicator 차선 인식 합성 데이터 생성
=======================================================================
트랙 씬을 구성하고 가상 카메라로 촬영한 RGB 이미지와
시맨틱 시그먼테이션 마스크를 YOLOv8-seg 레이블로 변환·저장한다.

실행:
  /home/linux/isaac_env/bin/python isaac_sim/generate_synthetic_data.py
  HEADLESS=0 /home/linux/isaac_env/bin/python isaac_sim/generate_synthetic_data.py  # GUI 확인

출력:
  data/synthetic/images/{train,val}/*.png
  data/synthetic/labels/{train,val}/*.txt
  data/synthetic/dataset.yaml
"""

import os
import sys
import math
import random
import numpy as np

# RTX 5070 Ti(Blackwell) cold-start 충돌 우회 + ROS2 브리지 비활성화
# run_track_sim.py 와 동일하게 Storm(pxr) 렌더러 사용 → 합성 데이터 외관 일치
sys.argv += [
    "--/app/extensions/excluded/0=omni.graph.image.core",
    "--/app/extensions/excluded/1=isaacsim.ros2.bridge",
    "--/app/extensions/excluded/2=omni.hydra.rtx",
    "--/renderer/active=pxr",
]

from isaacsim import SimulationApp

HEADLESS = os.environ.get("HEADLESS", "1") != "0"  # 580 드라이버: headless 배치 생성
simulation_app = SimulationApp({
    "headless": HEADLESS,
    "experience": "/home/linux/isaac_env/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.full.kit",
    "width":  640,
    "height": 480,
})

import omni.usd
import omni.kit.commands
import omni.replicator.core as rep
from pxr import UsdGeom, UsdLux, UsdShade, UsdPhysics, Gf, Sdf

try:
    import cv2
except ImportError:
    raise SystemExit("[ERROR] opencv-python 미설치. pip install opencv-python-headless")

# ─── 경로 ─────────────────────────────────────────────────────────────────────
ROOT    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(ROOT, "data", "synthetic")
IMG_W, IMG_H = 640, 480
# N_TOTAL 환경변수로 재정의 가능: N_TOTAL=10 python generate_synthetic_data.py
N_TOTAL   = int(os.environ.get("N_TOTAL", "500"))
VAL_RATIO = 0.2
CAMERA_H  = 0.12  # m (TurtleBot3 카메라 높이)
PITCH_DEG = -15.0 # 카메라 앞쪽 하향 경사 (도)
MIN_MASK_AREA  = 8    # 유효 마스크 최소 픽셀 면적
LANE_WHITE_THR = int(os.environ.get("LANE_WHITE_THR", "150"))  # 차선 마킹 흰색 임계값

for split in ("train", "val"):
    os.makedirs(os.path.join(OUT_DIR, "images", split), exist_ok=True)
    os.makedirs(os.path.join(OUT_DIR, "labels", split), exist_ok=True)

# 단일 클래스: 차선(lane) — 흰색 픽셀 임계값으로 RGB 이미지에서 직접 추출
# 차선인가 아닌가 2값 분류이므로 stop_line 별도 레이블 불필요
CLASS_NAMES = ["lane"]

# ─── 트랙 파라미터 (run_track_sim.py v8 동일) ────────────────────────────────
LANE = 0.16; TW = LANE * 2; LW = 0.02; EDGE = LANE - LW / 2
TH = 0.003; TZ = TH / 2; MH = 0.001; MZ = 0.005
BY = -1.87; RX = 1.80; TY = 8.13; LX = -1.80
CY = (BY + TY) / 2
BL = abs(RX - LX); VL = abs(TY - BY)
R_CORNER = TW; R_IN = R_CORNER - EDGE; R_OUT = R_CORNER + EDGE
BL_inner = BL - 2 * R_CORNER; VL_inner = VL - 2 * R_CORNER
pvSE = (RX - R_CORNER, BY + R_CORNER)
pvNE = (RX - R_CORNER, TY - R_CORNER)
pvNW = (LX + R_CORNER, TY - R_CORNER)
pvSW = (LX + R_CORNER, BY + R_CORNER)
EW = (0.7, 0.7, 0.7)
ASPHALT = (0.05, 0.05, 0.05); WHITE = (1.00, 1.00, 1.00)
DASH_ON = 0.06; DASH_OFF = 0.06; N_ARC = 14

# ─── 재료 캐시 ────────────────────────────────────────────────────────────────
_mats: dict = {}


def _mat(stage, key, rgb, rough=0.8, emit=None):
    if key in _mats:
        return _mats[key]
    p   = f"/World/Mats/M{len(_mats):03d}"
    mat = UsdShade.Material.Define(stage, p)
    sh  = UsdShade.Shader.Define(stage, f"{p}/Sh")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness",    Sdf.ValueTypeNames.Float  ).Set(rough)
    if emit is not None:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emit))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    _mats[key] = mat
    return mat


def _bind(stage, prim, key, rgb, rough=0.8, emit=None):
    _mat(stage, key, rgb, rough, emit)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(_mats[key])


def _box(stage, path, cx, cy, cz, sx, sy, sz, mkey, rgb,
         rough=0.8, emit=None, phys=False, rz=None):
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


# ─── 시맨틱 레이블 ──────────────────────────────────────────────────────────────
def _add_semantics(stage, scope_path: str, label: str, sem_type: str = "class"):
    """USD 프림(과 모든 자식)에 시맨틱 레이블 추가 — Replicator 시그먼테이션 전용."""
    def _tag(prim):
        try:
            prim.CreateAttribute(
                "semantics:labels:Semantics:semantic_type",
                Sdf.ValueTypeNames.Token, custom=True
            ).Set(sem_type)
            prim.CreateAttribute(
                "semantics:labels:Semantics:semantic_label",
                Sdf.ValueTypeNames.Token, custom=True
            ).Set(label)
        except Exception:
            pass  # 이미 존재하는 속성이면 무시
        for child in prim.GetChildren():
            _tag(child)

    root = stage.GetPrimAtPath(scope_path)
    if root.IsValid():
        _tag(root)


# ─── 씬 구성 ──────────────────────────────────────────────────────────────────
def build_scene():
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Scope.Define(stage, "/World/Mats")

    # Physics + Ground
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)
    omni.kit.commands.execute(
        "AddGroundPlaneCommand", stage=stage, planePath="/World/GroundPlane",
        axis="Z", size=30.0, position=Gf.Vec3f(0, 0, 0),
        color=Gf.Vec3f(0.02, 0.02, 0.02),  # 거의 검정 — 아스팔트 트랙과 일치
    )

    # 조명 — DomeLight 50~200, RectLight 1000~5000 (루프 내에서 강도 랜덤화)
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(120.0)
    rect = UsdLux.RectLight.Define(stage, "/World/RectLight")
    rect.CreateIntensityAttr(3000.0)
    rect.CreateWidthAttr(8.0); rect.CreateHeightAttr(14.0)
    UsdGeom.XformCommonAPI(rect).SetTranslate(Gf.Vec3d(0, 0, 8))   # run_track_sim.py 와 동일: Y=0
    UsdGeom.XformCommonAPI(rect).SetRotate(Gf.Vec3f(-90, 0, 0))

    # 아스팔트 바닥 (트랙 전체 커버)
    _box(stage, "/World/Ground", 0, CY, -0.005, 6.0, 14.0, 0.01,
         "asphalt", ASPHALT, rough=0.9)

    # ── 트랙 표면
    UsdGeom.Scope.Define(stage, "/World/Track")
    for nm, cx, cy, sx, sy in [
        ("Bot",   0.0, BY,  BL_inner, TW),
        ("Right", RX,  CY,  TW,       VL_inner),
        ("Top",   0.0, TY,  BL_inner, TW),
        ("Left",  LX,  CY,  TW,       VL_inner),
    ]:
        _box(stage, f"/World/Track/{nm}", cx, cy, TZ, sx, sy, TH,
             "track", ASPHALT, rough=0.9)
    for nm, pvx, pvy, sx_s, sy_s in [
        ("CSE", pvSE[0], pvSE[1],  1, -1), ("CNE", pvNE[0], pvNE[1],  1,  1),
        ("CNW", pvNW[0], pvNW[1], -1,  1), ("CSW", pvSW[0], pvSW[1], -1, -1),
    ]:
        _box(stage, f"/World/Track/{nm}",
             pvx + sx_s * R_OUT / 2, pvy + sy_s * R_OUT / 2, TZ,
             R_OUT, R_OUT, TH, "track", ASPHALT, rough=0.9)

    # ── 차선 마킹 (흰색 실선 2줄, 중앙선·정지선·내부 도로 없음)
    UsdGeom.Scope.Define(stage, "/World/Marks")

    def _solid(path, cx, cy, sx, sy):
        _box(stage, path, cx, cy, MZ, sx, sy, MH, "mk_white", WHITE, rough=0.05, emit=EW)

    UsdGeom.Scope.Define(stage, "/World/Marks/Straight")
    _solid("/World/Marks/Straight/BS",  0.0,       BY - EDGE, BL_inner, LW)
    _solid("/World/Marks/Straight/BN",  0.0,       BY + EDGE, BL_inner, LW)
    _solid("/World/Marks/Straight/RE",  RX + EDGE, CY,        LW, VL_inner)
    _solid("/World/Marks/Straight/RW",  RX - EDGE, CY,        LW, VL_inner)
    _solid("/World/Marks/Straight/TN",  0.0,       TY + EDGE, BL_inner, LW)
    _solid("/World/Marks/Straight/TS",  0.0,       TY - EDGE, BL_inner, LW)
    _solid("/World/Marks/Straight/LW",  LX - EDGE, CY,        LW, VL_inner)
    _solid("/World/Marks/Straight/LE",  LX + EDGE, CY,        LW, VL_inner)

    UsdGeom.Scope.Define(stage, "/World/Marks/Corners")

    def _corner_2arcs(scope, pvx, pvy, a0_deg, a1_deg):
        span = a1_deg - a0_deg
        for r, key in [(R_IN, "in"), (R_OUT, "out")]:
            for i in range(N_ARC + 1):
                t = i / N_ARC
                ad = a0_deg + t * span
                ar = math.radians(ad)
                sl = max(r * math.radians(abs(span)) / N_ARC * 1.2, LW * 2)
                _box(stage, f"{scope}/{key}_{i:02d}",
                     pvx + r * math.cos(ar), pvy + r * math.sin(ar), MZ,
                     LW, sl, MH, "mk_white", WHITE, rough=0.05, emit=EW, rz=ad)

    for cname, (pvx, pvy), a0, a1 in [
        ("SE", pvSE, 270.0, 360.0), ("NE", pvNE, 0.0,  90.0),
        ("NW", pvNW,  90.0, 180.0), ("SW", pvSW, 180.0, 270.0),
    ]:
        sp = f"/World/Marks/Corners/{cname}"
        UsdGeom.Scope.Define(stage, sp)
        _corner_2arcs(sp, pvx, pvy, a0, a1)

    # ── 장애물 (semantic 없음 — 레이블 불필요)
    # run_track_sim.py 와 동일하게 RED 사용 → 흰 차선 마킹과 혼동 방지
    RED = (1.00, 0.00, 0.00)
    UsdGeom.Scope.Define(stage, "/World/Obstacles")
    OBH = 0.25; OBS = 0.05
    for pname, ox, oy in [
        ("Obs0",  -1.00, TY + LANE * 0.60),
        ("Obs1",   0.00, TY - LANE * 0.60),
        ("Obs2",  +1.00, TY + LANE * 0.60),
        ("ObsL1", LX - LANE * 0.60,   0.0),
        ("ObsL2", LX + LANE * 0.60,  +3.0),
    ]:
        _box(stage, f"/World/Obstacles/{pname}",
             ox, oy, OBH / 2, OBS, OBS, OBH, "obs_red", RED, phys=True)

    # ── 시맨틱 레이블 부여
    _add_semantics(stage, "/World/Marks", "white_lane")

    print("[INFO] 씬 구성 완료 (run_track_sim.py v8 동일)")


# ─── 트랙 웨이포인트 ──────────────────────────────────────────────────────────
def get_track_waypoints():
    """트랙 중심선 위 (x, y, yaw_deg) 웨이포인트 리스트 (CCW 한 바퀴)."""
    pts = []

    def _straight_pts(x0, y0, x1, y1, heading, n=20):
        for i in range(n):
            t = i / n
            pts.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0), heading))

    def _arc_pts(pvx, pvy, a0, a1, n=10):
        for i in range(n):
            t  = i / n
            a  = math.radians(a0 + t * (a1 - a0))
            yaw = (math.degrees(a) + 90.0) % 360.0
            pts.append((pvx + R_CORNER * math.cos(a),
                         pvy + R_CORNER * math.sin(a), yaw))

    # Bottom (east), SE corner, Right (north), NE corner,
    # Top (west), NW corner, Left (south), SW corner
    _straight_pts(LX + R_CORNER, BY, RX - R_CORNER, BY, 0.0,   20)
    _arc_pts(*pvSE, 270.0, 360.0, 10)
    _straight_pts(RX, BY + R_CORNER, RX, TY - R_CORNER, 90.0,  25)
    _arc_pts(*pvNE,   0.0,  90.0, 10)
    _straight_pts(RX - R_CORNER, TY, LX + R_CORNER, TY, 180.0, 20)
    _arc_pts(*pvNW,  90.0, 180.0, 10)
    _straight_pts(LX, TY - R_CORNER, LX, BY + R_CORNER, 270.0, 25)
    _arc_pts(*pvSW, 180.0, 270.0, 10)

    return pts


# ─── RGB 이미지 → YOLOv8-detect 레이블 변환 ──────────────────────────────────
def rgb_to_detect_lines(img_bgr: np.ndarray) -> list[str]:
    """
    BGR 이미지에서 흰색 픽셀(차선 마킹)을 추출하여 YOLOv8-detect 레이블을 생성한다.
    이미지를 좌/우로 절반 분할하여 각각의 bbox를 독립 객체로 출력한다.
      - 좌측 bbox: 이미지 왼쪽 절반의 흰색 픽셀 영역 → class 0 (left lane)
      - 우측 bbox: 이미지 오른쪽 절반의 흰색 픽셀 영역 → class 0 (right lane)

    추론 시 좌/우 bbox 중심의 x 좌표 평균이 로봇의 목표 경로 중심이 된다.
    """
    h, w = img_bgr.shape[:2]
    roi_top = int(h * LANE_ROI_TOP_RATIO)  # ROI: 하단 45% 만 사용

    # 흰색 마스크: 전체 이미지
    full_mask = np.all(img_bgr > LANE_WHITE_THR, axis=2)

    lines = []
    for x_start, x_end in [(0, w // 2), (w // 2, w)]:
        half = full_mask[roi_top:, x_start:x_end]
        ys, xs = np.where(half)
        if xs.size < MIN_MASK_AREA:
            continue
        min_x_h = int(xs.min()) + x_start
        max_x_h = int(xs.max()) + x_start
        min_y_h = int(ys.min()) + roi_top
        max_y_h = int(ys.max()) + roi_top

        cx = (min_x_h + max_x_h) / 2 / w
        cy = (min_y_h + max_y_h) / 2 / h
        bw = (max_x_h - min_x_h) / w
        bh = (max_y_h - min_y_h) / h
        lines.append(f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
    return lines


LANE_ROI_TOP_RATIO = 0.55  # ROI 상단 비율 (상위 55% 무시)


# ─── 메인 ────────────────────────────────────────────────────────────────────
def main():
    build_scene()
    for _ in range(20):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    waypoints = get_track_waypoints()

    # 가상 카메라 — IMX219 파라미터 (run_track_sim.py 의 실제 카메라와 일치)
    # 3.04 mm 렌즈 + 1/4" 센서(3.68×2.76 mm) → H-FOV ≈ 62.2°, V-FOV ≈ 48.8°
    # rep.create.camera() 는 Isaac Sim 5.1 에서 vertical_aperture 를 지원하지 않으므로
    # UsdGeom.Camera.Define() 으로 직접 생성한다 (run_track_sim.py 방식과 동일).
    cam_prim_path = "/World/SyntheticCam"
    cam_usd = UsdGeom.Camera.Define(stage, cam_prim_path)
    cam_usd.CreateFocalLengthAttr(3.04)
    cam_usd.CreateHorizontalApertureAttr(3.68)
    cam_usd.CreateVerticalApertureAttr(2.76)
    cam_usd.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10.0))

    cam_prim = stage.GetPrimAtPath(cam_prim_path)
    # USD XYZ Euler 변환: SetRotate(a, b, c) = Rz(c)*Ry(b)*Rx(a)
    # 카메라 -Z 시선 → 전방(yaw)+하방(pitch) 방향으로 보내는 올바른 각도:
    #   a = 90+pitch_deg  (예: pitch=-15° → a=75°)
    #   c = yaw_deg - 90
    # 검증: yaw=0(동), pitch=-15 → SetRotate(75,0,-90) →
    #   Rz(-90)*Rx(75)*(0,0,-1) = (cos15, 0, -sin15) = 전방+하방15° ✓
    xf0 = UsdGeom.XformCommonAPI(cam_usd)
    xf0.SetTranslate(Gf.Vec3d(waypoints[0][0], waypoints[0][1], CAMERA_H))
    xf0.SetRotate(Gf.Vec3f(90.0 + PITCH_DEG, 0.0, waypoints[0][2] - 90.0))

    for _ in range(5):
        simulation_app.update()

    rp = rep.create.render_product(cam_prim_path, (IMG_W, IMG_H))

    # RGB annotator 만 사용 — 시맨틱 어노테이터 불필요 (RGB 임계값으로 직접 레이블 생성)
    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annot.attach([rp])

    # headless 모드에서 Replicator 파이프라인 초기화
    # simulation_app.update() 만으로는 annotator 데이터가 채워지지 않으므로
    # rep.orchestrator.step() 으로 실제 렌더 파이프라인을 명시적으로 실행한다.
    for _ in range(3):
        rep.orchestrator.step(pause_timeline=False)

    print(f"[INFO] 카메라 프림: {cam_prim_path}")

    dome_prim = stage.GetPrimAtPath("/World/DomeLight")
    rect_prim = stage.GetPrimAtPath("/World/RectLight")

    val_count   = int(N_TOTAL * VAL_RATIO)
    train_count = N_TOTAL - val_count
    split_list  = ["train"] * train_count + ["val"] * val_count
    random.shuffle(split_list)

    saved = 0
    for idx, split in enumerate(split_list):
        # 웨이포인트 선택 (순환 + 랜덤 오프셋 — yolov8s 640 학습용 다양화 강화)
        wp = waypoints[idx % len(waypoints)]
        rx = wp[0] + random.uniform(-0.10, 0.10)        # x 변이 (±0.04 → ±0.10)
        ry = wp[1] + random.uniform(-0.10, 0.10)        # y 변이 (±0.04 → ±0.10)
        rz_cam = CAMERA_H + random.uniform(-0.03, 0.03) # 높이 변이 (±0.01 → ±0.03)
        yaw_deg = wp[2] + random.uniform(-15.0, 15.0)   # yaw 변이 (±8° → ±15°)
        pitch_deg = PITCH_DEG + random.uniform(-8.0, 8.0)  # pitch 변이 (±5° → ±8°)

        # 카메라 위치·방향 업데이트
        # SetRotate(90+pitch, 0, yaw-90): USD XYZ Euler 에서 올바른 전방+하방 시선
        xf = UsdGeom.XformCommonAPI(cam_prim)
        xf.SetTranslate(Gf.Vec3d(rx, ry, rz_cam))
        xf.SetRotate(Gf.Vec3f(90.0 + pitch_deg, 0.0, yaw_deg - 90.0))

        # 조명 도메인 랜덤화 — 강도 범위 확대 + 위치/색온도 다양화
        UsdLux.DomeLight(dome_prim).CreateIntensityAttr().Set(
            random.uniform(30, 400)         # 50~200 → 30~400
        )
        UsdLux.RectLight(rect_prim).CreateIntensityAttr().Set(
            random.uniform(500, 6000)       # 1000~5000 → 500~6000
        )
        # RectLight 위치 Y 다양화 (그림자/하이라이트 방향 변화)
        UsdGeom.XformCommonAPI(rect_prim).SetTranslate(
            Gf.Vec3d(0, random.uniform(-3.0, 3.0), 8)
        )
        # RectLight 색온도 다양화 (형광등/일광/저녁 시뮬, RGB ±15%)
        rect_r = 1.0 + random.uniform(-0.15, 0.15)
        rect_g = 1.0 + random.uniform(-0.15, 0.15)
        rect_b = 1.0 + random.uniform(-0.15, 0.15)
        UsdLux.RectLight(rect_prim).CreateColorAttr().Set(
            Gf.Vec3f(max(0.0, rect_r), max(0.0, rect_g), max(0.0, rect_b))
        )

        # 렌더 파이프라인 실행 — rep.orchestrator.step() 으로 annotator 데이터 갱신
        rep.orchestrator.step(pause_timeline=False)

        # RGB 데이터 수집
        rgb_data = rgb_annot.get_data()

        if rgb_data is None:
            continue  # 데이터 없음: 건너뜀

        # Isaac Sim 버전에 따라 dict 또는 ndarray 직접 반환
        img_rgba = rgb_data.get("data") if isinstance(rgb_data, dict) else rgb_data
        if img_rgba is None or not hasattr(img_rgba, "shape") or img_rgba.shape == (0,):
            continue

        img_bgr    = cv2.cvtColor(img_rgba[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR)
        yolo_lines = rgb_to_detect_lines(img_bgr)   # 노이즈 적용 전 깨끗한 이미지로 라벨 생성

        # ── 이미지 후처리 다양화 (실물 카메라 특성 시뮬) ──────────────
        # ① 가우시안 노이즈 (σ 0~5, IMX219 ISO 노이즈 반영)
        noise_sigma = random.uniform(0.0, 5.0)
        if noise_sigma > 0.5:
            noise = np.random.normal(0, noise_sigma, img_bgr.shape).astype(np.int16)
            img_bgr = np.clip(img_bgr.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        # ② 30% 확률 가벼운 가우시안 블러 (카메라 흔들림/모션 블러 반영)
        if random.random() < 0.30:
            img_bgr = cv2.GaussianBlur(img_bgr, (3, 3), 0)

        stem    = f"{idx:05d}"
        img_dir = os.path.join(OUT_DIR, "images", split)
        lbl_dir = os.path.join(OUT_DIR, "labels", split)
        cv2.imwrite(os.path.join(img_dir, f"{stem}.png"), img_bgr)
        with open(os.path.join(lbl_dir, f"{stem}.txt"), "w") as f:
            f.write("\n".join(yolo_lines))

        saved += 1
        if saved % 50 == 0:
            print(f"[INFO] {saved}/{N_TOTAL} 저장 완료 (split={split})")

    # dataset.yaml 작성
    yaml_path = os.path.join(OUT_DIR, "dataset.yaml")
    with open(yaml_path, "w") as f:
        f.write(f"path: {OUT_DIR}\n")
        f.write("train: images/train\n")
        f.write("val:   images/val\n\n")
        f.write(f"nc: {len(CLASS_NAMES)}\n")
        f.write("names:\n")
        for i, name in enumerate(CLASS_NAMES):
            f.write(f"  {i}: {name}\n")
    print(f"[INFO] dataset.yaml 저장: {yaml_path}")
    print(f"[INFO] 완료 — 총 {saved}개 이미지 생성")

    simulation_app.close()


if __name__ == "__main__":
    main()
