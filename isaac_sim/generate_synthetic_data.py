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

# RTX 5070 Ti(Blackwell) cold-start 충돌 우회
sys.argv += ["--/app/extensions/excluded/0=omni.graph.image.core"]

from isaacsim import SimulationApp

HEADLESS = os.environ.get("HEADLESS", "1") != "0"
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
N_TOTAL   = 500   # 총 생성 이미지 수
VAL_RATIO = 0.2
CAMERA_H  = 0.12  # m (TurtleBot3 카메라 높이)
PITCH_DEG = -15.0 # 카메라 앞쪽 하향 경사 (도)
MIN_MASK_AREA = 8  # 유효 마스크 최소 픽셀 면적

for split in ("train", "val"):
    os.makedirs(os.path.join(OUT_DIR, "images", split), exist_ok=True)
    os.makedirs(os.path.join(OUT_DIR, "labels", split), exist_ok=True)

CLASS_NAMES = ["white_lane", "stop_line"]

# ─── 트랙 파라미터 (run_track_sim.py 동일) ────────────────────────────────────
LANE = 0.22; TW = LANE * 2; LW = 0.02; EDGE = LANE - LW / 2
TH = 0.003; TZ = TH / 2; MH = 0.001; MZ = 0.005
BY = -5.00; RX = 1.80; TY = 5.00; LX = -1.80
CY = (BY + TY) / 2
BL = abs(RX - LX); VL = abs(TY - BY)
R_CORNER = TW; R_IN = R_CORNER - EDGE; R_OUT = R_CORNER + EDGE
BL_inner = BL - 2 * R_CORNER; VL_inner = VL - 2 * R_CORNER
pvSE = (RX - R_CORNER, BY + R_CORNER)
pvNE = (RX - R_CORNER, TY - R_CORNER)
pvNW = (LX + R_CORNER, TY - R_CORNER)
pvSW = (LX + R_CORNER, BY + R_CORNER)
IH_X0 = LX + TW / 2; IH_X1 = RX - TW / 2
CROSS_H = TW / 2; IH_SEG = -CROSS_H - IH_X0
IV_Y0 = BY + TW / 2; IV_Y1 = TY - TW / 2
CROSS_V = TW / 2
IH_CX_W = (IH_X0 + (-CROSS_H)) / 2; IH_CX_E = (CROSS_H + IH_X1) / 2
IV_CY_S = (IV_Y0 + (CY - CROSS_V)) / 2; IV_CY_N = ((CY + CROSS_V) + IV_Y1) / 2
H2_Y = IV_CY_S; H3_Y = IV_CY_N
SL_OFF = 0.10; SL_THK = 0.04
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
        color=Gf.Vec3f(0.20, 0.20, 0.20),
    )

    # 조명 — 기준값 (루프 내에서 강도만 조정)
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(600.0)
    rect = UsdLux.RectLight.Define(stage, "/World/RectLight")
    rect.CreateIntensityAttr(28000.0)
    rect.CreateWidthAttr(8.0); rect.CreateHeightAttr(14.0)
    UsdGeom.XformCommonAPI(rect).SetTranslate(Gf.Vec3d(0, 0, 8))
    UsdGeom.XformCommonAPI(rect).SetRotate(Gf.Vec3f(-90, 0, 0))

    # 아스팔트 바닥
    _box(stage, "/World/Ground", 0, 0, -0.005, 6.0, 12.0, 0.01,
         "asphalt", ASPHALT, rough=0.9)

    # ── 트랙 직선
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

    # ── 내부 십자 도로
    UsdGeom.Scope.Define(stage, "/World/InnerRoad")
    for nm, cx, cy, sx, sy in [
        ("HW", IH_CX_W, CY, IH_SEG, TW), ("HE", IH_CX_E, CY, IH_SEG, TW),
        ("VS", 0.0, IV_CY_S, TW, abs(CY - CROSS_V - IV_Y0)),
        ("VN", 0.0, IV_CY_N, TW, abs(IV_Y1 - (CY + CROSS_V))),
        ("Cross", 0.0, CY, TW, TW),
        ("HW2", IH_CX_W, H2_Y, IH_SEG, TW), ("HE2", IH_CX_E, H2_Y, IH_SEG, TW),
        ("Cross2", 0.0, H2_Y, TW, TW),
        ("HW3", IH_CX_W, H3_Y, IH_SEG, TW), ("HE3", IH_CX_E, H3_Y, IH_SEG, TW),
        ("Cross3", 0.0, H3_Y, TW, TW),
    ]:
        _box(stage, f"/World/InnerRoad/{nm}", cx, cy, TZ, sx, sy, TH,
             "track", ASPHALT, rough=0.9)

    # ── 차선 마킹 (white_lane 클래스)
    UsdGeom.Scope.Define(stage, "/World/Marks")

    def _solid(path, cx, cy, sx, sy):
        _box(stage, path, cx, cy, MZ, sx, sy, MH, "mk_white", WHITE, rough=0.05, emit=EW)

    UsdGeom.Scope.Define(stage, "/World/Marks/Straight")
    _solid("/World/Marks/Straight/BS",    0.0,         BY - EDGE, BL_inner, LW)
    _solid("/World/Marks/Straight/BN",    0.0,         BY + EDGE, BL_inner, LW)
    _solid("/World/Marks/Straight/RE",    RX + EDGE,   CY,        LW, VL_inner)
    _solid("/World/Marks/Straight/RW",    RX - EDGE,   CY,        LW, VL_inner)
    _solid("/World/Marks/Straight/TN",    0.0,         TY + EDGE, BL_inner, LW)
    _solid("/World/Marks/Straight/TS",    0.0,         TY - EDGE, BL_inner, LW)
    _solid("/World/Marks/Straight/LW",    LX - EDGE,   CY,        LW, VL_inner)
    _solid("/World/Marks/Straight/LE",    LX + EDGE,   CY,        LW, VL_inner)
    _solid("/World/Marks/Straight/Start", 0.0,         BY,        LW * 4, TW)

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

    UsdGeom.Scope.Define(stage, "/World/Marks/Inner")
    for nm, cx, cy, sx, sy in [
        ("HW/S", IH_CX_W, CY - EDGE, IH_SEG, LW), ("HW/N", IH_CX_W, CY + EDGE, IH_SEG, LW),
        ("HE/S", IH_CX_E, CY - EDGE, IH_SEG, LW), ("HE/N", IH_CX_E, CY + EDGE, IH_SEG, LW),
        ("VS/W", -EDGE, IV_CY_S, LW, abs(CY - CROSS_V - IV_Y0)),
        ("VS/E",  EDGE, IV_CY_S, LW, abs(CY - CROSS_V - IV_Y0)),
        ("VN/W", -EDGE, IV_CY_N, LW, abs(IV_Y1 - (CY + CROSS_V))),
        ("VN/E",  EDGE, IV_CY_N, LW, abs(IV_Y1 - (CY + CROSS_V))),
    ]:
        parent = f"/World/Marks/Inner/{nm.split('/')[0]}"
        UsdGeom.Scope.Define(stage, parent)
        _solid(f"/World/Marks/Inner/{nm}", cx, cy, sx, sy)

    # ── 정지선 (stop_line 클래스)
    UsdGeom.Scope.Define(stage, "/World/Track/StopLines")
    for nm, cx, cy, sx, sy in [
        ("stop_N",  0.0,                 CY + CROSS_H + SL_OFF, TW,     SL_THK),
        ("stop_S",  0.0,                 CY - CROSS_H - SL_OFF, TW,     SL_THK),
        ("stop_E",  CROSS_H + SL_OFF,    CY,                    SL_THK, TW    ),
        ("stop_W", -(CROSS_H + SL_OFF),  CY,                    SL_THK, TW    ),
    ]:
        _box(stage, f"/World/Track/StopLines/{nm}",
             cx, cy, MZ, sx, sy, MH, "mk_white", WHITE, rough=0.4, emit=EW)

    # ── 장애물 (semantic 없음 — 레이블 불필요)
    UsdGeom.Scope.Define(stage, "/World/Obstacles")
    for pname, ox, oy in [
        ("Obs0", -1.00, TY + LANE * 0.60), ("Obs1",  0.00, TY - LANE * 0.60),
        ("Obs2", +1.00, TY + LANE * 0.60),
        ("ObsL0", LX + LANE * 0.60, -3.0), ("ObsL1", LX - LANE * 0.60, 0.0),
        ("ObsL2", LX + LANE * 0.60, +3.0),
    ]:
        _box(stage, f"/World/Obstacles/{pname}",
             ox, oy, 0.15, 0.15, 0.15, 0.30, "obs_cube", WHITE, phys=True)

    # ── 시맨틱 레이블 부여
    _add_semantics(stage, "/World/Marks",              "white_lane")
    _add_semantics(stage, "/World/Track/StopLines",    "stop_line")

    print("[INFO] 씬 구성 완료 + 시맨틱 레이블 적용")


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


# ─── 세그먼테이션 → YOLO 변환 ──────────────────────────────────────────────────
def seg_to_yolo_lines(seg_output: dict) -> list[str]:
    """Replicator 시맨틱 출력 → YOLOv8-seg 레이블 줄 목록."""
    data        = seg_output.get("data")         # (H, W) uint32
    id_to_labels = seg_output.get("info", {}).get("idToLabels", {})

    if data is None or data.size == 0:
        return []

    sem_id_to_class: dict[int, int] = {}
    for sid_str, label_dict in id_to_labels.items():
        lbl = label_dict.get("class", "")
        if lbl == "white_lane":
            sem_id_to_class[int(sid_str)] = 0
        elif lbl == "stop_line":
            sem_id_to_class[int(sid_str)] = 1

    lines = []
    for sem_id, class_id in sem_id_to_class.items():
        mask = (data == sem_id).astype(np.uint8) * 255
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in cnts:
            if cv2.contourArea(cnt) < MIN_MASK_AREA:
                continue
            eps   = max(0.01 * cv2.arcLength(cnt, True), 0.5)
            approx = cv2.approxPolyDP(cnt, eps, True)
            if len(approx) < 3:
                continue
            pts_norm = np.clip(
                approx.reshape(-1, 2) / np.array([IMG_W, IMG_H]), 0.0, 1.0
            )
            flat = " ".join(f"{v:.6f}" for v in pts_norm.flatten())
            lines.append(f"{class_id} {flat}")
    return lines


# ─── 메인 ────────────────────────────────────────────────────────────────────
def main():
    build_scene()
    for _ in range(20):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    waypoints = get_track_waypoints()

    # 가상 카메라 (TurtleBot3 전방 카메라 모사)
    cam_prim_path = "/World/SyntheticCam"
    cam_item = rep.create.camera(
        position=(waypoints[0][0], waypoints[0][1], CAMERA_H),
        look_at=(waypoints[0][0] + 0.5, waypoints[0][1], CAMERA_H - 0.13),
        focal_length=3.0,
        horizontal_aperture=6.0,   # ≈ 90° HFOV
        vertical_aperture=4.5,
        clipping_range=(0.01, 15.0),
    )
    rp = rep.create.render_product(cam_item, (IMG_W, IMG_H))

    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    seg_annot = rep.AnnotatorRegistry.get_annotator(
        "semantic_segmentation", init_params={"colorize": False}
    )
    rgb_annot.attach([rp])
    seg_annot.attach([rp])

    # 카메라 프림 경로 확인 (rep.create.camera 가 생성하는 실제 경로 획득)
    cam_prim = None
    for p in stage.TraverseAll():
        if p.IsA(UsdGeom.Camera) and p.GetPath().pathString != "/OmniverseKit_Persp":
            cam_prim = p
            break
    if cam_prim is None:
        raise RuntimeError("[ERROR] 카메라 프림을 찾지 못했습니다.")
    cam_path = cam_prim.GetPath().pathString
    print(f"[INFO] 카메라 프림: {cam_path}")

    dome_prim = stage.GetPrimAtPath("/World/DomeLight")
    rect_prim = stage.GetPrimAtPath("/World/RectLight")

    val_count   = int(N_TOTAL * VAL_RATIO)
    train_count = N_TOTAL - val_count
    split_list  = ["train"] * train_count + ["val"] * val_count
    random.shuffle(split_list)

    saved = 0
    for idx, split in enumerate(split_list):
        # 웨이포인트 선택 (순환 + 약간의 랜덤 오프셋)
        wp = waypoints[idx % len(waypoints)]
        rx = wp[0] + random.uniform(-0.04, 0.04)
        ry = wp[1] + random.uniform(-0.04, 0.04)
        rz_cam = CAMERA_H + random.uniform(-0.01, 0.01)
        yaw_deg = wp[2] + random.uniform(-8.0, 8.0)
        pitch_deg = PITCH_DEG + random.uniform(-5.0, 5.0)

        heading_rad = math.radians(yaw_deg)
        fwd_dist = 0.5
        lx = rx + fwd_dist * math.cos(heading_rad)
        ly = ry + fwd_dist * math.sin(heading_rad)
        lz = rz_cam + fwd_dist * math.tan(math.radians(pitch_deg))

        # 카메라 위치 업데이트
        xf = UsdGeom.XformCommonAPI(cam_prim)
        xf.SetTranslate(Gf.Vec3d(rx, ry, rz_cam))
        # look_at 은 별도 SetRotate 로 근사 (pitch + yaw)
        xf.SetRotate(Gf.Vec3f(pitch_deg, 0.0, yaw_deg))

        # 조명 도메인 랜덤화
        UsdLux.DomeLight(dome_prim).CreateIntensityAttr().Set(
            random.uniform(350, 900)
        )
        UsdLux.RectLight(rect_prim).CreateIntensityAttr().Set(
            random.uniform(18000, 40000)
        )

        # 렌더 업데이트 (2회 이상 필요)
        for _ in range(3):
            simulation_app.update()

        # 데이터 수집
        rgb_data = rgb_annot.get_data()
        seg_data = seg_annot.get_data()

        if rgb_data is None or rgb_data.get("data") is None:
            print(f"[WARN] {idx}: RGB 데이터 없음, 건너뜀")
            continue

        img_rgba  = rgb_data["data"]   # (H, W, 4)
        img_bgr   = cv2.cvtColor(img_rgba, cv2.COLOR_RGBA2BGR)
        yolo_lines = seg_to_yolo_lines(seg_data) if seg_data else []

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
