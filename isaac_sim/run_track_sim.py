"""
TurtleBot3 AutoRace 2020 트랙  —  Isaac Sim Standalone (v4)
============================================================
맵     : 6 m × 6 m 아스팔트 바닥판
루프   : 3.6 m × 3.3 m 직사각형 + R=0.44 m 원호 코너 (4 코너)
차선   : 1차선 2선 시스템 @ z=0.005 m (점선 없음)
         ┌ 좌측 실선 (흰색, 0.02 m 폭)  ← 도로 중심 ±0.21 m
         └ 우측 실선 (흰색, 0.02 m 폭)
도로폭 : 0.44 m (TB3 Burger 0.14 m + 장애물 0.20 m + 여유 0.10 m)
내부망 : 십자형(+) 내부 연결로 — 수평(y=CY) + 수직(x=0) + 4-way 교차점
로봇   : USD 원점 (0.06, -1.87, -0.712)

5주차 추가:
  omni.graph.image.core 를 항상 제외하고 Replicator annotator + rclpy 직접 발행
  방식으로 /camera/image_raw 토픽을 활성화한다 (RTX 5070 Ti Blackwell 안전 우회).
  ENABLE_CAMERA=1 bash isaac_sim/launch_sim.sh  으로 카메라 토픽 활성화.
"""
import os
import sys
import math
from isaacsim import SimulationApp

# omni.graph.image.core: RTX 5070 Ti(Blackwell) cold-start segfault 우회
# 카메라는 Replicator annotator + rclpy 직접 발행으로 획득 — 이 확장 불필요
import os as _os
sys.argv += ["--/app/extensions/excluded/0=omni.graph.image.core"]
_ENABLE_CAM = _os.environ.get("ENABLE_CAMERA", "0") == "1"

simulation_app = SimulationApp({
    "headless": False,
    "experience": "/home/linux/isaac_env/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.full.kit",
})

import omni.usd
import omni.kit.commands
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


# ─── 로봇 USD 프림 경로 (모듈 레벨) ──────────────────────────────────────────
# 실제 경로는 launch 로그의 물리 경고에서 확인:
#   /World/turtlebot_tutorial_multi_sensor_publish_rates/turtlebot3_burger/base_footprint
_ROBOT_PATH = "/World/turtlebot_tutorial_multi_sensor_publish_rates/turtlebot3_burger"

# ─── 5주차: IMX219 카메라 브리지 상수 ────────────────────────────────────────
_CAM_FL   = 3.04    # 초점거리 [mm]  — IMX219 3.04 mm 렌즈
_CAM_HA   = 3.68    # 수평 센서 [mm] — 1/4" 센서 → H-FOV ≈ 62.2°
_CAM_VA   = 2.76    # 수직 센서 [mm] — 1/4" 센서 → V-FOV ≈ 48.8°
_CAM_W    = 640     # 발행 해상도 (가로)
_CAM_HT   = 480     # 발행 해상도 (세로)
_CAM_OFZ  = 0.12    # 카메라 높이 오프셋 [m]  (로봇 기준)
_CAM_OFX  = 0.05    # 카메라 전방 오프셋 [m]  (로봇 기준)
_CAM_TILT = -15.0   # 하방 경사 [deg]
_CAM_HZ   = 10      # /camera/image_raw 발행 주기 [Hz]  (시뮬 60 Hz 분주)


def _setup_cam_bridge(stage):
    """IMX219 가상 카메라 + Replicator RGB annotator + rclpy 퍼블리셔 초기화.

    카메라 USD 프림을 로봇 루트 프림 자식으로 생성 → 로봇 이동 시 자동 추종.
    omni.graph.image.core 없이 동작하므로 RTX 5070 Ti Blackwell 에서 안전.

    반환:
        (rgb_annotator, rclpy_node, img_publisher)  또는 실패 시 None
    """
    try:
        import omni.replicator.core as rep
        import rclpy
        from sensor_msgs.msg import Image as RosImage

        # ── 1) 카메라 USD 프림 — base_footprint 자식으로 생성 (물리 시뮬 추종)
        # turtlebot3_burger 루트 프림은 USD Xform 이 물리 업데이트에 갱신되지 않음.
        # 실제로 움직이는 강체는 base_footprint 이므로 그 자식으로 붙여야 카메라가 따라감.
        cam_path = _ROBOT_PATH + "/base_footprint/imx219_camera"
        cam_usd  = UsdGeom.Camera.Define(stage, cam_path)
        cam_usd.CreateFocalLengthAttr(_CAM_FL)
        cam_usd.CreateHorizontalApertureAttr(_CAM_HA)
        cam_usd.CreateVerticalApertureAttr(_CAM_VA)
        cam_usd.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10.0))

        # 로봇 로컬 좌표: 전방 _CAM_OFX m, 높이 _CAM_OFZ m
        # USD 카메라 기본: -Z 시선, +Y 이미지 up
        # base_footprint 프레임: +X=전방, +Y=좌측, +Z=위
        #
        # USD XYZ Euler (b=-90° 고정 시) image up 공식: (0, cos(a-c), sin(a-c))
        #   b=-90°: view = +X_base(전방), a와 c는 roll 보정에 사용
        #   up=+Z_base 조건: sin(a-c)=1 → c = a-90°
        #   a=0°, c=-90° → up=(0,0,1)=+Z_base ✓  image right=-Y_base(로봇 우측) ✓
        xf = UsdGeom.XformCommonAPI(cam_usd)
        xf.SetTranslate(Gf.Vec3d(_CAM_OFX, 0.0, _CAM_OFZ))
        xf.SetRotate(Gf.Vec3f(0.0, -90.0, -90.0))

        # ── 2) Replicator render product + RGB annotator
        rp  = rep.create.render_product(cam_path, (_CAM_W, _CAM_HT))
        ann = rep.AnnotatorRegistry.get_annotator("rgb")
        ann.attach([rp])

        # ── 3) rclpy 초기화 + /camera/image_raw 퍼블리셔
        try:
            rclpy.init()
        except RuntimeError:
            pass  # ROS2 브리지 확장이 이미 초기화한 경우

        # use_sim_time=True: Isaac Sim 브리지의 /clock 을 구독하여
        # TF 타임스탬프(시뮬 시간)와 이미지 타임스탬프를 동기화
        cam_node = rclpy.create_node(
            "isaac_imx219_bridge",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.Parameter.Type.BOOL,
                    True,
                )
            ],
        )
        img_pub  = cam_node.create_publisher(RosImage, "/camera/image_raw", 1)
        from sensor_msgs.msg import CameraInfo
        info_pub = cam_node.create_publisher(CameraInfo, "/camera/camera_info", 1)

        # ── 4) Static TF: base_link → camera_rgb_optical_frame
        # RViz2 Camera 디스플레이가 로봇 이동에 따라 카메라 시점을 실시간 추종하려면
        # TF 트리에 base_link → camera_rgb_optical_frame 경로가 있어야 함
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        import math as _math

        tf_static = tf2_ros.StaticTransformBroadcaster(cam_node)
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = cam_node.get_clock().now().to_msg()
        tf_msg.header.frame_id = "base_link"
        tf_msg.child_frame_id  = "camera_rgb_optical_frame"

        # 카메라 위치: 전방 _CAM_OFX m, 높이 _CAM_OFZ m
        tf_msg.transform.translation.x = _CAM_OFX
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = _CAM_OFZ

        # 카메라 방향: base_link → camera_rgb_optical_frame
        # ROS optical frame 규약: +X 우측(이미지), +Y 하방(이미지), +Z 전방(시선)
        #
        # SetRotate(0, -90°, -90°) 결과:
        #   +Z_cam(시선) = +X_base(전방)
        #   +X_cam(이미지 우) = -Y_base(로봇 우측)
        #   +Y_cam(이미지 하) = -Z_base(세계 아래)
        #
        # 표준 수평 전방 카메라 쿼터니언: [w=0.5, x=-0.5, y=0.5, z=-0.5]
        tf_msg.transform.rotation.x = -0.5
        tf_msg.transform.rotation.y =  0.5
        tf_msg.transform.rotation.z = -0.5
        tf_msg.transform.rotation.w =  0.5

        tf_static.sendTransform(tf_msg)

        # ── 5) CameraInfo 퍼블리셔 — IMX219 내부 파라미터 (핀홀 모델, 왜곡 없음)
        # fx = (focal_length / h_aperture) * width  = (3.04/3.68)*640  ≈ 528.70 px
        # fy = (focal_length / v_aperture) * height = (3.04/2.76)*480  ≈ 529.04 px
        _fx = (_CAM_FL / _CAM_HA) * _CAM_W   # ≈ 528.70
        _fy = (_CAM_FL / _CAM_VA) * _CAM_HT  # ≈ 529.04
        _cx = _CAM_W  / 2.0                   # 320.0
        _cy = _CAM_HT / 2.0                   # 240.0

        from sensor_msgs.msg import CameraInfo as _CamInfo
        _cam_info_msg = _CamInfo()
        _cam_info_msg.header.frame_id = "camera_rgb_optical_frame"
        _cam_info_msg.width           = _CAM_W
        _cam_info_msg.height          = _CAM_HT
        _cam_info_msg.distortion_model = "plumb_bob"
        _cam_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        _cam_info_msg.k = [_fx, 0.0, _cx,
                           0.0, _fy, _cy,
                           0.0, 0.0, 1.0]
        _cam_info_msg.r = [1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0]
        _cam_info_msg.p = [_fx, 0.0, _cx, 0.0,
                           0.0, _fy, _cy, 0.0,
                           0.0, 0.0, 1.0, 0.0]

        print("[CAM] IMX219 카메라 브리지 초기화 완료")
        print(f"[CAM]   USD 경로  : {cam_path}")
        print(f"[CAM]   해상도    : {_CAM_W}×{_CAM_HT}")
        print(f"[CAM]   H-FOV     : ≈ 62.2°  (focal {_CAM_FL} mm / sensor {_CAM_HA} mm)")
        print(f"[CAM]   내부파라미터: fx={_fx:.1f} fy={_fy:.1f} cx={_cx} cy={_cy}")
        print("[CAM]   ROS2 토픽 : /camera/image_raw  /camera/camera_info")
        print("[CAM]   TF        : base_link → camera_rgb_optical_frame (static)")

        return ann, cam_node, img_pub, info_pub, _cam_info_msg

    except Exception as exc:
        print(f"[CAM][ERROR] 카메라 브리지 초기화 실패: {exc}")
        return None


def _publish_cam_frame(ann, cam_node, img_pub, info_pub, cam_info_msg):
    """Replicator RGB annotator → /camera/image_raw + /camera/camera_info 동시 발행."""
    import omni.replicator.core as rep
    import numpy as np
    from sensor_msgs.msg import Image as RosImage
    import rclpy

    rep.orchestrator.step(pause_timeline=False)

    frame = ann.get_data()
    if frame is None:
        return
    rgba = frame.get("data") if isinstance(frame, dict) else frame
    if rgba is None or rgba.size == 0:
        return

    # ── 진단용 디버그 출력 (10회마다 1회) ─────────────────────────────────────
    # 카메라 월드 좌표가 바뀌는지(로봇 이동 추종 여부) + 픽셀 평균(렌더 갱신 여부) 확인
    if not hasattr(_publish_cam_frame, "_dbg_cnt"):
        _publish_cam_frame._dbg_cnt = 0
    _publish_cam_frame._dbg_cnt += 1
    if _publish_cam_frame._dbg_cnt % 10 == 1:
        try:
            import omni.usd
            from pxr import UsdGeom
            _stage = omni.usd.get_context().get_stage()
            _cp    = _stage.GetPrimAtPath(_ROBOT_PATH + "/base_footprint/imx219_camera")
            if _cp.IsValid():
                _xfc   = UsdGeom.XformCache()
                _pos   = _xfc.GetLocalToWorldTransform(_cp).ExtractTranslation()
                print(f"[CAM][D] world=({_pos[0]:.3f},{_pos[1]:.3f},{_pos[2]:.3f})"
                      f"  pixel_mean={rgba.mean():.1f}")
        except Exception:
            pass
    # ─────────────────────────────────────────────────────────────────────────

    rgb = np.ascontiguousarray(rgba[:, :, :3])   # (H, W, 3) uint8 RGB
    stamp = cam_node.get_clock().now().to_msg()

    msg                = RosImage()
    msg.header.stamp   = stamp
    msg.header.frame_id = "camera_rgb_optical_frame"
    msg.height         = rgb.shape[0]
    msg.width          = rgb.shape[1]
    msg.encoding       = "rgb8"
    msg.is_bigendian   = 0
    msg.step           = rgb.shape[1] * 3
    msg.data           = rgb.tobytes()

    cam_info_msg.header.stamp = stamp
    info_pub.publish(cam_info_msg)
    img_pub.publish(msg)
    rclpy.spin_once(cam_node, timeout_sec=0.0)


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
    # 1차선: TurtleBot3 Burger 폭(0.14 m) + 장애물(0.20 m) = 0.34 m → 여유 0.10 m 추가
    LANE = 0.22          # 도로 반폭 (TW / 2)
    TW   = LANE * 2      # 도로 총 폭 = 0.44 m
    LW   = 0.02          # 차선 마킹 폭
    EDGE = LANE - LW / 2 # 도로 중심→차선 중심 = 0.21 m

    TH = 0.003           # 트랙 슬랩 두께
    TZ = TH / 2
    MH = 0.001           # 차선 마킹 두께
    MZ = 0.005           # 차선 마킹 z 위치

    # ── 루프 중심선 좌표 (6 × 12 m 바닥판에 맞게 y 확장)
    BY = -5.00   # Bottom y (바닥 여유 ~1.0 m)
    RX =  1.80   # Right  x
    TY =  5.00   # Top    y (상단 여유 ~1.0 m)
    LX = -1.80   # Left   x
    CY = (BY + TY) / 2   # = -0.15
    BL = abs(RX - LX)    # = 3.60 m
    VL = abs(TY - BY)    # = 3.30 m

    # ── 코너 아크 반지름 (1차선: 내측·외측 2개만 사용, 중앙선 없음)
    R_CORNER = TW                  # = 0.44 m  (도로 중심선 코너 아크 반지름)
    R_IN     = R_CORNER - EDGE     # = 0.23 m  (내측 차선 아크)
    R_OUT    = R_CORNER + EDGE     # = 0.65 m  (외측 차선 아크)

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
    rect.CreateWidthAttr(8.0)    # 6 m 폭 커버
    rect.CreateHeightAttr(14.0)  # 12 m 길이 커버
    UsdGeom.XformCommonAPI(rect).SetTranslate(Gf.Vec3d(0, 0, 8))
    UsdGeom.XformCommonAPI(rect).SetRotate(Gf.Vec3f(-90, 0, 0))

    # ─── 3) 아스팔트 바닥판 (6 × 12 m) ─────────────────────────────────────
    _box(stage, "/World/Ground", 0, 0, -0.005, 6.0, 12.0, 0.01,
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

    # IV_CY_S=-2.5, IV_CY_N=+2.5 → H2/H3을 VS·VN 중앙에 배치 (T-교차점)
    H2_Y = IV_CY_S   # = -2.5
    H3_Y = IV_CY_N   # = +2.5

    for nm, cx, cy, sx, sy in [
        # 기존 중앙 십자 (y=CY=0)
        ("HW",     IH_CX_W, CY,    IH_SEG,  TW),
        ("HE",     IH_CX_E, CY,    IH_SEG,  TW),
        ("VS",     0.0,  IV_CY_S,  TW,   IV_SEG_S),
        ("VN",     0.0,  IV_CY_N,  TW,   IV_SEG_N),
        ("Cross",  0.0,  CY,       TW,   TW),
        # 추가 수평 연결로 H2 (y=-2.5, VS 중앙 교차)
        ("HW2",    IH_CX_W, H2_Y,  IH_SEG,  TW),
        ("HE2",    IH_CX_E, H2_Y,  IH_SEG,  TW),
        ("Cross2", 0.0,     H2_Y,  TW,      TW),
        # 추가 수평 연결로 H3 (y=+2.5, VN 중앙 교차)
        ("HW3",    IH_CX_W, H3_Y,  IH_SEG,  TW),
        ("HE3",    IH_CX_E, H3_Y,  IH_SEG,  TW),
        ("Cross3", 0.0,     H3_Y,  TW,      TW),
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

    # 5-a) 직선 구간 2선 (1차선: 좌측 실선 + 우측 실선, 중앙선 없음) ──────────
    UsdGeom.Scope.Define(stage, "/World/Marks/Straight")

    SL  = BL_inner   # Bottom/Top 직선 길이
    VLS = VL_inner   # Right/Left 직선 길이

    # ── Bottom (y=BY, x 방향)
    _solid("/World/Marks/Straight/BS",  0.0, BY - EDGE, SL,  LW)   # 좌측선(남)
    _solid("/World/Marks/Straight/BN",  0.0, BY + EDGE, SL,  LW)   # 우측선(북)

    # ── Right (x=RX, y 방향)
    _solid("/World/Marks/Straight/RE",  RX + EDGE, CY,  LW, VLS)   # 좌측선(동)
    _solid("/World/Marks/Straight/RW",  RX - EDGE, CY,  LW, VLS)   # 우측선(서)

    # ── Top (y=TY, x 방향): 슬라롬 장애물 구간
    _solid("/World/Marks/Straight/TN",  0.0, TY + EDGE, SL,  LW)   # 좌측선(북)
    _solid("/World/Marks/Straight/TS",  0.0, TY - EDGE, SL,  LW)   # 우측선(남)

    # ── Left (x=LX, y 방향)
    _solid("/World/Marks/Straight/LW",  LX - EDGE, CY,  LW, VLS)   # 좌측선(서)
    _solid("/World/Marks/Straight/LE",  LX + EDGE, CY,  LW, VLS)   # 우측선(동)

    # ── 시작선 (Bottom 중앙)
    _solid("/World/Marks/Straight/Start", 0.0, BY, LW * 4, TW)

    # 5-b) 코너 3선 원호 마킹 ─────────────────────────────────────────────────
    # rot_z = angle_deg 으로 설정해야 local+y 가 CCW 접선 방향과 일치
    # (rot_z = angle + 90 은 반지름 방향 → 오류)
    UsdGeom.Scope.Define(stage, "/World/Marks/Corners")

    N_ARC = 14   # 원호 분할 수

    def _corner_3arcs(scope, pvx, pvy, a0_deg, a1_deg):
        """pivot(pvx,pvy) 기준 a0→a1 도 범위의 2선 원호 마킹 (1차선).
        내측(R_IN) / 외측(R_OUT): 실선만. 중앙선 없음.
        """
        span_deg = a1_deg - a0_deg
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

    for cname, (pvx, pvy), a0, a1 in [
        ("SE", pvSE, 270.0, 360.0),
        ("NE", pvNE,   0.0,  90.0),
        ("NW", pvNW,  90.0, 180.0),
        ("SW", pvSW, 180.0, 270.0),
    ]:
        sp = f"/World/Marks/Corners/{cname}"
        UsdGeom.Scope.Define(stage, sp)
        _corner_3arcs(sp, pvx, pvy, a0, a1)

    # 5-c) 내부 십자 도로 차선 마킹 (2선, 모두 실선 — 1차선) ──────────────────
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner")

    # 수평 연결로 서쪽 (IH_W): 좌측선(S) + 우측선(N)
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/HW")
    _solid("/World/Marks/Inner/HW/S", IH_CX_W, CY - EDGE, IH_SEG, LW)
    _solid("/World/Marks/Inner/HW/N", IH_CX_W, CY + EDGE, IH_SEG, LW)

    # 수평 연결로 동쪽 (IH_E): 좌측선(S) + 우측선(N)
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/HE")
    _solid("/World/Marks/Inner/HE/S", IH_CX_E, CY - EDGE, IH_SEG, LW)
    _solid("/World/Marks/Inner/HE/N", IH_CX_E, CY + EDGE, IH_SEG, LW)

    # 수직 연결로 남쪽 (IV_S): 좌측선(W) + 우측선(E)
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/VS")
    _solid("/World/Marks/Inner/VS/W", -EDGE, IV_CY_S, LW, IV_SEG_S)
    _solid("/World/Marks/Inner/VS/E",  EDGE, IV_CY_S, LW, IV_SEG_S)

    # 수직 연결로 북쪽 (IV_N): 좌측선(W) + 우측선(E)
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/VN")
    _solid("/World/Marks/Inner/VN/W", -EDGE, IV_CY_N, LW, IV_SEG_N)
    _solid("/World/Marks/Inner/VN/E",  EDGE, IV_CY_N, LW, IV_SEG_N)

    # 추가 수평 연결로 H2 (y=H2_Y=-2.5): 좌측(S) + 우측(N) 실선
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/HW2")
    _solid("/World/Marks/Inner/HW2/S", IH_CX_W, H2_Y - EDGE, IH_SEG, LW)
    _solid("/World/Marks/Inner/HW2/N", IH_CX_W, H2_Y + EDGE, IH_SEG, LW)
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/HE2")
    _solid("/World/Marks/Inner/HE2/S", IH_CX_E, H2_Y - EDGE, IH_SEG, LW)
    _solid("/World/Marks/Inner/HE2/N", IH_CX_E, H2_Y + EDGE, IH_SEG, LW)

    # 추가 수평 연결로 H3 (y=H3_Y=+2.5): 좌측(S) + 우측(N) 실선
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/HW3")
    _solid("/World/Marks/Inner/HW3/S", IH_CX_W, H3_Y - EDGE, IH_SEG, LW)
    _solid("/World/Marks/Inner/HW3/N", IH_CX_W, H3_Y + EDGE, IH_SEG, LW)
    UsdGeom.Scope.Define(stage, "/World/Marks/Inner/HE3")
    _solid("/World/Marks/Inner/HE3/S", IH_CX_E, H3_Y - EDGE, IH_SEG, LW)
    _solid("/World/Marks/Inner/HE3/N", IH_CX_E, H3_Y + EDGE, IH_SEG, LW)

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

    # 5-e) 교차로 내부 점선 — 1차선 전환으로 제거 (점선 없음 정책)

    # ─── 6) 슬라롬 장애물 큐브 (Top 직선) ───────────────────────────────────
    UsdGeom.Scope.Define(stage, "/World/Obstacles")
    OBH = 0.30
    # ── Top 직선 장애물: x 간격 1.0 m, 지그재그 3개
    for pname, ox, oy in [
        ("Obs0", -1.00, TY + LANE * 0.60),   # 외측(북)
        ("Obs1",  0.00, TY - LANE * 0.60),   # 내측(남)
        ("Obs2", +1.00, TY + LANE * 0.60),   # 외측(북)
    ]:
        _box(stage, f"/World/Obstacles/{pname}",
             ox, oy, OBH / 2, 0.15, 0.15, OBH,
             "obs_cube", WHITE, phys=True)

    # ── Left 직선 장애물: y 간격 3.0 m, 지그재그 3개 (x축 좌우 교차)
    # 도로 중심 x=LX=-1.80, 장애물을 동서로 번갈아 배치
    for pname, ox, oy in [
        ("ObsL0", LX + LANE * 0.60, -3.0),   # 내측(동)
        ("ObsL1", LX - LANE * 0.60,  0.0),   # 외측(서)
        ("ObsL2", LX + LANE * 0.60, +3.0),   # 내측(동)
    ]:
        _box(stage, f"/World/Obstacles/{pname}",
             ox, oy, OBH / 2, 0.15, 0.15, OBH,
             "obs_cube", WHITE, phys=True)

    # ─── 7) QIV 터널 — 제거됨

    # ─── 완료 로그 ──────────────────────────────────────────────────────────
    print("[INFO] AutoRace 트랙 구성 완료 (v7 — 1차선, 6×12 m)")
    print(f"  바닥판 : 6.0 × 12.0 m")
    print(f"  루프   : {BL:.1f} × {VL:.1f} m, 코너 R={R_CORNER:.2f} m")
    print(f"  도로폭 : {TW:.2f} m  (TB3 0.14 + 장애물 0.20 + 여유 0.10 m)")
    print(f"  차선   : 좌측·우측 실선 2줄 (간격 {EDGE*2:.2f} m, 점선 없음)")
    print(f"  정지선 : N/S/E/W 각 1개, 교차로 경계+{SL_OFF:.2f}m, 두께 {SL_THK:.2f}m")
    print(f"  교차점 : (0, {CY:.2f}) — 4-way, CROSS_H={CROSS_H:.2f} m")
    print(f"  내부망 : 기존 HW/HE(y=0) + H2(y={H2_Y:.1f}) + H3(y={H3_Y:.1f}) 추가")
    print(f"  장애물 : Top x=±1.0 m 3개 | Left y=-3/0/+3 m 3개 (지그재그)")

    # ─── 로봇 시작 위치 배치 ─────────────────────────────────────────────────
    _robot_prim = stage.GetPrimAtPath(_ROBOT_PATH)
    if _robot_prim.IsValid():
        UsdGeom.XformCommonAPI(_robot_prim).SetTranslate(
            Gf.Vec3d(0.2261, -5.0052, 0.0)
        )
        print(f"[INFO] 로봇 배치: {_ROBOT_PATH}")
        print(f"         위치 (0.2261, -5.0052, 0.0)")
    else:
        print(f"[WARN] 로봇 프림 미발견: {_ROBOT_PATH}")


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
# SimulationContext 로 명시적 물리 스텝 진행 → OmniGraph ROS2 publisher trigger 보장
from isaacsim.core.api.simulation_context import SimulationContext
sim_context = SimulationContext(physics_dt=1.0/60.0, rendering_dt=1.0/60.0,
                                 stage_units_in_meters=1.0)
sim_context.initialize_physics()
sim_context.play()

# 초기 안정화 (몇 step 진행하여 OmniGraph 활성화)
for _ in range(30):
    sim_context.step(render=True)

# ── 5주차: IMX219 카메라 브리지 초기화 ────────────────────────────────────────
# 반드시 sim_context.play() + 초기 안정화 이후에 생성해야
# Replicator render_product 가 실행 중인 시뮬레이션에 올바르게 연결된다.
_cam_bridge = None
if _ENABLE_CAM:
    stage_ref = omni.usd.get_context().get_stage()
    _cam_bridge = _setup_cam_bridge(stage_ref)
    if _cam_bridge:
        # 렌더 제품 초기 워밍업 — 첫 번째 프레임을 즉시 렌더링
        simulation_app.update()

print("[INFO] 시뮬레이션 시작")
print("[INFO] 토픽: /scan /imu /odom /cmd_vel /tf /clock")
if _ENABLE_CAM and _cam_bridge:
    print("[INFO] 카메라 토픽: /camera/image_raw (Replicator → rclpy 직접 발행)")
print(f"[INFO] ROS_DOMAIN_ID = {os.environ.get('ROS_DOMAIN_ID', '0')}")
print("[INFO] 수동 제어: ros2 run teleop_twist_keyboard teleop_twist_keyboard")
print("[INFO] 종료: Ctrl+C")

_cam_tick = 0
_cam_interval = max(1, 60 // _CAM_HZ)   # 60 Hz 시뮬 기준 발행 분주비

try:
    while simulation_app.is_running():
        sim_context.step(render=True)

        if _ENABLE_CAM and _cam_bridge and _cam_tick % _cam_interval == 0:
            # simulation_app.update(): Replicator render_product 를 현재 프레임으로 갱신.
            # rep.orchestrator.step() 대신 사용 — 가볍고 시뮬 루프와 충돌 없음.
            simulation_app.update()
            _publish_cam_frame(*_cam_bridge)
        _cam_tick += 1

except KeyboardInterrupt:
    print("\n[INFO] 종료")
finally:
    if _ENABLE_CAM and _cam_bridge:
        try:
            import rclpy
            _cam_bridge[1].destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

sim_context.stop()
simulation_app.close()
