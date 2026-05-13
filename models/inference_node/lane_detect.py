#!/usr/bin/env python3
"""
lane_detect.py — YOLOv8-seg ONNX 추론 기반 차선 추종 노드
===========================================================
/camera/image_raw 를 구독하여 YOLOv8-seg ONNX 모델로 차선·정지선을 검출하고
/cmd_vel 을 발행한다.  /scan 을 함께 구독하여 LiDAR 장애물 회피를 병용한다.

실행 전 준비:
  1. Isaac Sim 에서 ENABLE_CAMERA=1 로 트랙 씬 시작
     ENABLE_CAMERA=1 bash isaac_sim/launch_sim.sh
  2. 합성 데이터 생성 후 학습:
     /home/linux/isaac_env/bin/python isaac_sim/generate_synthetic_data.py
     python3 models/train_yolo_lane.py
  3. 이 노드 실행:
     source /home/linux/gyusama-project/isaac_sim/ros2_terminal.sh
     python3 models/inference_node/lane_detect.py

모델이 없으면 raw 이미지 흰색 픽셀 중심값을 사용하는 폴백 모드로 동작한다.

토픽:
  구독: /camera/image_raw, /scan
  발행: /cmd_vel, /lane/debug_image (시각화용)
"""

import math
import time
import os
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image as RosImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

ROOT = Path(__file__).resolve().parent.parent.parent  # gyusama-project/

# ─── 카메라 파라미터 ───────────────────────────────────────────────────────────
IMG_W, IMG_H = 640, 480
YOLO_SZ      = 640   # ONNX 입력 크기

# ─── YOLOv8-seg 클래스 인덱스 ──────────────────────────────────────────────────
CLS_WHITE_LANE = 0
CLS_STOP_LINE  = 1
CONF_THRESH    = 0.35   # 검출 신뢰도 임계값
MASK_THRESH    = 0.50   # 세그멘테이션 마스크 이진화 임계값

# ─── 제어 파라미터 ─────────────────────────────────────────────────────────────
MAX_SPEED     = 0.12   # 정상 직진 속도 (m/s)
MIN_SPEED     = 0.04   # 장애물 감속 최솟값
MAX_ANGULAR   = 1.80   # 최대 각속도 (rad/s)
KP_ANGULAR    = 2.20   # 횡방향 오프셋 → 각속도 비례 이득

# ─── LiDAR 장애물 파라미터 ─────────────────────────────────────────────────────
OBS_WARN  = 0.70
OBS_AVOID = 0.50
OBS_STOP  = 0.25
FRONT_DEG = 35
SIDE_DEG  = 65

# ─── 정지선 파라미터 ──────────────────────────────────────────────────────────
STOP_DURATION   = 2.0    # 정지선 감지 시 정지 시간 (초)
STOP_COOLDOWN   = 10.0   # 동일 정지선 재감지 방지 쿨다운 (초)
STARTUP_DELAY   = 5.0    # 시작 후 정지선 감지 비활성 시간 (초)
STOP_MIN_PIXELS = 300    # 정지선으로 판정할 최소 마스크 픽셀 수

# ─── 차선 검출 파라미터 (폴백 및 YOLO 공용) ────────────────────────────────────
LANE_ROI_TOP_RATIO = 0.55   # 이미지 상단 이 비율 이상 위는 무시 (원거리)
WHITE_THRESH       = 200    # 폴백 흰색 임계값 (그레이스케일)


# ═══════════════════════════════════════════════════════════════════════════════
# ── ONNX 추론 핸들러 ──────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

class YoloSegDetector:
    """YOLOv8-seg ONNX 모델 로드 및 추론."""

    def __init__(self, onnx_path: str):
        import onnxruntime as ort
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        self.sess = ort.InferenceSession(onnx_path, providers=providers)
        inp = self.sess.get_inputs()[0]
        self.input_name = inp.name
        # 출력: output0=[1,38,8400], output1=[1,32,160,160]
        self.out_names = [o.name for o in self.sess.get_outputs()]
        print(f"[YOLO] 모델 로드: {onnx_path}")
        print(f"[YOLO] 입력: {inp.name} {inp.shape}")

    def _preprocess(self, bgr: np.ndarray):
        """BGR → [1,3,640,640] float32 정규화."""
        import cv2
        img = cv2.resize(bgr, (YOLO_SZ, YOLO_SZ))
        img = img[:, :, ::-1].astype(np.float32) / 255.0  # RGB + normalize
        return np.ascontiguousarray(img.transpose(2, 0, 1)[None])  # NCHW

    def _sigmoid(self, x: np.ndarray) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-np.clip(x, -50, 50)))

    def infer(self, bgr: np.ndarray):
        """
        Parameters
        ----------
        bgr : np.ndarray  (H, W, 3) BGR

        Returns
        -------
        masks : dict[int, np.ndarray]
            클래스 ID → 원본 해상도 이진 마스크 (H, W) bool
        """
        inp = self._preprocess(bgr)
        outs = self.sess.run(self.out_names, {self.input_name: inp})

        # output0: [1, 4+nc+32, 8400]  output1: [1, 32, 160, 160]
        pred   = outs[0][0]            # (38, 8400)
        protos = outs[1][0]            # (32, 160, 160)

        nc = 2   # white_lane, stop_line
        boxes      = pred[:4]          # (4, 8400) cx cy w h
        cls_confs  = pred[4:4+nc]      # (nc, 8400)
        mask_coeff = pred[4+nc:]       # (32, 8400)

        # 클래스별 최대 신뢰도와 인덱스
        cls_ids    = np.argmax(cls_confs, axis=0)           # (8400,)
        max_confs  = cls_confs[cls_ids, np.arange(8400)]    # (8400,)

        valid_mask  = max_confs > CONF_THRESH
        valid_idx   = np.where(valid_mask)[0]

        h_orig, w_orig = bgr.shape[:2]
        combined = {CLS_WHITE_LANE: np.zeros((h_orig, w_orig), dtype=bool),
                    CLS_STOP_LINE:  np.zeros((h_orig, w_orig), dtype=bool)}

        if valid_idx.size == 0:
            return combined

        # 마스크 디코드: (32, N) @ (32, 160*160) → (N, 160, 160)
        mc      = mask_coeff[:, valid_idx].T            # (N, 32)
        proto_f = protos.reshape(32, -1)                # (32, 160*160)
        raw     = self._sigmoid(mc @ proto_f)           # (N, 160*160)
        raw     = raw.reshape(-1, 160, 160)             # (N, 160, 160)

        # bbox crop (160x160 기준)
        cxs = boxes[0, valid_idx] / YOLO_SZ * 160
        cys = boxes[1, valid_idx] / YOLO_SZ * 160
        ws  = boxes[2, valid_idx] / YOLO_SZ * 160
        hs  = boxes[3, valid_idx] / YOLO_SZ * 160

        import cv2
        for i in range(len(valid_idx)):
            cls_id = cls_ids[valid_idx[i]]
            x1 = int(max(0,   cxs[i] - ws[i] / 2))
            y1 = int(max(0,   cys[i] - hs[i] / 2))
            x2 = int(min(160, cxs[i] + ws[i] / 2))
            y2 = int(min(160, cys[i] + hs[i] / 2))

            m = (raw[i] > MASK_THRESH).astype(np.uint8)
            # bbox 외부 마스크 제거
            crop = np.zeros((160, 160), dtype=np.uint8)
            crop[y1:y2, x1:x2] = m[y1:y2, x1:x2]

            # 원본 해상도로 리사이즈
            m_full = cv2.resize(crop, (w_orig, h_orig),
                                interpolation=cv2.INTER_NEAREST).astype(bool)
            combined[cls_id] |= m_full

        return combined


# ═══════════════════════════════════════════════════════════════════════════════
# ── 폴백: 흰색 픽셀 기반 차선 중심 ─────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

def fallback_lane_mask(bgr: np.ndarray):
    """ONNX 모델 없을 때 간단한 흰색 픽셀 임계값으로 차선 마스크 생성."""
    import cv2
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    roi_y = int(IMG_H * LANE_ROI_TOP_RATIO)
    mask  = np.zeros_like(gray, dtype=bool)
    mask[roi_y:] = gray[roi_y:] > WHITE_THRESH
    return mask


# ═══════════════════════════════════════════════════════════════════════════════
# ── 차선 중심 추출 ────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

def compute_lane_offset(lane_mask: np.ndarray) -> float | None:
    """
    차선 마스크에서 횡방향 오프셋을 계산한다.
    이미지 하단 ROI 내 흰색 픽셀 무게중심의 x 좌표를 사용한다.

    Returns
    -------
    offset : float in [-1, 1]  (음수=왼쪽, 양수=오른쪽) 또는 None (차선 미검출)
    """
    roi_y  = int(IMG_H * LANE_ROI_TOP_RATIO)
    bottom = lane_mask[roi_y:]

    ys, xs = np.where(bottom)
    if xs.size < 50:
        return None

    cx     = float(np.mean(xs))
    offset = (cx - IMG_W / 2.0) / (IMG_W / 2.0)   # normalized [-1, 1]
    return float(np.clip(offset, -1.0, 1.0))


# ═══════════════════════════════════════════════════════════════════════════════
# ── LiDAR 장애물 판단 ─────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

def scan_obstacle_state(ranges, angle_min, angle_increment):
    """
    Returns
    -------
    (state, avoid_dir)
        state    : 'clear' | 'warn' | 'avoid' | 'stop'
        avoid_dir: +1(좌) | -1(우)  (state=='clear' 이면 None)
    """
    n = len(ranges)

    def idx(deg):
        rad = math.radians(deg)
        return int(round((rad - angle_min) / angle_increment)) % n

    front = [r for i in range(idx(-FRONT_DEG), idx(FRONT_DEG) + 1)
             for r in [ranges[i % n]] if 0.03 < r < 30.0]
    if not front:
        return 'clear', None

    min_front = min(front)

    if min_front > OBS_WARN:
        return 'clear', None

    # 회피 방향 결정: 공간이 더 넓은 쪽
    left  = [r for i in range(idx(0), idx(SIDE_DEG) + 1)
             for r in [ranges[i % n]] if 0.03 < r < 30.0]
    right = [r for i in range(idx(-SIDE_DEG), idx(0) + 1)
             for r in [ranges[i % n]] if 0.03 < r < 30.0]
    avg_l = float(np.mean(left)) if left else 0.0
    avg_r = float(np.mean(right)) if right else 0.0
    avoid_dir = 1 if avg_l >= avg_r else -1   # 1=좌, -1=우

    if min_front <= OBS_STOP:
        return 'stop',  avoid_dir
    if min_front <= OBS_AVOID:
        return 'avoid', avoid_dir
    return 'warn', avoid_dir


# ═══════════════════════════════════════════════════════════════════════════════
# ── ROS2 노드 ─────────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

class LaneDetectNode(Node):

    def __init__(self):
        super().__init__("lane_detect_node")

        best_onnx = self._find_onnx()
        if best_onnx:
            self.get_logger().info(f"ONNX 모델 사용: {best_onnx}")
            self.detector = YoloSegDetector(best_onnx)
        else:
            self.get_logger().warn(
                "ONNX 모델 없음 — 흰색 픽셀 폴백 모드로 동작\n"
                "  먼저: python3 models/train_yolo_lane.py"
            )
            self.detector = None

        self.bridge = CvBridge()

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(RosImage,   "/camera/image_raw", self._cb_image,
                                 best_effort_qos)
        self.create_subscription(LaserScan,  "/scan",             self._cb_scan,
                                 best_effort_qos)

        self.cmd_pub   = self.create_publisher(Twist,    "/cmd_vel",            10)
        self.dbg_pub   = self.create_publisher(RosImage, "/lane/debug_image",   1)

        # 내부 상태
        self._scan_ranges     = None
        self._scan_angle_min  = 0.0
        self._scan_angle_inc  = 0.0
        self._last_stop_time  = 0.0
        self._stop_until      = 0.0    # 이 시간까지 정지
        self._avoid_dir_lock  = None   # 회피 방향 잠금
        self._avoid_lock_dist = OBS_WARN + 0.15
        self._start_time      = time.time()

        self.get_logger().info("LaneDetectNode 시작 — /camera/image_raw 대기 중")

    # ── 모델 경로 탐색 ─────────────────────────────────────────────────────────

    def _find_onnx(self) -> str | None:
        runs_dir = ROOT / "models" / "runs"
        candidates = sorted(runs_dir.glob("lane_seg*/weights/best.onnx"),
                            key=lambda p: p.stat().st_mtime)
        return str(candidates[-1]) if candidates else None

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _cb_scan(self, msg: LaserScan):
        self._scan_ranges    = list(msg.ranges)
        self._scan_angle_min = msg.angle_min
        self._scan_angle_inc = msg.angle_increment

    def _cb_image(self, msg: RosImage):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")
            return

        now = time.time()

        # ── 1) 차선/정지선 마스크 획득 ─────────────────────────────────────────
        if self.detector is not None:
            masks = self.detector.infer(bgr)
            lane_mask      = masks[CLS_WHITE_LANE]
            stop_line_mask = masks[CLS_STOP_LINE]
        else:
            lane_mask      = fallback_lane_mask(bgr)
            stop_line_mask = np.zeros((IMG_H, IMG_W), dtype=bool)

        # ── 2) 정지선 감지 ─────────────────────────────────────────────────────
        stop_pixels = int(np.sum(stop_line_mask))
        elapsed_since_start = now - self._start_time
        cooldown_ok = (now - self._last_stop_time) > STOP_COOLDOWN
        startup_ok  = elapsed_since_start > STARTUP_DELAY

        if (stop_pixels > STOP_MIN_PIXELS and startup_ok and cooldown_ok
                and now >= self._stop_until):
            self.get_logger().info(
                f"정지선 감지 ({stop_pixels}px) — {STOP_DURATION}초 정지"
            )
            self._stop_until      = now + STOP_DURATION
            self._last_stop_time  = now

        # ── 3) 정지 중이면 속도 0 발행 ────────────────────────────────────────
        if now < self._stop_until:
            self._publish_cmd(0.0, 0.0)
            self._publish_debug(bgr, lane_mask, stop_line_mask, 0.0, "STOP_LINE")
            return

        # ── 4) LiDAR 장애물 판단 ──────────────────────────────────────────────
        obs_state, avoid_dir = 'clear', None
        if self._scan_ranges is not None:
            obs_state, avoid_dir = scan_obstacle_state(
                self._scan_ranges, self._scan_angle_min, self._scan_angle_inc)

        if obs_state == 'stop':
            self._avoid_dir_lock = avoid_dir
            ang = float(self._avoid_dir_lock or 1) * MAX_ANGULAR
            self._publish_cmd(0.0, ang)
            self._publish_debug(bgr, lane_mask, stop_line_mask, ang, "STOP_OBS")
            return

        # ── 5) 차선 중심 오프셋 계산 ─────────────────────────────────────────
        offset = compute_lane_offset(lane_mask)

        if obs_state == 'avoid':
            if self._avoid_dir_lock is None:
                self._avoid_dir_lock = avoid_dir
            ang   = float(self._avoid_dir_lock or 1) * MAX_ANGULAR * 0.8
            speed = MIN_SPEED
            label = "AVOID"
        elif obs_state == 'warn':
            self._avoid_dir_lock = None
            if offset is not None:
                ang = float(-np.clip(KP_ANGULAR * offset, -MAX_ANGULAR, MAX_ANGULAR))
            else:
                ang = 0.0
            speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * 0.5
            label = "WARN"
        else:
            self._avoid_dir_lock = None
            if offset is not None:
                ang = float(-np.clip(KP_ANGULAR * offset, -MAX_ANGULAR, MAX_ANGULAR))
            else:
                ang = 0.0
            speed = MAX_SPEED
            label = "LANE"

        self._publish_cmd(speed, ang)
        self._publish_debug(bgr, lane_mask, stop_line_mask,
                            offset if offset else 0.0, label)

    # ── 발행 헬퍼 ─────────────────────────────────────────────────────────────

    def _publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _publish_debug(self, bgr: np.ndarray, lane_mask: np.ndarray,
                       stop_mask: np.ndarray, offset: float, label: str):
        """차선 마스크 오버레이 + 상태 텍스트를 /lane/debug_image 로 발행."""
        if self.dbg_pub.get_subscription_count() == 0:
            return  # 구독자 없으면 렌더링 생략

        import cv2
        vis = bgr.copy()

        # 흰선 오버레이 (초록)
        vis[lane_mask]  = (vis[lane_mask]  * 0.4 + np.array([0, 255, 0])   * 0.6).astype(np.uint8)
        # 정지선 오버레이 (빨강)
        vis[stop_mask]  = (vis[stop_mask]  * 0.4 + np.array([0, 0,   255]) * 0.6).astype(np.uint8)

        # 오프셋 중심선
        cx = int(IMG_W / 2 + offset * IMG_W / 2)
        cv2.line(vis, (cx, int(IMG_H * LANE_ROI_TOP_RATIO)), (cx, IMG_H - 1),
                 (0, 255, 255), 2)
        cv2.line(vis, (IMG_W // 2, IMG_H - 40), (IMG_W // 2, IMG_H - 1),
                 (255, 255, 255), 1)

        cv2.putText(vis, f"{label}  off={offset:+.2f}", (8, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            self.dbg_pub.publish(dbg_msg)
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════════════════
# ── 진입점 ────────────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = LaneDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
