#!/usr/bin/env python3
"""
lane_detect.py — YOLOv8-detect ONNX 추론 기반 차선 추종 노드
=============================================================
/camera/image_raw 를 구독하여 YOLOv8-detect ONNX 모델로 좌/우 차선 마킹을 검출하고
두 박스의 중심 x 좌표 중간값을 목표로 /cmd_vel 을 발행한다.

모델 출력 형식 (nc=1):
  output0: (1, 5, 8400)  ← [cx, cy, w, h, conf] × 8400 앵커

차선 추종 알고리즘:
  1. 신뢰도 임계값 이상의 bbox를 NMS 로 정리
  2. x 좌표로 정렬 → 가장 왼쪽 = 왼쪽 차선, 가장 오른쪽 = 오른쪽 차선
  3. 목표 중심 x = (left_cx + right_cx) / 2
  4. offset = (target_cx - image_center) / (image_width / 2)  → [-1, 1]
  5. angular.z = -KP_ANGULAR * offset

토픽:
  구독: /camera/image_raw, /scan
  발행: /cmd_vel, /lane/debug_image
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
YOLO_SZ      = 640

# ─── 검출 파라미터 ─────────────────────────────────────────────────────────────
CONF_THRESH = 0.30   # 신뢰도 임계값 (detect 모델은 seg 보다 신뢰도가 높아 조금 낮게)
NMS_IOU_THR = 0.45

# ─── 차선 ROI ──────────────────────────────────────────────────────────────────
LANE_ROI_TOP_RATIO = 0.55   # 상단 이 비율 위는 무시

# ─── 제어 파라미터 ─────────────────────────────────────────────────────────────
MAX_SPEED   = 0.12
MIN_SPEED   = 0.04
MAX_ANGULAR = 1.80
KP_ANGULAR  = 2.20

# ─── LiDAR 장애물 파라미터 ─────────────────────────────────────────────────────
OBS_WARN  = 0.70
OBS_AVOID = 0.50
OBS_STOP  = 0.25
FRONT_DEG = 35
SIDE_DEG  = 65


# ═══════════════════════════════════════════════════════════════════════════════
# ── YOLOv8-detect ONNX 추론 ───────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

class YoloDetector:
    """YOLOv8-detect ONNX 모델 로드 및 추론."""

    def __init__(self, onnx_path: str):
        import onnxruntime as ort
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        self.sess = ort.InferenceSession(onnx_path, providers=providers)
        inp = self.sess.get_inputs()[0]
        self.input_name = inp.name
        self.out_names  = [o.name for o in self.sess.get_outputs()]
        print(f"[YOLO] 모델 로드: {onnx_path}")
        print(f"[YOLO] 입력: {inp.name} {inp.shape}")
        outs_info = [(o.name, o.shape) for o in self.sess.get_outputs()]
        print(f"[YOLO] 출력: {outs_info}")

    def _preprocess(self, bgr: np.ndarray) -> np.ndarray:
        import cv2
        img = cv2.resize(bgr, (YOLO_SZ, YOLO_SZ))
        img = img[:, :, ::-1].astype(np.float32) / 255.0
        return np.ascontiguousarray(img.transpose(2, 0, 1)[None])

    @staticmethod
    def _nms(cx: np.ndarray, cy: np.ndarray, w: np.ndarray, h: np.ndarray,
             scores: np.ndarray, iou_thr: float = NMS_IOU_THR) -> np.ndarray:
        x1 = cx - w / 2;  y1 = cy - h / 2
        x2 = cx + w / 2;  y2 = cy + h / 2
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep  = []
        while order.size:
            i = order[0];  keep.append(i)
            if order.size == 1:
                break
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
            iou   = inter / (areas[i] + areas[order[1:]] - inter + 1e-7)
            order = order[1:][iou < iou_thr]
        return np.array(keep, dtype=np.int64)

    def infer(self, bgr: np.ndarray):
        """
        Returns
        -------
        boxes : list[dict]  각 원소: {'cx', 'cy', 'w', 'h', 'conf'}
            좌표는 원본 이미지(BGR) 픽셀 단위
        """
        inp  = self._preprocess(bgr)
        outs = self.sess.run(self.out_names, {self.input_name: inp})

        # output0: (1, 5, 8400)  [cx cy w h conf]
        pred = outs[0][0]   # (5, 8400)

        raw_cx   = pred[0]   # (8400,) 0~640 범위 (YOLO 입력 좌표계)
        raw_cy   = pred[1]
        raw_w    = pred[2]
        raw_h    = pred[3]
        scores   = pred[4]   # nc=1 이므로 마지막 행이 클래스 0 신뢰도

        # ROI 필터: cy > YOLO_SZ * LANE_ROI_TOP_RATIO 인 박스만 유지
        roi_top_px = YOLO_SZ * LANE_ROI_TOP_RATIO
        pre_idx = np.where((scores > CONF_THRESH) & (raw_cy > roi_top_px))[0]

        if pre_idx.size == 0:
            return []

        keep = self._nms(raw_cx[pre_idx], raw_cy[pre_idx],
                         raw_w[pre_idx], raw_h[pre_idx],
                         scores[pre_idx])
        valid_idx = pre_idx[keep]

        h_orig, w_orig = bgr.shape[:2]
        sx = w_orig / YOLO_SZ
        sy = h_orig / YOLO_SZ

        results = []
        for i in valid_idx:
            results.append({
                "cx":   float(raw_cx[i] * sx),
                "cy":   float(raw_cy[i] * sy),
                "w":    float(raw_w[i]  * sx),
                "h":    float(raw_h[i]  * sy),
                "conf": float(scores[i]),
            })
        return results


# ═══════════════════════════════════════════════════════════════════════════════
# ── 차선 미드포인트 계산 ───────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

def compute_lane_offset(boxes: list[dict], img_w: int) -> tuple[float | None, dict]:
    """
    검출된 bbox 에서 목표 중심 오프셋을 계산한다.

    Returns
    -------
    offset : float [-1, 1] 또는 None (차선 미검출)
    info   : {'left_cx', 'right_cx', 'mid_cx'}  (시각화용)
    """
    if not boxes:
        return None, {}

    sorted_boxes = sorted(boxes, key=lambda b: b["cx"])

    if len(sorted_boxes) == 1:
        # 한쪽 차선만 검출 — 검출된 쪽의 반대로 보정
        b  = sorted_boxes[0]
        cx = b["cx"]
        if cx < img_w / 2:
            # 왼쪽 차선만 보임 → 로봇이 오른쪽으로 치우침 → 왼쪽으로 조향
            mid_cx = cx + img_w * 0.25
        else:
            # 오른쪽 차선만 보임 → 로봇이 왼쪽으로 치우침 → 오른쪽으로 조향
            mid_cx = cx - img_w * 0.25
        offset = (mid_cx - img_w / 2.0) / (img_w / 2.0)
        return float(np.clip(offset, -1.0, 1.0)), {
            "left_cx":  cx if cx < img_w / 2 else None,
            "right_cx": cx if cx >= img_w / 2 else None,
            "mid_cx":   mid_cx,
        }

    # 좌/우 차선 모두 검출
    left_cx  = sorted_boxes[0]["cx"]
    right_cx = sorted_boxes[-1]["cx"]
    mid_cx   = (left_cx + right_cx) / 2.0
    offset   = (mid_cx - img_w / 2.0) / (img_w / 2.0)
    return float(np.clip(offset, -1.0, 1.0)), {
        "left_cx":  left_cx,
        "right_cx": right_cx,
        "mid_cx":   mid_cx,
    }


# ═══════════════════════════════════════════════════════════════════════════════
# ── 폴백: 흰색 픽셀 기반 좌/우 차선 중심 ─────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

def fallback_lane_offset(bgr: np.ndarray) -> tuple[float | None, dict]:
    """ONNX 모델 없을 때 흰색 픽셀로 좌/우 차선 x 중심을 추정한다."""
    import cv2
    gray   = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    roi_y  = int(IMG_H * LANE_ROI_TOP_RATIO)
    roi    = gray[roi_y:]

    WHITE_THRESH = 200
    mask   = roi > WHITE_THRESH
    left   = mask.copy(); left[:,  IMG_W // 2:] = False
    right  = mask.copy(); right[:, :IMG_W // 2] = False

    def _col_center(m):
        _, xs = np.where(m)
        return float(xs.mean()) if xs.size > 30 else None

    lx = _col_center(left)
    rx = _col_center(right)

    boxes = []
    if lx is not None:
        boxes.append({"cx": lx})
    if rx is not None:
        boxes.append({"cx": rx + IMG_W // 2})

    return compute_lane_offset(boxes, IMG_W)


# ═══════════════════════════════════════════════════════════════════════════════
# ── LiDAR 장애물 판단 ─────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

def scan_obstacle_state(ranges, angle_min, angle_increment):
    n = len(ranges)

    def idx(deg):
        return int(round((math.radians(deg) - angle_min) / angle_increment)) % n

    front = [r for i in range(idx(-FRONT_DEG), idx(FRONT_DEG) + 1)
             for r in [ranges[i % n]] if 0.03 < r < 30.0]
    if not front:
        return 'clear', None

    min_front = min(front)
    if min_front > OBS_WARN:
        return 'clear', None

    left  = [r for i in range(idx(0), idx(SIDE_DEG) + 1)
             for r in [ranges[i % n]] if 0.03 < r < 30.0]
    right = [r for i in range(idx(-SIDE_DEG), idx(0) + 1)
             for r in [ranges[i % n]] if 0.03 < r < 30.0]
    avg_l = float(np.mean(left))  if left  else 0.0
    avg_r = float(np.mean(right)) if right else 0.0
    avoid_dir = 1 if avg_l >= avg_r else -1

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
            self.detector = YoloDetector(best_onnx)
        else:
            self.get_logger().warn(
                "ONNX 모델 없음 — 흰색 픽셀 폴백 모드\n"
                "  먼저: python3 models/train_yolo_lane.py"
            )
            self.detector = None

        self.bridge = CvBridge()

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(RosImage,  "/camera/image_raw", self._cb_image, best_effort_qos)
        self.create_subscription(LaserScan, "/scan",             self._cb_scan,  best_effort_qos)

        self.cmd_pub = self.create_publisher(Twist,    "/cmd_vel",          10)
        self.dbg_pub = self.create_publisher(RosImage, "/lane/debug_image",  1)

        self._scan_ranges    = None
        self._scan_angle_min = 0.0
        self._scan_angle_inc = 0.0
        self._avoid_dir_lock = None
        self._start_time     = time.time()

        self.get_logger().info("LaneDetectNode 시작 — /camera/image_raw 대기 중")

    def _find_onnx(self) -> str | None:
        runs_dir = ROOT / "models" / "runs"
        # lane_det* 우선, 없으면 lane_seg* 에서도 탐색
        candidates = sorted(
            list(runs_dir.glob("lane_det*/weights/best.onnx")) +
            list(runs_dir.glob("lane_seg*/weights/best.onnx")),
            key=lambda p: p.stat().st_mtime,
        )
        return str(candidates[-1]) if candidates else None

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

        # ── 1) 차선 bbox 검출 ──────────────────────────────────────────────────
        if self.detector is not None:
            boxes = self.detector.infer(bgr)
            offset, info = compute_lane_offset(boxes, IMG_W)
        else:
            boxes = []
            offset, info = fallback_lane_offset(bgr)

        # ── 2) LiDAR 장애물 판단 ──────────────────────────────────────────────
        obs_state, avoid_dir = 'clear', None
        if self._scan_ranges is not None:
            obs_state, avoid_dir = scan_obstacle_state(
                self._scan_ranges, self._scan_angle_min, self._scan_angle_inc)

        if obs_state == 'stop':
            self._avoid_dir_lock = avoid_dir
            ang = float(self._avoid_dir_lock or 1) * MAX_ANGULAR
            self._publish_cmd(0.0, ang)
            self._publish_debug(bgr, boxes, info, ang, "STOP_OBS")
            return

        # ── 3) 차선 중심 오프셋 → 조향 ──────────────────────────────────────
        if obs_state == 'avoid':
            if self._avoid_dir_lock is None:
                self._avoid_dir_lock = avoid_dir
            ang   = float(self._avoid_dir_lock or 1) * MAX_ANGULAR * 0.8
            speed = MIN_SPEED
            label = "AVOID"
        elif obs_state == 'warn':
            self._avoid_dir_lock = None
            ang   = float(-np.clip(KP_ANGULAR * (offset or 0.0), -MAX_ANGULAR, MAX_ANGULAR))
            speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * 0.5
            label = "WARN"
        else:
            self._avoid_dir_lock = None
            ang   = float(-np.clip(KP_ANGULAR * (offset or 0.0), -MAX_ANGULAR, MAX_ANGULAR))
            speed = MAX_SPEED if offset is not None else MIN_SPEED
            label = "LANE" if offset is not None else "NO_LANE"

        self._publish_cmd(speed, ang)
        self._publish_debug(bgr, boxes, info, offset or 0.0, label)

    def _publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _publish_debug(self, bgr: np.ndarray, boxes: list[dict],
                       info: dict, offset: float, label: str):
        if self.dbg_pub.get_subscription_count() == 0:
            return

        import cv2
        vis = bgr.copy()

        # 각 bbox 를 녹색 사각형으로 그림
        for b in boxes:
            cx, cy = int(b["cx"]), int(b["cy"])
            hw, hh = int(b.get("w", 40) / 2), int(b.get("h", 40) / 2)
            cv2.rectangle(vis, (cx - hw, cy - hh), (cx + hw, cy + hh),
                          (0, 255, 0), 2)
            cv2.circle(vis, (cx, cy), 4, (0, 255, 0), -1)

        # 좌/우 차선 중심 및 목표 중심선
        if "left_cx" in info and info["left_cx"] is not None:
            cv2.line(vis, (int(info["left_cx"]), 0), (int(info["left_cx"]), IMG_H),
                     (255, 0, 0), 1)   # 파랑 = 왼쪽
        if "right_cx" in info and info["right_cx"] is not None:
            cv2.line(vis, (int(info["right_cx"]), 0), (int(info["right_cx"]), IMG_H),
                     (0, 0, 255), 1)   # 빨강 = 오른쪽
        if "mid_cx" in info:
            mid_x = int(info["mid_cx"])
            cv2.line(vis, (mid_x, int(IMG_H * LANE_ROI_TOP_RATIO)), (mid_x, IMG_H - 1),
                     (0, 255, 255), 2)   # 노랑 = 목표 중심

        # 이미지 중심선
        cv2.line(vis, (IMG_W // 2, IMG_H - 40), (IMG_W // 2, IMG_H - 1),
                 (255, 255, 255), 1)

        cv2.putText(vis, f"{label}  off={offset:+.2f}  det={len(boxes)}",
                    (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        try:
            self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding="bgr8"))
        except Exception:
            pass


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
