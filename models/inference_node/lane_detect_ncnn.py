#!/usr/bin/env python3
"""
lane_detect_ncnn.py — YOLOv8-detect ncnn 추론 기반 차선 추종 노드
=============================================================
/camera/image_raw 를 구독하여 YOLOv8-detect ONNX 모델로 좌/우 차선 마킹을 검출하고
두 박스의 중심 x 좌표 중간값을 목표로 /lane/cmd_vel 을 발행한다.

모델 출력 형식 (nc=1):
  output0: (1, 5, 8400)  ← [cx, cy, w, h, conf] × 8400 앵커

차선 추종 알고리즘:
  1. 신뢰도 임계값 이상의 bbox를 NMS 로 정리
  2. x 좌표로 정렬 → 가장 왼쪽 = 왼쪽 차선, 가장 오른쪽 = 오른쪽 차선
  3. 목표 중심 x = (left_cx + right_cx) / 2
  4. offset = (target_cx - image_center) / (image_width / 2)  → [-1, 1]
  5. angular.z = -KP_ANGULAR * offset

토픽:
  구독: /camera/image_raw
  발행: /lane/cmd_vel, /lane/debug_image
"""

import math
import time
import os
import sys
import threading
import termios
import tty
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image as RosImage
from geometry_msgs.msg import Twist, TwistStamped

# RPi5(turtlebot3_node Jazzy): TwistStamped / Isaac Sim: Twist
_USE_STAMPED = os.environ.get('CMD_VEL_STAMPED', '0') == '1'
from cv_bridge import CvBridge

ROOT = Path(__file__).resolve().parent.parent.parent  # gyusama-project/

# ─── 카메라 파라미터 ───────────────────────────────────────────────────────────
IMG_W, IMG_H = 640, 480
YOLO_SZ      = 640

# ─── 검출 파라미터 ─────────────────────────────────────────────────────────────
CONF_THRESH = 0.12   # 가장자리 차선은 신뢰도가 낮게 나옴 → 낮게 설정
NMS_IOU_THR = 0.45

# ─── 차선 ROI ──────────────────────────────────────────────────────────────────
LANE_ROI_TOP_RATIO = 0.50   # 상단 이 비율 위는 무시

# ─── 제어 파라미터 ─────────────────────────────────────────────────────────────
MAX_SPEED   = 0.14   # 직선 속도
MIN_SPEED   = 0.08   # 커브 속도
MAX_ANGULAR = 12.0   # 11.0→12.0: 조향 상한 상향
KP_ANGULAR  = 11.0   # 10.0→11.0: 조향 게인 상향
STEER_EXP   = 1.4    # 큰 오프셋(커브)에서 조향 가파르게

# ─── 차선별 독립 추적 ──────────────────────────────────────────────────────────
LANE_TRACK_THRESH = 80     # 마지막 위치에서 이 픽셀 이내 검출만 해당 차선으로 수락


# ═══════════════════════════════════════════════════════════════════════════════
# ── YOLOv8-detect ncnn 추론 ───────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

class YoloDetector:
    """YOLOv8-detect ncnn 모델 로드 및 추론."""

    def __init__(self, model_dir: str):
        import ncnn
        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute  = False
        self.net.opt.num_threads         = 4      # RPi5 4코어 활용
        self.net.opt.use_fp16_packed     = True   # fp16 연산 활성화
        self.net.opt.use_fp16_storage    = True
        self.net.opt.use_fp16_arithmetic = True
        self.net.load_param(f"{model_dir}/model.ncnn.param")
        self.net.load_model(f"{model_dir}/model.ncnn.bin")
        self.last_infer_ms = 0.0
        print(f"[YOLO] ncnn 모델 로드: {model_dir}")
        print(f"[YOLO] 입력: in0 [1, 3, {YOLO_SZ}, {YOLO_SZ}]")
        print(f"[YOLO] 출력: out0 [5, 8400]")

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
        import cv2, ncnn
        img = cv2.resize(bgr, (YOLO_SZ, YOLO_SZ))
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        mat_in = ncnn.Mat.from_pixels(img_rgb, 1, YOLO_SZ, YOLO_SZ)  # 1 = PIXEL_RGB
        mat_in.substract_mean_normalize([], [1/255.0, 1/255.0, 1/255.0])

        ex = self.net.create_extractor()
        ex.input("in0", mat_in)
        _t0 = time.perf_counter()
        _, mat_out = ex.extract("out0")
        self.last_infer_ms = (time.perf_counter() - _t0) * 1000.0

        pred = np.array(mat_out)          # (5, 8400) 또는 (5, 1, 8400)
        if pred.ndim == 3:
            pred = pred[:, 0, :]          # → (5, 8400)

        raw_cx = pred[0];  raw_cy = pred[1]
        raw_w  = pred[2];  raw_h  = pred[3]
        scores = pred[4]

        roi_top_px = YOLO_SZ * LANE_ROI_TOP_RATIO
        pre_idx = np.where((scores > CONF_THRESH) & (raw_cy > roi_top_px))[0]
        if pre_idx.size == 0:
            return []

        keep = self._nms(raw_cx[pre_idx], raw_cy[pre_idx],
                         raw_w[pre_idx],  raw_h[pre_idx],
                         scores[pre_idx])
        valid_idx = pre_idx[keep]

        h_orig, w_orig = bgr.shape[:2]
        sx = w_orig / YOLO_SZ;  sy = h_orig / YOLO_SZ

        results = []
        for i in valid_idx:
            results.append({
                "cx":   float(raw_cx[i] * sx),
                "cy":   float(raw_cy[i] * sy),
                "w":    float(raw_w[i]  * sx),
                "h":    float(raw_h[i]  * sy),
                "conf": float(scores[i]),
            })

        zone_best: dict[int, dict] = {}
        for box in results:
            zone = 0 if box["cx"] < w_orig / 2 else 1
            if zone not in zone_best or box["conf"] > zone_best[zone]["conf"]:
                zone_best[zone] = box
        return list(zone_best.values())


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

    WHITE_THRESH = 180   # 200→180: 조명 변화에 더 강하게
    mask   = roi > WHITE_THRESH
    left   = mask.copy(); left[:,  IMG_W // 2:] = False
    right  = mask.copy(); right[:, :IMG_W // 2] = False

    def _col_center(m):
        _, xs = np.where(m)
        return float(xs.mean()) if xs.size > 20 else None

    lx = _col_center(left)
    rx = _col_center(right)

    boxes = []
    if lx is not None:
        boxes.append({"cx": lx})
    if rx is not None:
        # rx 는 np.where(right) 의 결과 → 이미 절대 x 좌표 (320~639)
        # 기존 코드의 "+ IMG_W // 2" 는 이중 오프셋 버그였음
        boxes.append({"cx": rx})

    return compute_lane_offset(boxes, IMG_W)



# ═══════════════════════════════════════════════════════════════════════════════
# ── ROS2 노드 ─────────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

class LaneDetectNode(Node):

    def __init__(self):
        super().__init__("lane_detect_node")

        model_path = os.environ.get('ONNX_MODEL') or self._find_ncnn()
        if model_path:
            self.get_logger().info(f"ncnn 모델 사용: {model_path}")
            self.detector = YoloDetector(model_path)
        else:
            self.get_logger().warn(
                "ncnn 모델 없음 — 흰색 픽셀 폴백 모드\n"
                "  먼저: models/runs/lane_det*/weights/best_ncnn_model 디렉터리 확인"
            )
            self.detector = None

        self.bridge = CvBridge()

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(RosImage, "/camera/image_raw", self._cb_image, best_effort_qos)

        msg_type     = TwistStamped if _USE_STAMPED else Twist
        self.cmd_pub = self.create_publisher(msg_type, "/lane/cmd_vel",      10)
        self.dbg_pub = self.create_publisher(RosImage, "/lane/debug_image",  1)

        self._start_time     = time.time()

        self._last_offset  = 0.0   # 마지막 유효 오프셋
        self._last_lx      = None  # 마지막 유효 좌측 차선 x (픽셀)
        self._last_rx      = None  # 마지막 유효 우측 차선 x (픽셀)
        self._lost_frames  = 0     # 차선 미검출 연속 프레임 수

        # FPS 측정
        self._frame_count  = 0
        self._fps_t0       = time.perf_counter()
        self._fps          = 0.0

        # AUTOSTART=1 환경변수가 있으면 's' 없이 바로 활성화 (Docker 배포용)
        self._active = os.environ.get('AUTOSTART', '').strip() in ('1', 'true', 'yes')
        threading.Thread(target=self._wait_start_key, daemon=True).start()

        if self._active:
            self.get_logger().info("LaneDetectNode 준비 완료 — AUTOSTART 모드: 자동 주행")
        else:
            self.get_logger().info("LaneDetectNode 준비 완료 — 's' 키를 눌러 주행 시작")

    def _wait_start_key(self):
        """키 입력 루프: 's' 시작/재시작, 'q' 제어 정지, ESC/Ctrl+C 종료."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            print("\n[KEY] 's' = 시작  |  'q' = 제어 정지  |  ESC/Ctrl+C = 종료", flush=True)
            while True:
                ch = sys.stdin.read(1)
                if ch in ('s', 'S'):
                    self._active = True
                    print("\n[KEY] 주행 시작!", flush=True)
                elif ch in ('q', 'Q'):
                    self._active = False
                    print("\n[KEY] 제어 정지. 's'를 눌러 재시작.", flush=True)
                elif ch in ('\x03', '\x1b'):   # Ctrl+C / ESC
                    self._active = False
                    print("\n[KEY] 종료", flush=True)
                    break
        except Exception:
            self._active = True   # 터미널 없는 환경에서는 바로 활성화
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception:
                pass

    def _find_ncnn(self) -> str | None:
        runs_dir = ROOT / "models" / "runs"
        candidates = sorted(
            list(runs_dir.glob("lane_det*/weights/best_ncnn_model")) +
            list(runs_dir.glob("lane_seg*/weights/best_ncnn_model")),
            key=lambda p: p.stat().st_mtime,
        )
        return str(candidates[-1]) if candidates else None

    def _cb_image(self, msg: RosImage):
        try:
            import cv2
            if msg.encoding in ('bgra8', 'rgba8', 'xrgb8888'):
                raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                code = cv2.COLOR_RGBA2BGR if msg.encoding == 'rgba8' else cv2.COLOR_BGRA2BGR
                bgr = cv2.cvtColor(raw, code)
            else:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")
            return

        if os.environ.get('CAMERA_FLIP') == '1':
            bgr = cv2.flip(bgr, -1)  # 180° 회전 (RPi5 IMX219 PiSP 파이프라인 보정)

        # FPS 측정 (100프레임마다 출력)
        self._frame_count += 1
        if self._frame_count % 100 == 0:
            elapsed = time.perf_counter() - self._fps_t0
            self._fps = 100.0 / elapsed if elapsed > 0 else 0.0
            self._fps_t0 = time.perf_counter()
            infer_ms = self.detector.last_infer_ms if self.detector else 0.0
            self.get_logger().info(
                f"[PERF] 처리속도: {self._fps:.1f} FPS | "
                f"ONNX 추론: {infer_ms:.1f} ms ({1000/infer_ms:.0f} FPS 등가)"
                if infer_ms > 0 else
                f"[PERF] 처리속도: {self._fps:.1f} FPS"
            )

        # ── 1) 차선 bbox 검출 ──────────────────────────────────────────────────
        if self.detector is not None:
            boxes = self.detector.infer(bgr)
            offset, info = compute_lane_offset(boxes, IMG_W)
            # YOLO 미검출 시 흰색 픽셀 기반 폴백
            if offset is None:
                offset, info = fallback_lane_offset(bgr)
        else:
            boxes = []
            offset, info = fallback_lane_offset(bgr)

        # ── 1b) 차선별 독립 추적 ─────────────────────────────────────────────
        raw_lx = info.get("left_cx")  if info else None
        raw_rx = info.get("right_cx") if info else None
        have_ref = (self._last_lx is not None or self._last_rx is not None)

        if have_ref:
            # 이전 위치에서 LANE_TRACK_THRESH 초과 → 해당 차선 검출 무시, 이전 위치 유지
            if self._last_lx is not None and raw_lx is not None:
                if abs(raw_lx - self._last_lx) > LANE_TRACK_THRESH:
                    raw_lx = None
            if self._last_rx is not None and raw_rx is not None:
                if abs(raw_rx - self._last_rx) > LANE_TRACK_THRESH:
                    raw_rx = None

        # 감지된 경우만 위치 갱신, 미감지 시 이전 위치 유지
        if raw_lx is not None:
            self._last_lx = raw_lx
        if raw_rx is not None:
            self._last_rx = raw_rx

        # 유지된 차선 위치로 오프셋 재계산
        if self._last_lx is not None and self._last_rx is not None:
            mid    = (self._last_lx + self._last_rx) / 2
            offset = float(np.clip((mid - IMG_W / 2) / (IMG_W / 2), -1, 1))
            info   = {"left_cx": self._last_lx, "right_cx": self._last_rx, "mid_cx": mid}
            label_hint = "LANE"
        elif self._last_lx is not None:
            mid    = self._last_lx + IMG_W * 0.25
            offset = float(np.clip((mid - IMG_W / 2) / (IMG_W / 2), -1, 1))
            info   = {"left_cx": self._last_lx, "right_cx": None, "mid_cx": mid}
            label_hint = "TRACK_L"
        elif self._last_rx is not None:
            mid    = self._last_rx - IMG_W * 0.25
            offset = float(np.clip((mid - IMG_W / 2) / (IMG_W / 2), -1, 1))
            info   = {"left_cx": None, "right_cx": self._last_rx, "mid_cx": mid}
            label_hint = "TRACK_R"
        else:
            offset = self._last_offset or 0.0
            info   = {}
            label_hint = "MISS"

        self._last_offset = offset

        # ── 2) 차선 중심 오프셋 → 조향 ──────────────────────────────────────
        def _steer(off: float) -> float:
            nonlinear = math.copysign(abs(off) ** STEER_EXP, off)
            return float(-np.clip(KP_ANGULAR * nonlinear, -MAX_ANGULAR, MAX_ANGULAR))

        ang   = _steer(offset)
        turn_factor = max(0.0, 1.0 - abs(offset))
        speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * turn_factor
        if self._lost_frames > 0:
            speed = MIN_SPEED
        label = label_hint

        # 's' 키 입력 전: 정지 유지, 검출 결과는 디버그 이미지로 계속 표시
        if not self._active:
            self._publish_cmd(0.0, 0.0)
            self._publish_debug(bgr, boxes, info, offset or 0.0, f"STANDBY|{label}")
            return

        self._publish_cmd(speed, ang)
        self._publish_debug(bgr, boxes, info, offset or 0.0, label)

    def _publish_cmd(self, linear: float, angular: float):
        if _USE_STAMPED:
            msg = TwistStamped()
            msg.header.stamp   = self.get_clock().now().to_msg()
            msg.twist.linear.x = float(linear)
            msg.twist.angular.z = float(angular)
        else:
            msg = Twist()
            msg.linear.x  = float(linear)
            msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _publish_debug(self, bgr: np.ndarray, boxes: list[dict],
                       info: dict, offset: float, label: str):

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
