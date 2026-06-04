#!/usr/bin/env python3
"""
lane_detect_hailo.py — YOLOv8-detect HailoRT(NPU) 추론 기반 차선 추종 노드
========================================================================
lane_detect_ncnn.py 와 동일한 ROS2 노드 인터페이스를 제공한다.
추론 엔진만 NCNN → Hailo-8/8L NPU(HailoRT) 로 교체.

전후처리(차선 중심 오프셋, 폴백, ROI 등)는 lane_common.py 의 공통 함수 재사용.

모델 형식: HEF (Hailo Executable Format)
  - Hailo DFC 로 best.onnx → best.hef 변환 (--hw-arch hailo8l)
  - scripts/compile_hef.sh 참고

HEF 출력 형식 두 가지 지원:
  ① NMS 미포함:   shape (1, 5, 8400)   — ONNX/NCNN 과 동일
  ② NMS 포함:     shape (1, N, 6) 또는 (N, 6)  — [x, y, w, h, conf, cls]
                  (Hailo Model Zoo `hailomz compile yolov8s` 디폴트)

토픽:
  구독: /camera/image_raw
  발행: /lane/cmd_vel | /cmd_vel(DIRECT_CMD_VEL=1) , /lane/debug_image

환경변수:
  HEF_MODEL          .hef 파일 경로 (미설정 시 lane_det*/weights/*.hef 자동 탐색)
  YOLO_SZ            HEF 입력 한 변 (640 디폴트, HEF 메타와 일치해야 함)
  PUBLISH_DEBUG      0/1 — 디버그 이미지 발행 토글 (FPS 영향 큼)
  DIRECT_CMD_VEL     0/1 — 1=behavior-node 우회, /cmd_vel 직접 발행
  AUTOSTART          0/1 — 1=`s` 키 없이 즉시 주행
  CAMERA_FLIP        0/1 — 1=IMX219 180° 회전 보정
  CMD_VEL_STAMPED    0/1 — 1=TwistStamped(RPi5 Jazzy), 0=Twist(Isaac Sim)
"""

from __future__ import annotations

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

from cv_bridge import CvBridge

# 공통 순수 함수 (lane_common.py — plan.md Phase 4 선행 리팩토링)
from lane_common import (
    ROOT,
    load_lane_params,
    compute_lane_offset,
    fallback_lane_offset,
    nms as common_nms,
)


# ─── 환경 ──────────────────────────────────────────────────────────────────────
_USE_STAMPED = os.environ.get('CMD_VEL_STAMPED', '0') == '1'

IMG_W, IMG_H = 640, 480
YOLO_SZ      = int(os.environ.get('YOLO_SZ', '640'))

_LP = load_lane_params()
CONF_THRESH        = float(_LP['conf_thresh'])
LANE_ROI_TOP_RATIO = float(_LP['lane_roi_top'])
MAX_SPEED          = float(_LP['max_speed'])
MIN_SPEED          = float(_LP['min_speed'])
MAX_ANGULAR        = float(_LP['max_angular'])
KP_ANGULAR         = float(_LP['kp_angular'])
STEER_EXP          = float(_LP['steer_exp'])
LANE_TRACK_THRESH  = int(_LP['lane_track_thresh'])
NMS_IOU_THR        = 0.45


# ═══════════════════════════════════════════════════════════════════════════════
# ── HailoRT 추론 ──────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

class HailoYoloDetector:
    """YOLOv8-detect HEF 모델을 HailoRT VStream 으로 추론."""

    def __init__(self, hef_path: str):
        from hailo_platform import (
            HEF, VDevice, FormatType, HailoStreamInterface,
            InferVStreams, InputVStreamParams, OutputVStreamParams,
            ConfigureParams,
        )

        self.hef = HEF(hef_path)
        self.vdev = VDevice()
        cfg = ConfigureParams.create_from_hef(self.hef, interface=HailoStreamInterface.PCIe)
        self.network_groups = self.vdev.configure(self.hef, cfg)
        self.ng = self.network_groups[0]

        # 입출력 vstream 메타데이터
        self.in_infos  = self.hef.get_input_vstream_infos()
        self.out_infos = self.hef.get_output_vstream_infos()
        if len(self.in_infos) != 1:
            raise RuntimeError(f"[HAILO] 입력 vstream 1개 기대했으나 {len(self.in_infos)}개")

        # vstream 파라미터 — 입력 UINT8 (Hailo Model Zoo 디폴트), 출력 FLOAT32
        self.in_params  = InputVStreamParams.make_from_network_group(
            self.ng, format_type=FormatType.UINT8)
        self.out_params = OutputVStreamParams.make_from_network_group(
            self.ng, format_type=FormatType.FLOAT32)

        # network group 활성화 + infer pipeline 생성
        self._ng_active_ctx = self.ng.activate(self.ng.create_params())
        self._ng_active = self._ng_active_ctx.__enter__()
        self._infer_ctx = InferVStreams(self.ng, self.in_params, self.out_params)
        self.infer_pipeline = self._infer_ctx.__enter__()

        self.last_infer_ms = 0.0
        self._input_name = self.in_infos[0].name
        # HEF 메타에서 입력 한 변 자동 감지 (YOLO_SZ 환경변수 무관 — 견고)
        in_shape = self.in_infos[0].shape  # 보통 (H, W, C)
        self.input_size = int(in_shape[0])
        print(f"[HAILO] HEF 로드: {hef_path}")
        print(f"[HAILO] 입력: {self._input_name} shape={in_shape} UINT8 NHWC → resize 한 변 {self.input_size}")
        for o in self.out_infos:
            print(f"[HAILO] 출력: {o.name} shape={o.shape}")

        # ── DFL 후처리용 anchor grid (lazy 캐시) ─────────────────────
        # YOLOv8: 3개 stride (8, 16, 32) × feature map = 총 anchor 수
        # 320 입력 → 40×40 + 20×20 + 10×10 = 2100 anchor
        # 640 입력 → 80×80 + 40×40 + 20×20 = 8400 anchor
        self._anchor_grid: np.ndarray | None = None   # shape (N, 3) — (cx, cy, stride)

    def _get_anchor_grid(self, input_size: int) -> np.ndarray:
        """YOLOv8 표준 anchor grid 생성 + 캐시.

        Returns
        -------
        np.ndarray (N, 3)  → (center_x, center_y, stride)  in 입력 픽셀 단위
        """
        if self._anchor_grid is not None and len(self._anchor_grid) == self._cached_anchor_count(input_size):
            return self._anchor_grid

        anchors = []
        for stride in (8, 16, 32):
            feat = input_size // stride
            ys, xs = np.meshgrid(np.arange(feat), np.arange(feat), indexing='ij')
            cx = (xs.flatten() + 0.5) * stride
            cy = (ys.flatten() + 0.5) * stride
            s  = np.full_like(cx, stride, dtype=np.float32)
            anchors.append(np.stack([cx, cy, s], axis=1))
        self._anchor_grid = np.concatenate(anchors, axis=0).astype(np.float32)
        return self._anchor_grid

    @staticmethod
    def _cached_anchor_count(input_size: int) -> int:
        return sum((input_size // s) ** 2 for s in (8, 16, 32))

    def _decode_dfl_outputs(self, bbox_raw: np.ndarray, conf_raw: np.ndarray,
                            input_size: int, conf_thresh: float
                            ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Hailo HEF의 raw 2개 출력 → bbox + conf 디코딩.

        Parameters
        ----------
        bbox_raw : (N, 64)   — DFL encoded (16 bins × 4 coords)
        conf_raw : (N,)      — sigmoid 적용된 confidence
        input_size : int     — HEF 입력 크기 (한 변)
        conf_thresh : float  — 임계값

        Returns
        -------
        (cx, cy, w, h, scores)  모두 (M,)  — 입력 픽셀 단위, M=conf 필터 통과한 수
        """
        # 1) Conf 필터 (메모리 절약)
        keep = conf_raw > conf_thresh
        if not keep.any():
            return (np.array([]),) * 5

        bbox_kept = bbox_raw[keep]            # (M, 64)
        conf_kept = conf_raw[keep]            # (M,)

        # 2) DFL decode: (M, 64) → (M, 4, 16) → softmax → expectation
        bbox_dfl = bbox_kept.reshape(-1, 4, 16)
        # numerical stable softmax
        mx = bbox_dfl.max(axis=-1, keepdims=True)
        exp_d = np.exp(bbox_dfl - mx)
        sm    = exp_d / exp_d.sum(axis=-1, keepdims=True)
        bins  = np.arange(16, dtype=np.float32)
        dist  = (sm * bins).sum(axis=-1)      # (M, 4) — (l, t, r, b) in stride 단위

        # 3) Anchor 매핑
        anchors = self._get_anchor_grid(input_size)
        anchors_kept = anchors[keep]          # (M, 3) — (cx, cy, stride)
        a_cx = anchors_kept[:, 0]
        a_cy = anchors_kept[:, 1]
        stride = anchors_kept[:, 2]

        # 4) bbox: anchor + DFL distance × stride
        l = dist[:, 0] * stride
        t = dist[:, 1] * stride
        r = dist[:, 2] * stride
        b = dist[:, 3] * stride

        x1 = a_cx - l;  y1 = a_cy - t
        x2 = a_cx + r;  y2 = a_cy + b

        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        w  = x2 - x1
        h  = y2 - y1
        return cx, cy, w, h, conf_kept

    def close(self):
        try:
            self._infer_ctx.__exit__(None, None, None)
        except Exception:
            pass
        try:
            self._ng_active_ctx.__exit__(None, None, None)
        except Exception:
            pass
        try:
            self.vdev.release()
        except Exception:
            pass

    def __del__(self):
        self.close()

    def infer(self, bgr: np.ndarray) -> list[dict]:
        """원본 BGR 이미지 → [{cx,cy,w,h,conf}] (원본 픽셀 단위)."""
        import cv2

        # ── 1) 전처리: resize + RGB + UINT8 NHWC (HEF 메타 기반 입력 크기) ─
        if bgr.ndim == 3 and bgr.shape[-1] == 4:
            bgr = bgr[:, :, :3]
        sz = self.input_size  # HEF 메타에서 추출 — YOLO_SZ 환경변수 무관
        img = cv2.resize(bgr, (sz, sz))
        if img.ndim == 3 and img.shape[-1] == 4:
            img = img[:, :, :3]
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        input_arr = np.ascontiguousarray(rgb[np.newaxis, ...], dtype=np.uint8)

        # 첫 호출 시 한 번만 shape/dtype 로그 (검증용)
        if not hasattr(self, '_debug_logged'):
            print(f"[HAILO DEBUG] bgr shape={bgr.shape} dtype={bgr.dtype}")
            print(f"[HAILO DEBUG] resize 후 img shape={img.shape}")
            print(f"[HAILO DEBUG] input_arr shape={input_arr.shape} nbytes={input_arr.nbytes}")
            self._debug_logged = True

        input_data = {self._input_name: input_arr}

        # ── 2) 추론 ─────────────────────────────────────────────────
        _t0 = time.perf_counter()
        results = self.infer_pipeline.infer(input_data)
        self.last_infer_ms = (time.perf_counter() - _t0) * 1000.0

        # ── 3) 출력 해석 (세 가지 형식 모두 지원) ───────────────────
        # results: dict[name -> ndarray]
        h_orig, w_orig = bgr.shape[:2]
        sx = w_orig / self.input_size
        sy = h_orig / self.input_size
        boxes: list[dict] = []

        # ── 형식 ③: Hailo DFL 원시 출력 두 개 (concat14 64ch + activation 1ch) ──
        # (lane_det-6 HEF: hailo parser 가 yolov8 detection head 직전에서 끊은 결과)
        if len(results) == 2:
            bbox_raw: np.ndarray | None = None    # (1, 2100, 64)
            conf_raw: np.ndarray | None = None    # (1, 2100, 1) 또는 (1, 2100)
            for name, arr in results.items():
                last = arr.shape[-1]
                if last == 64:
                    bbox_raw = arr.reshape(-1, 64)        # (N, 64)
                elif last == 1 or arr.ndim == 2:
                    conf_raw = arr.reshape(-1)            # (N,)

            if bbox_raw is not None and conf_raw is not None:
                cx_in, cy_in, w_in, h_in, scores = self._decode_dfl_outputs(
                    bbox_raw, conf_raw, self.input_size, CONF_THRESH)

                if scores.size > 0:
                    # ROI 컷 (입력 픽셀 단위 cy 기준)
                    roi_top_px = self.input_size * LANE_ROI_TOP_RATIO
                    roi_keep = cy_in > roi_top_px
                    if roi_keep.any():
                        cx_in = cx_in[roi_keep]; cy_in = cy_in[roi_keep]
                        w_in  = w_in[roi_keep];  h_in  = h_in[roi_keep]
                        scores = scores[roi_keep]

                        # NMS (lane class 한 종류)
                        keep = common_nms(cx_in, cy_in, w_in, h_in, scores,
                                          iou_thr=NMS_IOU_THR)
                        for i in keep:
                            boxes.append({
                                "cx":   float(cx_in[i] * sx),
                                "cy":   float(cy_in[i] * sy),
                                "w":    float(w_in[i]  * sx),
                                "h":    float(h_in[i]  * sy),
                                "conf": float(scores[i]),
                            })

                # 좌/우 zone-best
                zone_best: dict[int, dict] = {}
                for box in boxes:
                    zone = 0 if box["cx"] < w_orig / 2 else 1
                    if zone not in zone_best or box["conf"] > zone_best[zone]["conf"]:
                        zone_best[zone] = box
                return list(zone_best.values())

        # 단일 출력 가정 (yolov8 nc=1 컴파일 결과)
        out_arr = next(iter(results.values()))

        # 형식 ②: NMS 포함 [batch, N, 6] 또는 [N, 6]  ([x, y, w, h, conf, cls])
        # 좌표는 보통 입력 픽셀 또는 [0,1] 정규화 — 둘 다 처리
        if (out_arr.ndim == 3 and out_arr.shape[-1] in (5, 6)) \
           or (out_arr.ndim == 2 and out_arr.shape[-1] in (5, 6)):
            det = out_arr.reshape(-1, out_arr.shape[-1])
            conf_col = 4
            # 좌표 정규화 여부 추정: 최대값이 2 이하면 [0,1] 정규화로 본다
            xy_max = np.abs(det[:, :4]).max() if det.size else 0.0
            normalized = xy_max < 2.0

            for row in det:
                if row[conf_col] < CONF_THRESH:
                    continue
                cx, cy, w, h = row[:4]
                if normalized:
                    cx *= w_orig;  cy *= h_orig
                    w  *= w_orig;  h  *= h_orig
                else:
                    cx *= sx;  cy *= sy
                    w  *= sx;  h  *= sy
                # ROI 컷 (mid_cy < ROI_TOP 인 검출 무시 — 원본 픽셀 기준)
                if cy < h_orig * LANE_ROI_TOP_RATIO:
                    continue
                boxes.append({
                    "cx": float(cx), "cy": float(cy),
                    "w":  float(w),  "h":  float(h),
                    "conf": float(row[conf_col]),
                })

            # 좌/우 zone 별 best 1개만 유지 (NCNN 코드와 동일 정책)
            zone_best: dict[int, dict] = {}
            for box in boxes:
                zone = 0 if box["cx"] < w_orig / 2 else 1
                if zone not in zone_best or box["conf"] > zone_best[zone]["conf"]:
                    zone_best[zone] = box
            return list(zone_best.values())

        # 형식 ①: NMS 미포함 raw output [1, 5, 8400] (ONNX/NCNN 호환)
        pred = out_arr
        if pred.ndim == 4:
            pred = pred[0]
        if pred.shape[0] != 5 and pred.shape[-1] == 5:
            # (1, 8400, 5) → (5, 8400)
            pred = pred.swapaxes(-1, -2)
        if pred.ndim == 3:
            pred = pred[:, 0, :]

        raw_cx = pred[0];  raw_cy = pred[1]
        raw_w  = pred[2];  raw_h  = pred[3]
        scores = pred[4]

        roi_top_px = self.input_size * LANE_ROI_TOP_RATIO
        pre_idx = np.where((scores > CONF_THRESH) & (raw_cy > roi_top_px))[0]
        if pre_idx.size == 0:
            return []

        keep = common_nms(
            raw_cx[pre_idx], raw_cy[pre_idx],
            raw_w[pre_idx],  raw_h[pre_idx],
            scores[pre_idx], iou_thr=NMS_IOU_THR,
        )
        valid_idx = pre_idx[keep]

        for i in valid_idx:
            boxes.append({
                "cx":   float(raw_cx[i] * sx),
                "cy":   float(raw_cy[i] * sy),
                "w":    float(raw_w[i]  * sx),
                "h":    float(raw_h[i]  * sy),
                "conf": float(scores[i]),
            })

        zone_best: dict[int, dict] = {}
        for box in boxes:
            zone = 0 if box["cx"] < w_orig / 2 else 1
            if zone not in zone_best or box["conf"] > zone_best[zone]["conf"]:
                zone_best[zone] = box
        return list(zone_best.values())


# ═══════════════════════════════════════════════════════════════════════════════
# ── ROS2 노드 ─────────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════════════

class LaneDetectNode(Node):

    def __init__(self):
        super().__init__("lane_detect_node")

        hef_path = os.environ.get('HEF_MODEL') or self._find_hef()
        if hef_path:
            self.get_logger().info(f"HEF 모델 사용: {hef_path}")
            try:
                self.detector = HailoYoloDetector(hef_path)
            except Exception as e:
                self.get_logger().error(f"HEF 로드 실패: {e}")
                self.get_logger().warn("→ 흰색 픽셀 폴백 모드로 진행")
                self.detector = None
        else:
            self.get_logger().warn(
                "HEF 모델 없음 — 흰색 픽셀 폴백 모드\n"
                "  먼저 models/runs/lane_det*/weights/*.hef 파일 확인 또는 HEF_MODEL 환경변수 지정"
            )
            self.detector = None

        self.bridge = CvBridge()

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(RosImage, "/camera/image_raw", self._cb_image, best_effort_qos)

        msg_type = TwistStamped if _USE_STAMPED else Twist
        _direct = os.environ.get('DIRECT_CMD_VEL', '0') == '1'
        _topic  = '/cmd_vel' if _direct else '/lane/cmd_vel'
        self.cmd_pub = self.create_publisher(msg_type, _topic, 10)
        if _direct:
            self.get_logger().info("[모드] DIRECT_CMD_VEL=1 — /cmd_vel 직접 발행 (behavior-node 불필요)")
        self.dbg_pub = self.create_publisher(RosImage, "/lane/debug_image", 1)

        self._start_time   = time.time()
        self._last_offset  = 0.0
        self._last_lx      = None
        self._last_rx      = None
        self._lost_frames  = 0

        self._frame_count = 0
        self._fps_t0      = time.perf_counter()
        self._fps         = 0.0

        self._active = os.environ.get('AUTOSTART', '').strip() in ('1', 'true', 'yes')
        threading.Thread(target=self._wait_start_key, daemon=True).start()

        if self._active:
            self.get_logger().info("LaneDetectNode 준비 완료 — AUTOSTART 모드: 자동 주행")
        else:
            self.get_logger().info("LaneDetectNode 준비 완료 — 's' 키를 눌러 주행 시작")

    def _wait_start_key(self):
        fd = sys.stdin.fileno()
        try:
            old = termios.tcgetattr(fd)
        except Exception:
            self._active = True   # 터미널 없는 환경
            return
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
                elif ch in ('\x03', '\x1b'):
                    self._active = False
                    print("\n[KEY] 종료", flush=True)
                    break
        except Exception:
            self._active = True
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception:
                pass

    def _find_hef(self) -> str | None:
        runs_dir = ROOT / "models" / "runs"
        candidates = sorted(
            runs_dir.glob("lane_det*/weights/*.hef"),
            key=lambda p: p.stat().st_mtime,
        )
        return str(candidates[-1]) if candidates else None

    def _cb_image(self, msg: RosImage):
        import cv2
        # cv_bridge 가 desired_encoding='bgr8' 으로 자동 변환 처리
        # (bgra8 / rgba8 / xrgb8888 등 모든 4채널 → 3채널 bgr 로 통일)
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(
                f"이미지 변환 오류: {e} (encoding={msg.encoding}, shape={msg.width}x{msg.height})")
            return
        # 안전 가드: 혹시라도 3채널 BGR uint8 이 아니면 강제 변환
        if bgr.dtype != np.uint8:
            bgr = bgr.astype(np.uint8)
        if bgr.ndim == 3 and bgr.shape[-1] == 4:
            bgr = bgr[:, :, :3]
        elif bgr.ndim == 2:
            bgr = cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)

        if os.environ.get('CAMERA_FLIP') == '1':
            bgr = cv2.flip(bgr, -1)

        # FPS 측정 (100프레임마다 출력)
        self._frame_count += 1
        if self._frame_count % 100 == 0:
            elapsed = time.perf_counter() - self._fps_t0
            self._fps = 100.0 / elapsed if elapsed > 0 else 0.0
            self._fps_t0 = time.perf_counter()
            infer_ms = self.detector.last_infer_ms if self.detector else 0.0
            self.get_logger().info(
                f"[PERF] 처리속도: {self._fps:.1f} FPS | "
                f"NPU 추론: {infer_ms:.1f} ms ({1000/infer_ms:.0f} FPS 등가) | "
                f"콜백전체: {1000/self._fps:.0f} ms"
                if infer_ms > 0 else
                f"[PERF] 처리속도: {self._fps:.1f} FPS"
            )

        # ── 1) 차선 bbox 검출 ─────────────────────────────────────────
        _t_infer = time.perf_counter()
        if self.detector is not None:
            boxes = self.detector.infer(bgr)
            offset, info = compute_lane_offset(boxes, IMG_W)
            if offset is None:
                offset, info = fallback_lane_offset(
                    bgr, LANE_ROI_TOP_RATIO, IMG_W, IMG_H)
        else:
            boxes = []
            offset, info = fallback_lane_offset(
                bgr, LANE_ROI_TOP_RATIO, IMG_W, IMG_H)

        # ── 1b) 차선별 독립 추적 (NCNN 노드와 동일 정책) ────────────
        raw_lx = info.get("left_cx")  if info else None
        raw_rx = info.get("right_cx") if info else None
        have_ref = (self._last_lx is not None or self._last_rx is not None)

        if have_ref:
            if self._last_lx is not None and raw_lx is not None:
                if abs(raw_lx - self._last_lx) > LANE_TRACK_THRESH:
                    raw_lx = None
            if self._last_rx is not None and raw_rx is not None:
                if abs(raw_rx - self._last_rx) > LANE_TRACK_THRESH:
                    raw_rx = None

        if raw_lx is not None:
            self._last_lx = raw_lx
        if raw_rx is not None:
            self._last_rx = raw_rx

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

        # ── 2) 조향 ──────────────────────────────────────────────────
        def _steer(off: float) -> float:
            nonlinear = math.copysign(abs(off) ** STEER_EXP, off)
            return float(-np.clip(KP_ANGULAR * nonlinear, -MAX_ANGULAR, MAX_ANGULAR))

        ang = _steer(offset)
        turn_factor = max(0.0, 1.0 - abs(offset))
        speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * turn_factor
        if self._lost_frames > 0:
            speed = MIN_SPEED
        label = label_hint

        if not self._active:
            self._publish_cmd(0.0, 0.0)
            self._publish_debug(bgr, boxes, info, offset or 0.0, f"STANDBY|{label}")
            return

        _t_debug = time.perf_counter()
        self._publish_cmd(speed, ang)
        if os.environ.get('PUBLISH_DEBUG', '1') != '0':
            self._publish_debug(bgr, boxes, info, offset or 0.0, label)
        if self._frame_count % 100 == 1:
            total_ms = (time.perf_counter() - _t_infer) * 1000
            debug_ms = (time.perf_counter() - _t_debug) * 1000
            infer_ms = self.detector.last_infer_ms if self.detector else 0
            self.get_logger().info(
                f"[TIME] 추론={infer_ms:.0f}ms  디버그발행={debug_ms:.0f}ms  전체콜백={total_ms:.0f}ms"
            )

    def _publish_cmd(self, linear: float, angular: float):
        if _USE_STAMPED:
            msg = TwistStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.twist.linear.x  = float(linear)
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

        for b in boxes:
            cx, cy = int(b["cx"]), int(b["cy"])
            hw, hh = int(b.get("w", 40) / 2), int(b.get("h", 40) / 2)
            cv2.rectangle(vis, (cx - hw, cy - hh), (cx + hw, cy + hh),
                          (0, 255, 0), 2)
            cv2.circle(vis, (cx, cy), 4, (0, 255, 0), -1)

        if info.get("left_cx") is not None:
            cv2.line(vis, (int(info["left_cx"]), 0), (int(info["left_cx"]), IMG_H),
                     (255, 0, 0), 1)
        if info.get("right_cx") is not None:
            cv2.line(vis, (int(info["right_cx"]), 0), (int(info["right_cx"]), IMG_H),
                     (0, 0, 255), 1)
        if "mid_cx" in info:
            mid_x = int(info["mid_cx"])
            cv2.line(vis, (mid_x, int(IMG_H * LANE_ROI_TOP_RATIO)), (mid_x, IMG_H - 1),
                     (0, 255, 255), 2)

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
        try:
            if node.detector is not None:
                node.detector.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
