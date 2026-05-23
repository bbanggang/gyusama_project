#!/usr/bin/env python3
"""
behavior_manager.py — APF(인공 포텐셜 필드) 기반 차선유지 + 장애물회피 노드

토픽:
  구독: /lane/cmd_vel (Twist|TwistStamped), /obstacle/state (std_msgs/String)
  발행: /cmd_vel (Twist|TwistStamped)

제어 법칙 (연속 합성):
  angular = clip(lane_angular + K_REP * fy,  ±MAX_ANGULAR)
  linear  = lane_linear * speed_factor(min_front)

  - lane_angular: 차선 중심으로 끄는 인력 (항상 살아있어 회피 후 자동 복귀)
  - K_REP * fy : 장애물이 미는 척력 (가까울수록 강함, 차선 폭 내로 제한)
  - speed_factor: 장애물 근접 시 감속, STOP_DIST 이하면 정지
"""

import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String

_USE_STAMPED = os.environ.get('CMD_VEL_STAMPED', '0') == '1'

# ─── YAML 파라미터 로드 (config/apf_params.yaml) ──────────────────────────────
def _load_params():
    cfg_path = Path(__file__).resolve().parents[2] / 'config' / 'apf_params.yaml'
    defaults = dict(
        max_angular=0.5, k_rep=0.8, lane_min=0.35,
        slow_dist=0.60, stop_dist=0.22, obs_timeout=0.5,
    )
    if not cfg_path.exists():
        return defaults
    try:
        import yaml
        data = yaml.safe_load(cfg_path.read_text()) or {}
        p = data.get('behavior_manager', {})
        return {k: p.get(k, v) for k, v in defaults.items()}
    except Exception:
        return defaults

_P = _load_params()

# ─── 제어 파라미터 ─────────────────────────────────────────────────────────────
MAX_ANGULAR = float(_P['max_angular'])
K_REP       = float(_P['k_rep'])
LANE_MIN    = float(_P['lane_min'])
SLOW_DIST   = float(_P['slow_dist'])
STOP_DIST   = float(_P['stop_dist'])
OBS_TIMEOUT = float(_P['obs_timeout'])


def _clip(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class BehaviorManager(Node):

    def __init__(self):
        super().__init__('behavior_manager')

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        msg_type = TwistStamped if _USE_STAMPED else Twist
        self.create_subscription(msg_type, '/lane/cmd_vel',   self._cb_lane, best_effort_qos)
        self.create_subscription(String,   '/obstacle/state', self._cb_obs,  10)
        self.cmd_pub = self.create_publisher(msg_type, '/cmd_vel', 10)

        self._lane_linear  = 0.0
        self._lane_angular = 0.0
        self._rep_fx       = 0.0      # 장애물 전방 척력
        self._rep_fy       = 0.0      # 장애물 횡방향 척력 (+좌)
        self._min_front    = 99.0     # 전방 최소거리
        self._has_obs      = False    # 영향권 내 장애물 존재 여부
        self._obs_stamp    = 0.0

        self.get_logger().info(
            f'BehaviorManager(APF) 준비 완료 | K_REP={K_REP} MAX_ANG={MAX_ANGULAR} '
            f'SLOW={SLOW_DIST} STOP={STOP_DIST}')

    # ── 구독 콜백 ──────────────────────────────────────────────────────────────

    def _cb_lane(self, msg):
        if _USE_STAMPED:
            self._lane_linear  = msg.twist.linear.x
            self._lane_angular = msg.twist.angular.z
        else:
            self._lane_linear  = msg.linear.x
            self._lane_angular = msg.angular.z
        self._select_and_publish()

    def _cb_obs(self, msg: String):
        self._obs_stamp = time.monotonic()
        if msg.data == 'clear':
            self._has_obs = False
            return
        try:
            fx, fy, mf = (float(x) for x in msg.data.split(':'))
        except (ValueError, TypeError):
            self._has_obs = False
            return
        self._rep_fx, self._rep_fy, self._min_front = fx, fy, mf
        self._has_obs = True

    # ── APF 합성 제어 ──────────────────────────────────────────────────────────

    def _select_and_publish(self):
        # 타임아웃 시 장애물 정보 무효화
        active = self._has_obs and (time.monotonic() - self._obs_stamp <= OBS_TIMEOUT)

        if not active:
            self._publish(self._lane_linear, self._lane_angular)
            return

        # 근접도 factor: SLOW_DIST→1.0(멀다), STOP_DIST→0.0(가깝다)
        factor = _clip((self._min_front - STOP_DIST) / (SLOW_DIST - STOP_DIST),
                       0.0, 1.0)

        # 장애물 근접 시 차선 인력 감쇠 → 척력이 회피를 주도 (곡선서 인력에 안 짐)
        lane_w  = LANE_MIN + (1.0 - LANE_MIN) * factor
        angular = _clip(lane_w * self._lane_angular + K_REP * self._rep_fy,
                        -MAX_ANGULAR, MAX_ANGULAR)

        # 근접 시 감속 (STOP_DIST 이하면 정지)
        linear = self._lane_linear * factor

        self.get_logger().info(
            f'[APF] fy={self._rep_fy:+.2f} d={self._min_front:.2f} '
            f'laneW={lane_w:.2f} → spd={linear:.2f} ang={angular:+.2f}')
        self._publish(linear, angular)

    # ── 발행 헬퍼 ──────────────────────────────────────────────────────────────

    def _publish(self, linear: float, angular: float):
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


def main():
    rclpy.init()
    node = BehaviorManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
