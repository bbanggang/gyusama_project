#!/usr/bin/env python3
"""
behavior_manager.py — Behavior Tree 기반 우선순위 제어 노드

토픽:
  구독: /lane/cmd_vel (Twist|TwistStamped), /obstacle/state (std_msgs/String)
  발행: /cmd_vel (Twist|TwistStamped)

우선순위 (높음 → 낮음):
  stop  → 회전 정지 (선속도 0, 최대 각속도로 회피 방향 전환)
  avoid → 빠른 탈출 (MIN_SPEED*1.5, 최대 각속도)
  warn  → 차선 + 회피 50:50 블렌드 (감속)
  clear → /lane/cmd_vel 그대로 전달
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String

_USE_STAMPED = os.environ.get('CMD_VEL_STAMPED', '0') == '1'

# ─── 제어 파라미터 ─────────────────────────────────────────────────────────────
MAX_SPEED   = 0.14
MIN_SPEED   = 0.08
MAX_ANGULAR = 12.0

# /obstacle/state 토픽이 이 시간(초) 이상 오지 않으면 clear로 간주
OBS_TIMEOUT = 0.5


class BehaviorManager(Node):

    def __init__(self):
        super().__init__('behavior_manager')

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        msg_type = TwistStamped if _USE_STAMPED else Twist
        self.create_subscription(msg_type, '/lane/cmd_vel',   self._cb_lane,    best_effort_qos)
        self.create_subscription(String,   '/obstacle/state', self._cb_obs,     10)
        self.cmd_pub = self.create_publisher(msg_type, '/cmd_vel', 10)

        self._lane_linear  = 0.0
        self._lane_angular = 0.0
        self._obs_state    = 'clear'
        self._avoid_dir    = 1        # 1=오른쪽, -1=왼쪽
        self._avoid_locked = None     # 회피 중 방향 고정
        self._obs_stamp    = 0.0      # 마지막 obstacle 메시지 수신 시각

        self.get_logger().info('BehaviorManager 준비 완료')

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
        parts = msg.data.split(':')
        self._obs_state = parts[0]
        if len(parts) == 2:
            try:
                self._avoid_dir = int(parts[1])
            except ValueError:
                pass

    # ── Behavior Tree 셀렉터 ───────────────────────────────────────────────────

    def _select_and_publish(self):
        # obstacle 메시지 타임아웃 시 clear 처리
        obs = self._obs_state
        if time.monotonic() - self._obs_stamp > OBS_TIMEOUT:
            obs = 'clear'

        if obs == 'stop':
            self._avoid_locked = self._avoid_dir
            linear  = 0.0
            angular = float(self._avoid_locked) * MAX_ANGULAR
            self.get_logger().info(f'[BT] STOP → ang={angular:+.1f}')

        elif obs == 'avoid':
            if self._avoid_locked is None:
                self._avoid_locked = self._avoid_dir
            linear  = MIN_SPEED * 1.5
            angular = float(self._avoid_locked) * MAX_ANGULAR
            self.get_logger().info(f'[BT] AVOID → spd={linear:.2f} ang={angular:+.1f}')

        elif obs == 'warn':
            if self._avoid_locked is None:
                self._avoid_locked = self._avoid_dir
            avoid_ang = float(self._avoid_locked) * MAX_ANGULAR * 0.6
            angular   = 0.5 * self._lane_angular + 0.5 * avoid_ang
            linear    = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * 0.25

        else:  # clear
            self._avoid_locked = None
            linear  = self._lane_linear
            angular = self._lane_angular

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
