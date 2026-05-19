#!/usr/bin/env python3
"""
obstacle_node.py — LiDAR 기반 장애물 감지 노드

토픽:
  구독: /scan
  발행: /obstacle/state (std_msgs/String)
    포맷: "clear" | "warn:1" | "warn:-1" | "avoid:1" | "avoid:-1" | "stop:1" | "stop:-1"
    방향: 1 = 오른쪽으로 회피, -1 = 왼쪽으로 회피
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# ─── LiDAR 장애물 파라미터 ─────────────────────────────────────────────────────
OBS_WARN  = 0.85
OBS_AVOID = 0.65
OBS_STOP  = 0.30
FRONT_DEG = 22   # 전방 감지 각도 (±22°)
SIDE_DEG  = 70   # 회피 방향 판단 각도


def scan_obstacle_state(ranges, angle_min, angle_increment):
    """LaserScan 데이터로 장애물 상태와 회피 방향을 반환한다."""
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


class ObstacleNode(Node):

    def __init__(self):
        super().__init__('obstacle_node')

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(LaserScan, '/scan', self._cb_scan, best_effort_qos)
        self.state_pub = self.create_publisher(String, '/obstacle/state', 10)

        self.get_logger().info('ObstacleNode 준비 완료')

    def _cb_scan(self, msg: LaserScan):
        state, direction = scan_obstacle_state(
            list(msg.ranges), msg.angle_min, msg.angle_increment)

        payload = f'{state}:{direction}' if direction is not None else state

        out = String()
        out.data = payload
        self.state_pub.publish(out)

        if state != 'clear':
            self.get_logger().info(f'[OBS] {payload}')


def main():
    rclpy.init()
    node = ObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
