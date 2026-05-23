#!/usr/bin/env python3
"""
monitor_nodes.py — 노드 헬스 및 토픽 지연시간 모니터
=====================================================
실행: python3 scripts/monitor_nodes.py

감시 토픽:
  /camera/image_raw  → 카메라 FPS
  /lane/cmd_vel      → 추론 FPS (lane_detect 출력)
  /obstacle/state    → 장애물 노드 상태 및 발행 주기
  /cmd_vel           → behavior_manager 최종 출력

출력 예시:
  [14:32:01] camera=15.2fps  lane=14.8fps  obs=12.1fps  cmd=14.8fps
             obs_state: fx=-0.12 fy=+0.34 d=0.52  latch 활성
"""

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
import os

_USE_STAMPED = os.environ.get('CMD_VEL_STAMPED', '0') == '1'

WINDOW = 3.0   # FPS 측정 슬라이딩 윈도우 (초)


class TopicRate:
    def __init__(self):
        self._times: list[float] = []
        self._lock = threading.Lock()

    def tick(self):
        now = time.monotonic()
        with self._lock:
            self._times.append(now)
            cutoff = now - WINDOW
            self._times = [t for t in self._times if t >= cutoff]

    def fps(self) -> float:
        with self._lock:
            now = time.monotonic()
            cutoff = now - WINDOW
            valid = [t for t in self._times if t >= cutoff]
            return len(valid) / WINDOW if valid else 0.0

    def last_seen(self) -> float:
        with self._lock:
            return self._times[-1] if self._times else 0.0


class MonitorNode(Node):

    def __init__(self):
        super().__init__('monitor_node')

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.rates = {
            'camera': TopicRate(),
            'lane':   TopicRate(),
            'obs':    TopicRate(),
            'cmd':    TopicRate(),
            'scan':   TopicRate(),
        }
        self._obs_last  = 'N/A'
        self._cmd_last  = (0.0, 0.0)
        self._lane_last = (0.0, 0.0)

        self.create_subscription(Image,  '/camera/image_raw', lambda _: self.rates['camera'].tick(), best_effort)
        self.create_subscription(LaserScan, '/scan', lambda _: self.rates['scan'].tick(), best_effort)

        cmd_type = TwistStamped if _USE_STAMPED else Twist
        self.create_subscription(cmd_type, '/lane/cmd_vel', self._cb_lane, best_effort)
        self.create_subscription(cmd_type, '/cmd_vel',      self._cb_cmd,  best_effort)
        self.create_subscription(String, '/obstacle/state', self._cb_obs, 10)

        self.create_timer(2.0, self._print_status)
        self.get_logger().info('모니터 시작 (2초 간격 출력)')

    def _cb_lane(self, msg):
        self.rates['lane'].tick()
        if _USE_STAMPED:
            self._lane_last = (msg.twist.linear.x, msg.twist.angular.z)
        else:
            self._lane_last = (msg.linear.x, msg.angular.z)

    def _cb_cmd(self, msg):
        self.rates['cmd'].tick()
        if _USE_STAMPED:
            self._cmd_last = (msg.twist.linear.x, msg.twist.angular.z)
        else:
            self._cmd_last = (msg.linear.x, msg.angular.z)

    def _cb_obs(self, msg: String):
        self.rates['obs'].tick()
        self._obs_last = msg.data

    def _print_status(self):
        now = time.monotonic()
        dead = {}
        for name, r in self.rates.items():
            elapsed = now - r.last_seen()
            dead[name] = elapsed > 5.0

        cam_fps  = self.rates['camera'].fps()
        scan_fps = self.rates['scan'].fps()
        lane_fps = self.rates['lane'].fps()
        obs_fps  = self.rates['obs'].fps()
        cmd_fps  = self.rates['cmd'].fps()

        def _tag(name, fps):
            if dead[name]:
                return f'\033[31m{name}=DEAD\033[0m'
            color = '\033[32m' if fps >= 10 else '\033[33m' if fps >= 5 else '\033[31m'
            return f'{color}{name}={fps:.1f}fps\033[0m'

        ts = time.strftime('%H:%M:%S')
        line1 = (f'[{ts}]  '
                 f'{_tag("camera", cam_fps)}  '
                 f'{_tag("scan", scan_fps)}  '
                 f'{_tag("lane", lane_fps)}  '
                 f'{_tag("obs", obs_fps)}  '
                 f'{_tag("cmd", cmd_fps)}')

        # 장애물 상태 파싱
        obs_str = self._obs_last
        if obs_str.startswith('clear') or obs_str == 'N/A':
            obs_detail = f'obs_state: {obs_str}'
        else:
            try:
                fx, fy, d = (float(x) for x in obs_str.split(':'))
                flag = '\033[33m[AVOID]\033[0m' if abs(fy) > 0.1 else ''
                obs_detail = f'obs_state: fx={fx:+.2f} fy={fy:+.2f} d={d:.2f}m {flag}'
            except Exception:
                obs_detail = f'obs_state: {obs_str}'

        spd, ang = self._cmd_last
        cmd_detail = f'cmd_vel:   linear={spd:.3f} angular={ang:+.3f}'

        print(f'\n{line1}')
        print(f'           {obs_detail}')
        print(f'           {cmd_detail}')


def main():
    rclpy.init()
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
