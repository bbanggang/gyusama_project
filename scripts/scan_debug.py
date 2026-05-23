#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class D(Node):
    def __init__(self):
        super().__init__('scan_debug')
        self.create_subscription(LaserScan, '/scan', self.cb, 10)

    def cb(self, m):
        n = len(m.ranges)
        valid = [(i, r) for i, r in enumerate(m.ranges) if 0.03 < r < 30]
        if not valid:
            print('no valid ranges')
            return
        min_i, min_r = min(valid, key=lambda x: x[1])
        min_deg = math.degrees(m.angle_min + min_i * m.angle_increment)

        def idx(d):
            return int(round((math.radians(d) - m.angle_min) / m.angle_increment)) % n
        fr = [m.ranges[i % n] for i in range(idx(-35), idx(35) + 1)
              if 0.03 < m.ranges[i % n] < 30]
        print(f'all_min={min_r:.3f} at {min_deg:+.1f}°  |  front_cnt={len(fr)}')

rclpy.init()
rclpy.spin(D())
