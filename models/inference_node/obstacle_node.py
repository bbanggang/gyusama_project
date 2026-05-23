#!/usr/bin/env python3
"""
obstacle_node.py — LiDAR 기반 장애물 척력(APF) 노드

토픽:
  구독: /scan
  발행: /obstacle/state (std_msgs/String)
    포맷: "clear"  (영향권 내 장애물 없음)
        | "fx:fy:min_front"  (합성 척력 벡터 + 전방 최소거리)
      - fx>0=전방 척력(감속 신호), fy>0=왼쪽으로 미는 힘
      - 좌표계: 로봇 기준 x=전방, y=좌측 (ROS REP-103)

APF 방식:
  영향권(INFLUENCE) 내 모든 LiDAR 포인트가 로봇을 밀어내는 척력을 합성한다.
  behavior_manager가 (차선 인력 + 이 척력)을 합쳐 조향한다.
  정면 대칭 장애물(지역최소)은 더 열린 쪽으로 탈출력을 주입해 해소한다.
"""

import math
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# ─── APF 척력 파라미터 ─────────────────────────────────────────────────────────
INFLUENCE  = 0.65   # 척력 영향권 (이 거리 밖은 무시) — 너무 일찍 반응 방지
FRONT_DEG  = 22     # 22→16: latch 유지 시간(=회전량) 추가 축소
SCAN_HALF  = 90     # 척력 합성 스캔 범위 (로봇 ±90°)
MIN_OBS_POINTS = 5  # 영향권 내 포인트 수 미만 → clear (모서리·노이즈 오탐 방지)
LATCH_DIST = 0.55   # 이 거리 안에서 회피 방향을 고정(latch) → 진동 방지
RELEASE_FRAMES = 2  # 전방이 LATCH_DIST 밖으로 연속 N프레임 멀어져야 latch 해제 (노이즈 디바운스)
FLIP_FRAMES = 4     # 반대 방위가 연속 N프레임 지속되면 회피 방향 전환 (다음 장애물 대응)
FY_EPS = 0.05       # |fy| 가 이 값 초과면 장애물 방위가 명확 → fy 부호로 방향 결정

# Isaac Sim: LiDAR 0°가 로봇 후방을 향함 → 180° 오프셋 필요 (실물=0)
# 실측 정합: offset 적용 후 idx()의 +deg = 물리적 로봇 왼쪽, -deg = 오른쪽
LIDAR_OFFSET_DEG = int(os.environ.get('LIDAR_OFFSET_DEG', '0'))

# fy 부호 규약(발행값): fy>0 → behavior_manager 좌회전(+angular), fy<0 → 우회전
#   왼쪽 장애물(+deg) 은 우회전(fy<0) 필요 → sin 항에 음부호


def compute_repulsion(ranges, angle_min, angle_increment):
    """전체 스캔에서 합성 척력 (fx, fy)·전방 최소거리·개방방향을 반환."""
    n = len(ranges)

    def idx(deg):
        return int(round((math.radians(deg + LIDAR_OFFSET_DEG) - angle_min) / angle_increment)) % n

    fx = 0.0   # +전방 (척력이 뒤로 밀면 +) — 감속용 크기
    fy = 0.0   # +좌회전 / -우회전 (behavior_manager angular 부호와 동일)
    nclose = 0
    min_front = INFLUENCE
    sum_pos = sum_neg = 0.0   # pos=물리적 좌측(deg>0), neg=물리적 우측(deg<0)
    cnt_pos = cnt_neg = 0

    for deg in range(-SCAN_HALF, SCAN_HALF + 1):
        r = ranges[idx(deg)]
        if not (0.03 < r < 30.0):
            continue
        # 좌우 개방도(탈출 방향 판정용) — 영향권 무관 전체 사용
        if deg > 0:
            sum_pos += r; cnt_pos += 1   # 물리적 좌측
        elif deg < 0:
            sum_neg += r; cnt_neg += 1   # 물리적 우측

        if r >= INFLUENCE:
            continue
        # 선형 램프 척력: 영향권 경계서 0, 접촉 시 1
        mag = (INFLUENCE - r) / INFLUENCE
        rad = math.radians(deg)
        # +deg=물리적 좌측 → 좌측 장애물은 우회전(fy<0) 필요 → sin 음부호
        fy += -mag * math.sin(rad)
        fx += -mag * math.cos(rad)   # 정면일수록 뒤로(-x) 미는 성분
        nclose += 1
        if -FRONT_DEG <= deg <= FRONT_DEG and r < min_front:
            min_front = r

    if nclose < MIN_OBS_POINTS:
        return None  # clear

    # 전역 개방방향: +1=좌측(deg>0)이 더 열림→좌회전, -1=우측이 더 열림→우회전
    avg_pos = sum_pos / cnt_pos if cnt_pos else 0.0
    avg_neg = sum_neg / cnt_neg if cnt_neg else 0.0
    open_dir = 1.0 if avg_pos > avg_neg else -1.0

    return fx, fy, min_front, open_dir


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

        self._latch_dir = None   # 회피 방향 고정값 (장애물 통과까지 유지)
        self._far_cnt   = 0      # 전방이 LATCH_DIST 밖으로 연속 멀어진 프레임 수
        self._flip_cnt  = 0      # 반대편이 연속 더 열린 프레임 수 (방향 전환 카운터)

        self.get_logger().info('ObstacleNode 준비 완료')

    def _cb_scan(self, msg: LaserScan):
        result = compute_repulsion(
            list(msg.ranges), msg.angle_min, msg.angle_increment)

        out = String()
        if result is None:
            self._latch_dir = None      # 장애물 완전히 사라짐 → latch 해제
            self._far_cnt   = 0
            out.data = 'clear'
            self.state_pub.publish(out)
            return

        fx, fy, min_front, open_dir = result
        fy_nat = fy   # 자연 척력 (방향 판정용 — override 전 보존)

        # 회피 방향 latch: 진동 방지 + 다음 장애물 위한 방향 전환(히스테리시스)
        if min_front < LATCH_DIST:
            self._far_cnt = 0
            # 장애물 방위(fy 부호)가 신뢰적. 작은 장애물은 open_dir(반평면 평균)이
            # 트랙 기하에 지배돼 부정확하므로 정면(거의 0)일 때만 폴백으로 사용.
            if abs(fy_nat) > FY_EPS:
                desired = 1.0 if fy_nat > 0 else -1.0
            elif self._latch_dir is not None:
                desired = self._latch_dir
            else:
                desired = open_dir

            if self._latch_dir is None:
                self._latch_dir = desired
                self._flip_cnt = 0
            elif desired != self._latch_dir:
                # 반대 방위가 지속(=다음 장애물) → 방향 전환
                self._flip_cnt += 1
                if self._flip_cnt >= FLIP_FRAMES:
                    self._latch_dir = desired
                    self._flip_cnt = 0
            else:
                self._flip_cnt = 0
        else:
            # 노이즈 디바운스: 연속 RELEASE_FRAMES 이상 멀어져야 해제
            self._far_cnt += 1
            if self._far_cnt >= RELEASE_FRAMES:
                self._latch_dir = None
                self._flip_cnt = 0

        if self._latch_dir is not None:
            fy = self._latch_dir * (abs(fy_nat) + abs(fx))   # 고정 방향 일관된 횡력

        out.data = f'{fx:.4f}:{fy:.4f}:{min_front:.4f}'
        self.state_pub.publish(out)
        self.get_logger().info(
            f'[OBS] fx={fx:+.2f} fy={fy:+.2f} d={min_front:.2f} latch={self._latch_dir}')


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
