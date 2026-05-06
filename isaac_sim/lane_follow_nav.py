#!/usr/bin/env python3
"""
lane_follow_nav.py — Pure Pursuit 차선 추종 + LiDAR 장애물 회피 + 정지선 감지
=============================================================================
실행:
  Terminal 1: bash launch_sim.sh
  Terminal 2: source /home/linux/gyusama-project/isaac_sim/ros2_terminal.sh
              python3 lane_follow_nav.py

토픽:
  구독:  /scan (장애물), /tf, /tf_static (좌표계)
  발행:  /cmd_vel (속도 명령), /tf_static (world→odom 정적 변환)

좌표계 (TF tree):
  world  ── (정적, spawn 위치 오프셋) ──→ odom
  odom   ── (Isaac Sim 발행) ────────────→ base_link
  → world→base_link 조회로 절대 좌표(트랙) 위치 획득

상태 머신:
  LANE      : Pure Pursuit 정상 추종
  WARN      : 장애물 접근 — 감속, 차선 유지
  AVOID     : 장애물 근접 — 감속 + 회피 조향 (Pure Pursuit 중단)
  STOP_OBS  : 장애물 긴급 정지 + 강한 회피 회전
  STOP_LINE : 정지선 감지 — 2초 정지 후 재출발
"""

import math
import time
import rclpy
from rclpy.node     import Node
from rclpy.qos      import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time     import Time as RclpyTime

import numpy as np

from sensor_msgs.msg    import LaserScan
from geometry_msgs.msg  import Twist, TransformStamped

import tf2_ros
from tf2_ros          import TransformException
from tf2_ros.buffer   import Buffer
from tf2_ros.transform_listener         import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


# ═══════════════════════════════════════════════════════════════════════
# ── 트랙 파라미터 (run_track_sim.py 와 동일) ────────────────────────────
# ═══════════════════════════════════════════════════════════════════════
BY       = -5.00
TY       =  5.00
RX       =  1.80
LX       = -1.80
R_CORNER =  0.44

# ═══════════════════════════════════════════════════════════════════════
# ── 로봇 spawn 위치 (run_track_sim.py 의 SetTranslate 와 동일하게 유지) ──
# ── world → odom 정적 변환 오프셋 ────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════
SPAWN_X = 0.2261
SPAWN_Y = -5.0052
SPAWN_Z = 0.0

# TF 프레임 이름
WORLD_FRAME = "world"
ODOM_FRAME  = "odom"
BASE_FRAME  = "base_link"

# ═══════════════════════════════════════════════════════════════════════
# ── 속도 파라미터 ────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════
MAX_SPEED   = 0.13   # 정상 최대 직진 속도 (m/s)
MIN_SPEED   = 0.04   # 장애물 감속 최소 속도
MAX_ANGULAR = 2.00   # 최대 회전 속도 (rad/s)
LOOKAHEAD   = 0.50   # Pure Pursuit 전방 주시 거리 (m)

# ═══════════════════════════════════════════════════════════════════════
# ── 장애물 탐지 파라미터 ─────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════
OBS_WARN   = 0.70    # 경고 거리: 감속 시작 (m)
OBS_AVOID  = 0.50    # 회피 거리: Pure Pursuit 중단, 회피 조향 시작
OBS_STOP   = 0.25    # 정지 거리: 완전 정지 + 강한 회피 회전

FRONT_DEG  = 35      # 전방 탐지 반각 (±도)
SIDE_DEG   = 65      # 측면 비교 반각 (좌/우 판별용)

AVOID_ANG_HARD = 1.60   # 긴급 정지 시 회피 회전 속도
AVOID_ANG_SOFT = 1.20   # 감속 회피 시 회피 회전 속도

# 회피 방향 전환 방지: 한번 결정된 회피 방향은 아래 거리까지 복구 전 유지
AVOID_LOCK_DIST = OBS_WARN + 0.15   # 이 거리까지 방향 잠금

# ═══════════════════════════════════════════════════════════════════════
# ── 정지선 파라미터 ──────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════
STOP_DURATION  = 2.0   # 정지 시간 (초)
STOP_COOLDOWN  = 12.0  # 동일 정지선 재감지 방지 쿨다운 (초)
STARTUP_DELAY  = 6.0   # 노드 시작 후 정지선 감지 비활성 시간 (초)

# 정지선 위치 (run_track_sim.py 트랙 기준, CY=0, CROSS_H=0.22, SL_OFF=0.10)
# (cx, cy, 감지_반경, 레이블)
STOP_LINES = [
    ( 1.30, BY,    0.18, 'StartFinish'),   # 출발/도착선 (Bottom 직선)
    ( 0.00,  0.32, 0.13, 'NorthStop'),     # 내부 교차로 북쪽 정지선
    ( 0.00, -0.32, 0.13, 'SouthStop'),     # 내부 교차로 남쪽 정지선
    ( 0.32,  0.00, 0.13, 'EastStop'),      # 내부 교차로 동쪽 정지선
    (-0.32,  0.00, 0.13, 'WestStop'),      # 내부 교차로 서쪽 정지선
]

# 웨이포인트 분해능
N_LINE = 35
N_ARC  = 28


# ═══════════════════════════════════════════════════════════════════════
# ── 트랙 웨이포인트 생성 ─────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════

def _arc(cx, cy, r, a0, a1, n=N_ARC):
    return [(cx + r*math.cos(math.radians(a0 + i*(a1-a0)/n)),
             cy + r*math.sin(math.radians(a0 + i*(a1-a0)/n)))
            for i in range(n + 1)]


def _line(x0, y0, x1, y1, n=N_LINE):
    return [(x0 + i*(x1-x0)/n, y0 + i*(y1-y0)/n) for i in range(n + 1)]


def generate_track_waypoints():
    R    = R_CORNER
    pvSE = (RX-R, BY+R);  pvNE = (RX-R, TY-R)
    pvNW = (LX+R, TY-R);  pvSW = (LX+R, BY+R)
    wps  = []
    wps += _line(pvSW[0], BY,    pvSE[0], BY)      # Bottom (→동)
    wps += _arc(*pvSE, R, 270, 360)                # SE 코너
    wps += _line(RX, pvSE[1],    RX, pvNE[1])      # Right  (→북)
    wps += _arc(*pvNE, R,   0,  90)                # NE 코너
    wps += _line(pvNE[0], TY,    pvNW[0], TY)      # Top    (→서)
    wps += _arc(*pvNW, R,  90, 180)                # NW 코너
    wps += _line(LX, pvNW[1],    LX, pvSW[1])      # Left   (→남)
    wps += _arc(*pvSW, R, 180, 270)                # SW 코너
    return wps


# ═══════════════════════════════════════════════════════════════════════
# ── Pure Pursuit ─────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))


def pure_pursuit(rx, ry, ryaw, waypoints, lookahead=LOOKAHEAD):
    n = len(waypoints)
    dists = [math.hypot(waypoints[i][0]-rx, waypoints[i][1]-ry)
             for i in range(n)]
    nearest = int(np.argmin(dists))

    target = nearest
    for step in range(n):
        idx = (nearest + step) % n
        if math.hypot(waypoints[idx][0]-rx, waypoints[idx][1]-ry) >= lookahead:
            target = idx
            break

    tx, ty = waypoints[target]
    alpha   = math.atan2(ty-ry, tx-rx) - ryaw
    alpha   = (alpha + math.pi) % (2*math.pi) - math.pi
    kappa   = 2.0 * math.sin(alpha) / max(lookahead, 0.01)
    return kappa, nearest, target


# ═══════════════════════════════════════════════════════════════════════
# ── ROS2 노드 ────────────────────────────────────────────────────────────
# ═══════════════════════════════════════════════════════════════════════

# 상태 상수
S_LANE      = 'LANE'
S_WARN      = 'WARN'
S_AVOID     = 'AVOID'
S_STOP_OBS  = 'STOP_OBS'
S_STOP_LINE = 'STOP_LINE'


class LaneFollowNav(Node):

    def __init__(self):
        super().__init__('lane_follow_nav')

        self.track_wps  = generate_track_waypoints()
        self.node_start = time.time()

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        ctrl_qos   = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── 토픽 ──────────────────────────────────────────────────
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self._cb_scan, sensor_qos)
        self.pub_cmd  = self.create_publisher(Twist, '/cmd_vel', ctrl_qos)

        # ── TF 설정 ───────────────────────────────────────────────
        self.tf_buffer    = Buffer()
        self.tf_listener  = TransformListener(self.tf_buffer, self)
        self.static_tf    = StaticTransformBroadcaster(self)
        self._publish_world_to_odom_tf()

        # ── 로봇 상태 ─────────────────────────────────────────────
        self.rx = self.ry = self.ryaw = 0.0
        self.tf_ok = False

        # ── 장애물 상태 ───────────────────────────────────────────
        self.obs_dist       = math.inf
        self.obs_side       = None     # 'left' | 'right'
        self.locked_side    = None     # 회피 방향 잠금 (방향 전환 방지)

        # ── 정지선 상태 ───────────────────────────────────────────
        self.state          = S_LANE
        self.stop_line_end  = 0.0            # 정지 종료 시각
        self.sl_last_time   = {}             # 각 정지선 마지막 감지 시각

        self.timer = self.create_timer(0.05, self._control_loop)  # 20 Hz

        self.get_logger().info('=' * 58)
        self.get_logger().info('  Pure Pursuit LaneFollow + 장애물 회피 + 정지선 감지')
        self.get_logger().info(f'  최대 속도  : {MAX_SPEED} m/s')
        self.get_logger().info(f'  경고 거리  : {OBS_WARN} m')
        self.get_logger().info(f'  회피 거리  : {OBS_AVOID} m')
        self.get_logger().info(f'  정지 거리  : {OBS_STOP} m')
        self.get_logger().info(f'  정지선 수  : {len(STOP_LINES)}개')
        self.get_logger().info('=' * 58)

    # ──────────────────────────────────────────────────────────────
    # TF: world → odom 정적 변환 발행 (한 번)
    # ──────────────────────────────────────────────────────────────

    def _publish_world_to_odom_tf(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = WORLD_FRAME
        t.child_frame_id  = ODOM_FRAME
        t.transform.translation.x = SPAWN_X
        t.transform.translation.y = SPAWN_Y
        t.transform.translation.z = SPAWN_Z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform(t)
        self.get_logger().info(
            f'정적 TF 발행: {WORLD_FRAME} → {ODOM_FRAME} '
            f'@ ({SPAWN_X:.4f}, {SPAWN_Y:.4f}, {SPAWN_Z:.4f})')

    def _update_pose_from_tf(self) -> bool:
        """world → base_link 변환 조회 → self.rx/ry/ryaw 갱신.

        Returns:
            True  : 변환 조회 성공
            False : 아직 TF 트리 미준비
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                WORLD_FRAME, BASE_FRAME,
                RclpyTime(),
                timeout=Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().info(
                f'TF 대기 중 ({WORLD_FRAME}→{BASE_FRAME}): {e}',
                throttle_duration_sec=2.0)
            return False

        self.rx   = trans.transform.translation.x
        self.ry   = trans.transform.translation.y
        q         = trans.transform.rotation
        self.ryaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.tf_ok = True
        return True

    # ──────────────────────────────────────────────────────────────
    # 콜백
    # ──────────────────────────────────────────────────────────────

    def _cb_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=float)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        valid  = (np.isfinite(ranges)
                  & (ranges > msg.range_min)
                  & (ranges < msg.range_max))
        ranges = np.where(valid, ranges, np.inf)

        fr = math.radians(FRONT_DEG)
        sr = math.radians(SIDE_DEG)

        # 전방 최근접 거리
        front = ranges[np.abs(angles) <= fr]
        self.obs_dist = float(front.min()) if front.size > 0 else math.inf

        # 좌/우 측면 비교 → 장애물 측면 판단
        left_r  = ranges[(angles >  fr) & (angles <=  sr)]
        right_r = ranges[(angles < -fr) & (angles >= -sr)]
        lm = float(left_r.min())  if left_r.size  > 0 else math.inf
        rm = float(right_r.min()) if right_r.size > 0 else math.inf

        if self.obs_dist < AVOID_LOCK_DIST:
            new_side = 'right' if rm < lm else 'left'
            if self.locked_side is None:
                # 회피 진입 시 방향 잠금
                self.locked_side = new_side
        else:
            # 장애물 충분히 멀어지면 잠금 해제
            self.locked_side = None
            self.obs_side    = 'right' if rm < lm else 'left'

        # 잠금 중에는 잠긴 방향 사용
        if self.locked_side is not None:
            self.obs_side = self.locked_side

    # ──────────────────────────────────────────────────────────────
    # 정지선 감지 (오도메트리 기반)
    # ──────────────────────────────────────────────────────────────

    def _check_stop_line(self) -> str | None:
        """현재 위치가 정지선 감지 반경 이내인지 확인.

        Returns:
            감지된 정지선 레이블 | None
        """
        if not self.tf_ok:
            return None
        now = time.time()
        # 노드 시작 직후 STARTUP_DELAY 동안 비활성
        if now - self.node_start < STARTUP_DELAY:
            return None

        for (cx, cy, radius, label) in STOP_LINES:
            dist = math.hypot(self.rx - cx, self.ry - cy)
            if dist < radius:
                last = self.sl_last_time.get(label, 0.0)
                if now - last > STOP_COOLDOWN:
                    self.sl_last_time[label] = now
                    return label
        return None

    # ──────────────────────────────────────────────────────────────
    # 제어 루프 (20 Hz)
    # ──────────────────────────────────────────────────────────────

    def _control_loop(self):
        cmd = Twist()

        # ── TF 조회로 로봇 절대 위치 갱신 ────────────────────────
        if not self._update_pose_from_tf():
            self.pub_cmd.publish(cmd)   # 정지 유지
            return

        # Pure Pursuit 곡률 계산 (항상 준비)
        kappa, nearest, target = pure_pursuit(
            self.rx, self.ry, self.ryaw, self.track_wps)
        lane_omega = float(np.clip(MAX_SPEED * kappa, -MAX_ANGULAR, MAX_ANGULAR))
        avd_side   = self.obs_side  # 현재 회피 방향

        # ══════════════════════════════════════════════════════════
        # STOP_LINE 상태 유지 (2초 정지)
        # ══════════════════════════════════════════════════════════
        if self.state == S_STOP_LINE:
            if time.time() < self.stop_line_end:
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                self._pub(cmd, S_STOP_LINE, nearest)
                return
            else:
                self.state = S_LANE
                self.get_logger().info('[STOP_LINE] 2초 정지 완료 → 재출발')

        # ══════════════════════════════════════════════════════════
        # STOP_OBS: 긴급 정지 (최우선)
        # ══════════════════════════════════════════════════════════
        if self.obs_dist <= OBS_STOP:
            self.state = S_STOP_OBS
            cmd.linear.x  = 0.0
            # 장애물 반대 방향으로 강하게 회전
            cmd.angular.z = (+AVOID_ANG_HARD if avd_side == 'right'
                             else -AVOID_ANG_HARD)
            self._pub(cmd, S_STOP_OBS, nearest)
            return

        # ══════════════════════════════════════════════════════════
        # AVOID: 감속 + 회피 조향 (Pure Pursuit 완전 차단)
        # ══════════════════════════════════════════════════════════
        if self.obs_dist <= OBS_AVOID:
            self.state = S_AVOID
            # 장애물 거리 비례 속도
            ratio        = ((self.obs_dist - OBS_STOP)
                            / (OBS_AVOID  - OBS_STOP))
            cmd.linear.x = float(np.clip(
                MIN_SPEED + (MAX_SPEED * 0.4 - MIN_SPEED) * ratio,
                MIN_SPEED, MAX_SPEED * 0.4))
            # 회피 각속도만 사용 — Pure Pursuit 완전 배제
            cmd.angular.z = (+AVOID_ANG_SOFT if avd_side == 'right'
                             else -AVOID_ANG_SOFT)
            self._pub(cmd, S_AVOID, nearest)
            return

        # ══════════════════════════════════════════════════════════
        # WARN: 감속 + Pure Pursuit 유지 + 약한 회피 예비 조향
        # ══════════════════════════════════════════════════════════
        if self.obs_dist <= OBS_WARN:
            self.state = S_WARN
            ratio        = ((self.obs_dist - OBS_AVOID)
                            / (OBS_WARN   - OBS_AVOID))
            # 속도: MAX*0.4 → MAX*0.75 (거리 비례)
            cmd.linear.x = float(np.clip(
                MAX_SPEED * 0.40 + (MAX_SPEED * 0.75 - MAX_SPEED * 0.40) * ratio,
                MIN_SPEED, MAX_SPEED))
            # Pure Pursuit + 약한 회피 혼합 (60:40)
            avoid_bias   = (+AVOID_ANG_SOFT * 0.40 if avd_side == 'right'
                            else -AVOID_ANG_SOFT * 0.40)
            cmd.angular.z = float(np.clip(
                lane_omega * 0.60 + avoid_bias, -MAX_ANGULAR, MAX_ANGULAR))
            self._pub(cmd, S_WARN, nearest)
            return

        # ══════════════════════════════════════════════════════════
        # 정지선 감지 확인 (장애물 없을 때만)
        # ══════════════════════════════════════════════════════════
        sl = self._check_stop_line()
        if sl is not None:
            self.state         = S_STOP_LINE
            self.stop_line_end = time.time() + STOP_DURATION
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info(f'[STOP_LINE] {sl} 감지 → 2초 정지')
            self._pub(cmd, S_STOP_LINE, nearest)
            return

        # ══════════════════════════════════════════════════════════
        # LANE: Pure Pursuit 정상 추종
        # ══════════════════════════════════════════════════════════
        self.state        = S_LANE
        cmd.linear.x      = MAX_SPEED
        cmd.angular.z     = lane_omega
        self._pub(cmd, S_LANE, nearest)

    def _pub(self, cmd: Twist, state: str, idx: int):
        self.pub_cmd.publish(cmd)
        wp = self.track_wps[idx]
        self.get_logger().info(
            f'[{state:9s}] '
            f'v={cmd.linear.x:+.3f} ω={cmd.angular.z:+.4f} | '
            f'({self.rx:.2f},{self.ry:.2f}) '
            f'→wp#{idx:03d}({wp[0]:.2f},{wp[1]:.2f}) | '
            f'obs={self.obs_dist:.2f}m({self.obs_side or "-"})',
            throttle_duration_sec=0.3)


# ═══════════════════════════════════════════════════════════════════════
def main():
    rclpy.init()
    node = LaneFollowNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료')
    finally:
        node.pub_cmd.publish(Twist())   # 정지
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
