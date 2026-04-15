"""
TurtleBot3 제어 시나리오 스크립트

Isaac Sim이 실행 중인 상태에서 별도 터미널로 실행합니다.
/cmd_vel 토픽으로 전진/회전/정지 시나리오를 수행합니다.

실행:
    source /opt/ros/jazzy/setup.bash
    python3 ~/gyusama-project/isaac_sim/control_scenario.py
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

SCENARIOS = [
    ("전진",    10.0,  0.2,  0.0),
    ("우회전",   5.0,  0.0, -0.5),
    ("전진",     8.0,  0.2,  0.0),
    ("좌회전",   5.0,  0.0,  0.5),
    ("전진",     8.0,  0.2,  0.0),
    ("정지",     3.0,  0.0,  0.0),
]

def main():
    rclpy.init()
    node = rclpy.create_node("turtlebot3_scenario")
    pub  = node.create_publisher(Twist, "/cmd_vel", 10)

    print("=" * 60)
    print(" TurtleBot3 제어 시나리오 시작")
    print(" Isaac Sim의 /cmd_vel 토픽으로 명령 전송")
    print("=" * 60)

    for name, duration, lx, az in SCENARIOS:
        print(f"\n[→] {name}  (linear.x={lx:.1f}, angular.z={az:.1f}, {duration:.0f}초)")
        msg = Twist()
        msg.linear.x  = lx
        msg.angular.z = az
        end = time.time() + duration
        while time.time() < end:
            pub.publish(msg)
            time.sleep(0.05)

    stop = Twist()
    pub.publish(stop)
    print("\n[INFO] 시나리오 완료")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
