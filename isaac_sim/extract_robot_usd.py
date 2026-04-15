"""
ros2-turtlebot.usd에서 turtlebot3_burger(로봇+ROS2 OmniGraph)만 추출해
ros2-turtlebot-robot-only.usd로 저장하는 일회성 스크립트

실행: /tmp/run_inspect.sh 와 동일한 환경 변수 설정 후
    ~/isaac_env/bin/python ~/gyusama-project/isaac_sim/extract_robot_usd.py
"""
import os
import sys

sys.argv += ["--/app/extensions/excluded/0=omni.graph.image.core"]

from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": True,
    "experience": "/home/linux/isaac_env/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.full.kit",
})

import omni.usd
from pxr import Sdf

ASSETS_DIR = os.path.join(os.path.dirname(__file__), "assets")
SRC_PATH = os.path.join(ASSETS_DIR, "ros2-turtlebot.usd")
DST_PATH = os.path.join(ASSETS_DIR, "ros2-turtlebot-robot-only.usd")

print(f"[INFO] USD 로드 중: {SRC_PATH}")
omni.usd.get_context().open_stage(SRC_PATH)
for _ in range(10):
    simulation_app.update()

stage = omni.usd.get_context().get_stage()

# simple_room 제거 (원본 stage에서만 — 파일은 건드리지 않음)
ROOM_PATH = "/World/turtlebot_tutorial_multi_sensor_publish_rates/simple_room"
if stage.GetPrimAtPath(ROOM_PATH).IsValid():
    stage.RemovePrim(ROOM_PATH)
    print(f"[INFO] stage에서 제거: {ROOM_PATH}")

# 남은 prim 확인
print("\n[INFO] 저장될 Prim (depth<=2):")
for prim in stage.Traverse():
    path = str(prim.GetPath())
    depth = path.count("/") - 1
    if depth <= 2:
        print("  " * depth + path + "  [" + prim.GetTypeName() + "]")

# 원본 파일은 그대로, 새 파일로만 내보내기
stage.GetRootLayer().Export(DST_PATH)
print(f"\n[SUCCESS] 로봇 전용 USD 저장: {DST_PATH}")

simulation_app.close()
