"""
ros2-turtlebot.usd 구조 검사 스크립트
실행: ~/isaac_env/bin/python ~/gyusama-project/isaac_sim/inspect_usd.py
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

USD_PATH = os.path.join(os.path.dirname(__file__), "assets", "ros2-turtlebot.usd")
omni.usd.get_context().open_stage(USD_PATH)

for _ in range(10):
    simulation_app.update()

stage = omni.usd.get_context().get_stage()

print("\n" + "=" * 60)
print("USD 전체 Prim 구조 (depth <= 3)")
print("=" * 60)
for prim in stage.Traverse():
    path = str(prim.GetPath())
    depth = path.count("/") - 1
    if depth <= 3:
        indent = "  " * depth
        print(f"{indent}{path}  [{prim.GetTypeName()}]")

simulation_app.close()
