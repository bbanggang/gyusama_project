"""
TurtleBot3 Burger URDF → USD 변환 스크립트

실행 방법:
    source ~/isaac_env/bin/activate
    cd ~/IsaacLab
    ./isaaclab.sh -p ~/gyusama-project/isaac_sim/convert_urdf_to_usd.py --headless

결과:
    ~/gyusama-project/isaac_sim/assets/turtlebot3_burger.usd
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="TurtleBot3 URDF → USD 변환")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ── Isaac Sim 완전 초기화 후 임포트 ─────────────────────────────────────────
import os
import omni.kit.app
import omni.kit.commands

# ── URDF importer 확장 명시적 활성화 ────────────────────────────────────────
manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)

# 확장 로드 후 임포트
from isaacsim.asset.importer.urdf import _urdf

# ── 경로 설정 ─────────────────────────────────────────────────────────────────
URDF_PATH = "/opt/ros/jazzy/share/turtlebot3_description/urdf/turtlebot3_burger.urdf"
OUTPUT_DIR = os.path.expanduser("~/gyusama-project/isaac_sim/assets")
OUTPUT_USD = os.path.join(OUTPUT_DIR, "turtlebot3_burger.usd")

os.makedirs(OUTPUT_DIR, exist_ok=True)

print("\n" + "=" * 60)
print("TurtleBot3 URDF → USD 변환 시작")
print(f"  입력: {URDF_PATH}")
print(f"  출력: {OUTPUT_USD}")
print("=" * 60)

# ── URDF Importer 설정 ────────────────────────────────────────────────────────
status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.self_collision = False
import_config.fix_base = False
import_config.distance_scale = 1.0
import_config.density = 0.0
import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY
import_config.default_drive_strength = 1e7
import_config.default_position_drive_damping = 1e5

# ── 변환 실행 ─────────────────────────────────────────────────────────────────
print("\n[INFO] 변환 중...")
status, stage_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF_PATH,
    import_config=import_config,
    dest_path=OUTPUT_USD,
)

if status:
    print(f"\n[SUCCESS] USD 저장 완료: {OUTPUT_USD}")
else:
    print("\n[ERROR] 변환 실패")

simulation_app.close()
