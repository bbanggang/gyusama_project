"""USD 구조 확인 + defaultPrim 수정 스크립트"""
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

launcher = AppLauncher(args_cli)
app = launcher.app

from pxr import Usd, Sdf

USD_PATH = "/home/linux/gyusama-project/isaac_sim/assets/turtlebot3_burger.usd"
stage = Usd.Stage.Open(USD_PATH)

# ── 전체 Prim 구조 출력 ───────────────────────────────────────────────────────
print("\n=== Prim 구조 ===")
for prim in stage.Traverse():
    depth = len(prim.GetPath().pathString.split("/")) - 2
    print("  " * depth + str(prim.GetPath()) + " [" + prim.GetTypeName() + "]")

# ── defaultPrim 자동 설정 (최상위 Xform prim) ─────────────────────────────────
root_children = [p for p in stage.GetPseudoRoot().GetChildren()]
print(f"\n=== 루트 자식 Prim: {[p.GetName() for p in root_children]} ===")

if root_children:
    default_prim = root_children[0]
    stage.SetDefaultPrim(default_prim)
    stage.Save()
    print(f"[SUCCESS] defaultPrim 설정: {default_prim.GetPath()}")

app.close()
