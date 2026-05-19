"""
U2D2로 Dynamixel 모터 스캔 및 baudrate 변경
사용법: python3 scripts/scan_dxl_u2d2.py [포트] [--fix]
  --fix: 발견된 모터의 baudrate를 1,000,000으로 변경 (ROS2 표준)
"""
import sys
import argparse

sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')
try:
    from dynamixel_sdk import *
except ImportError:
    print("[ERROR] dynamixel-sdk 없음: pip install dynamixel-sdk")
    sys.exit(1)

BAUDRATES = [57600, 115200, 1_000_000, 2_000_000, 3_000_000, 4_000_000]
TARGET_BAUDRATE = 1_000_000   # ROS2 TurtleBot3 표준

# Dynamixel XL430-W250 baudrate 레지스터 주소
ADDR_BAUDRATE   = 8    # EEPROM
BAUD_MAP = {
    0: 9_600,
    1: 57_600,
    2: 115_200,
    3: 1_000_000,
    4: 2_000_000,
    5: 3_000_000,
    6: 4_000_000,
    7: 4_500_000,
}
BAUD_MAP_INV = {v: k for k, v in BAUD_MAP.items()}

ADDR_TORQUE_ENABLE = 64


def scan(port_name: str, fix: bool):
    found = []

    for baud in BAUDRATES:
        port   = PortHandler(port_name)
        packet = PacketHandler(2.0)

        if not port.openPort():
            print(f"[ERROR] 포트 열기 실패: {port_name}")
            sys.exit(1)
        port.setBaudRate(baud)

        print(f"\n[SCAN] baudrate={baud:>9,} ...", end=" ", flush=True)
        ids_at_baud = []
        for dxl_id in range(1, 10):
            model, result, _ = packet.ping(port, dxl_id)
            if result == COMM_SUCCESS:
                ids_at_baud.append((dxl_id, model))

        if ids_at_baud:
            print(f"발견! {ids_at_baud}")
            found.append((baud, ids_at_baud))
        else:
            print("없음")

        port.closePort()

    print("\n" + "=" * 50)
    if not found:
        print("[결과] 모터를 찾지 못했습니다.")
        print("  → 케이블 연결 및 전원을 확인하세요.")
        return

    print(f"[결과] 발견된 모터:")
    for baud, motors in found:
        for mid, model in motors:
            print(f"  ID={mid}  모델={model}  현재 baudrate={baud:,}")

    if not fix:
        print(f"\n  baudrate를 {TARGET_BAUDRATE:,}으로 변경하려면 --fix 옵션을 추가하세요.")
        return

    # baudrate 변경
    print(f"\n[FIX] baudrate를 {TARGET_BAUDRATE:,}으로 변경합니다...")
    for baud, motors in found:
        if baud == TARGET_BAUDRATE:
            print(f"  이미 {TARGET_BAUDRATE:,} — 변경 불필요")
            continue

        port   = PortHandler(port_name)
        packet = PacketHandler(2.0)
        port.openPort()
        port.setBaudRate(baud)

        for mid, _ in motors:
            # 토크 비활성화 (EEPROM 쓰기 전 필수)
            packet.write1ByteTxRx(port, mid, ADDR_TORQUE_ENABLE, 0)
            # baudrate 변경
            new_val = BAUD_MAP_INV[TARGET_BAUDRATE]
            result, err = packet.write1ByteTxRx(port, mid, ADDR_BAUDRATE, new_val)
            if result == COMM_SUCCESS:
                print(f"  ID={mid} baudrate 변경 완료 ({baud:,} → {TARGET_BAUDRATE:,})")
            else:
                print(f"  ID={mid} 변경 실패 (result={result})")

        port.closePort()

    print("\n[완료] 변경 후 OpenCR에 재연결하고 control-node를 재시작하세요.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    parser.add_argument("--fix", action="store_true", help="baudrate를 1,000,000으로 변경")
    args = parser.parse_args()
    scan(args.port, args.fix)
