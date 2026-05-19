import sys
sys.path.append('/opt/ros/jazzy/lib/python3.12/site-packages')
from dynamixel_sdk import *

port = PortHandler('/dev/ttyACM0')
packet = PacketHandler(2.0)

port.openPort()
port.setBaudRate(57600)

print("=== Dynamixel 스캔 (ID 1~5) ===")
for dxl_id in range(1, 6):
    model_number, result, err = packet.ping(port, dxl_id)
    if result == COMM_SUCCESS:
        print(f"  ID {dxl_id}: 발견! 모델넘버={model_number}")
    else:
        print(f"  ID {dxl_id}: 없음")

port.closePort()
