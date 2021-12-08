import can
import os
import time

print('\n\rCAN Tx test')
print('Bring up CAN0....')
os.system("sudo ip link set can0 up type can bitrate 1000000")
time.sleep(0.1)

try:
    bus0 = can.Bus(interface='socketcan',channel='can0')
except OSError:
    print('Cannot find Can Device.')
    exit()

# send a message
message = can.Message(arbitration_id=123, is_extended_id=False,
                      data=[0x11, 0x22, 0x33])
bus0.send(message, timeout=0.2)

# over
os.system("sudo ip link set can0 down")





