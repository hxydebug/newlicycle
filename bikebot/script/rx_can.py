import can
import time
import os


print('\n\rCAN Rx test')
print('Bring up CAN0....')
os.system("sudo ip link set can0 up type can bitrate 1000000")
time.sleep(0.1) 

try:
    bus0 = can.Bus(interface='socketcan',channel='can0')
except OSError:
    print('Cannot find CAN DEVICE.')
    exit()
    
print('Ready')

try:
    while True:
        message = bus0.recv()    # Wait until a message is received.
        
        c = '{0:f} {1:x} {2:x} '.format(message.timestamp, message.arbitration_id, message.dlc)
        s=''
        for i in range(message.dlc ):
            s +=  '{0:x} '.format(message.data[i])
            
        print(' {}'.format(c+s))
    
    
except KeyboardInterrupt:
    #Catch keyboard interrupt
    os.system("sudo ip link set can0 down")
    print('\n\rKeyboard interrtupt')