#!/usr/bin/python3
#coding=utf-8

import threading
import can
import time
import os
# import rospy

class Can_recvThread (threading.Thread):
    def __init__(self,bus_name):
        threading.Thread.__init__(self)
        self.bus_name = bus_name

    def run(self):
        try:
            while True:
                message = self.bus_name.recv()    # Wait until a message is received.
                
                c = '{0:f} {1:x} {2:x} '.format(message.timestamp, message.arbitration_id, message.dlc)
                s=''
                for i in range(message.dlc ):
                    s +=  '{0:x} '.format(message.data[i])
                    
                # print(' {}'.format(c+s))
                 
        except KeyboardInterrupt:
            #Catch keyboard interrupt
            can_stop("can0")
            can_stop("can1")
            pass

def can_setup(can_name):
    can_setup_command = "sudo ip link set " + can_name +" up type can bitrate 1000000"
    os.system(can_setup_command)

def can_stop(can_name):
    can_stop_command = "sudo ip link set " + can_name + " down"
    os.system(can_stop_command)

if __name__ == "__main__":
    
    print('Bring up CAN0....')
    can_setup("can0")
    time.sleep(0.1)
    try:
        bus0 = can.Bus(interface='socketcan',channel='can0')
    except OSError:
        print('Cannot find CAN DEVICE.')
        exit()

    print('Bring up CAN1....')
    can_setup("can1")
    time.sleep(0.1)
    try:
        bus1 = can.Bus(interface='socketcan',channel='can1')
    except OSError:
        print('Cannot find CAN DEVICE.')
        exit()
    
    print('Ready')

    # 创建新线程
    can0_recThread = Can_recvThread(bus0)
    can1_recThread = Can_recvThread(bus1)

    # 开启新线程
    can0_recThread.start()
    can1_recThread.start()

    # 主程序
    # rospy.init_node('controller')
    # rate = rospy.Rate(1000)
    try:
        while True:
            begin = time.time()
            # print(time.time())
            # send a message
            message = can.Message(arbitration_id=123, is_extended_id=False,
                                data=[0x01, 0x22, 0x33, 0x44, 0x11, 0x22, 0x33, 0x44])
            bus0.send(message, timeout=0.001)
            message = can.Message(arbitration_id=123, is_extended_id=False,
                                data=[0x02, 0x22, 0x33, 0x44, 0x11, 0x22, 0x33, 0x44])
            bus0.send(message, timeout=0.001)
            message = can.Message(arbitration_id=123, is_extended_id=False,
                                data=[0x03, 0x22, 0x33, 0x44, 0x11, 0x22, 0x33, 0x44])
            bus0.send(message, timeout=0.001)
            message = can.Message(arbitration_id=123, is_extended_id=False,
                                data=[0x09, 0x22, 0x33, 0x44, 0x11, 0x22, 0x33, 0x44])
            bus1.send(message, timeout=0.001)
            message = can.Message(arbitration_id=123, is_extended_id=False,
                                data=[0x0a, 0x22, 0x33, 0x44, 0x11, 0x22, 0x33, 0x44])
            bus1.send(message, timeout=0.001)
            message = can.Message(arbitration_id=123, is_extended_id=False,
                                data=[0x0b, 0x22, 0x33, 0x44, 0x11, 0x22, 0x33, 0x44])
            bus1.send(message, timeout=0.001)
            # bus0.send(message, timeout=0.001)
            # bus0.send(message, timeout=0.001)
            # bus0.send(message, timeout=0.001)
            

            # rate.sleep()

            while time.time()-begin <= 0.001:
                pass
            print(time.time()-begin)


                
    except KeyboardInterrupt:
        #Catch keyboard interrupt
        can_stop("can0")
        can_stop("can1")
        print('\n\rKeyboard interrtupt')