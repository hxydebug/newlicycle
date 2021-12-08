#!/usr/bin/python3
#coding=utf-8

import os
import can

can.rc['interface'] = 'socketcan_native'   #```  Linux 里面的 socketcan 在am335x 上面应该使用这个 ```
can.rc['channel'] = 'can1'                 # ''' can 接口名字  '''
can.rc['bitrate'] = 1000000               # ''' 波特率       1M '''
# from can.interfaces.interface import Bus
#'''  这个函数是can 的启动函数 '''
def can_setup(can_name):
    can_setup_command = "sudo ip link set " + can_name +" type can bitrate 1000000"
    can_start_command = "sudo ip link set up " + can_name
    pass
    os.system(can_setup_command)
    os.system(can_start_command)
#'''  can 停止函数  '''
def can_stop(can_name):
    can_stop_command = "canconfig " + can_name + " stop"
    os.system(can_stop_command)
#'''   发送信息 '''
def send_one():
    bus = can.interface.Bus()
    msg = can.Message(arbitration_id=0x7f,
            data=[11, 25, 11, 1, 1, 2, 23, 18], 
            extended_id=False)
    try:
        bus.send(msg);   #''' 发送信息 '''
        print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")
    
def recv():
    bus = can.interface.Bus()
    msg = bus.recv(100);   #''' 接收信息 '''
    try:
        bus.send(msg)
        bus.shutdown()
        print(msg)
        print(msg.data[0]);              #  接收回来的第一个字节的数据
        print(msg.arbitration_id);    # 接收回来的ID
        return msg
    except can.CanError:
        print("Message NOT sent")

if __name__ == "__main__":
    #'''   can_setup("can1"); '''
    # can_setup("can1")
    # send_one()
    while(1):
        recv()
    #''' can_stop("can1"); '''