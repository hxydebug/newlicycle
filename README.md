# This Licycle project is about a bicycle-based robot which can cross obstacles without falling over
You can see the performance in the following video.

[![Watch the video](https://user-images.githubusercontent.com/35949664/177961370-0e0d035f-c752-49f6-a6b2-364e063c4b27.png)](https://youtu.be/kf8eAVD_ZEQ)
#### Introduction
A part of Licycle project, the code of the upper-level computer.

#### Function
The upper computer communicates with the lower microcontroller through the Uart serial port, and then drives the hub and steer motor indirectly. And leg motors communicate via a controller area network (CAN) and use an adapter to communicate with the upper computer to implement a frequency of 1000 Hz for leg actuation. An inertial measurement unit (IMU) is used for obtaining bikebot attitude and acceleration.
![image](https://user-images.githubusercontent.com/35949664/172154286-0e388910-1670-4d61-952a-a005cd253dfd.png)

#### Instruction

1.  hardware:
    	NVIDIA Jetson TX2

2.  model：
![image](https://user-images.githubusercontent.com/35949664/172154935-ebfeba36-db93-4fc2-a223-e6a2d5ee66a9.png)

