# newlicycle
The upper-level computer of the Licycle project
![image](https://user-images.githubusercontent.com/35949664/172154351-11d1be62-61a1-4635-ae05-d02ef72afad8.png)

#### Introduction
A part of Licycle project, the code of the upper-level computer.

#### Function
The upper computer communicates with the lower microcontroller through the Uart serial port, and then drives the hub and steer motor indirectly. And leg motors communicate via a controller area network (CAN) and use an adapter to communicate with the upper computer to implement a frequency of 500 Hz for leg actuation. Two encoders are used to measure bikebot steering angle and the wheel velocity. An inertial measurement unit (IMU) is used for obtaining bikebot attitude and acceleration.
![image](https://user-images.githubusercontent.com/35949664/172154286-0e388910-1670-4d61-952a-a005cd253dfd.png)

#### Instruction

1.  hardware:
    	NVIDIA Jetson TX2

2.  GPIO define：
    Serial 1     	PA9（TX）	PA10（RX）
    Serial 2 		PA2（TX）	PA3（RX）
    Serial 3		PB10（TX）PB11（RX）
    CAN1（SCA）	  PA11（RX）PA12（TX）
