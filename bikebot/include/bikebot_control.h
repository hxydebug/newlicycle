#ifndef __BIKEBOT_CONTROL_H
#define __BIKEBOT_CONTROL_H

#include "motor_drive.h"
#include <fstream>
#include "licycle_config.h"
#include <vector>

typedef struct{
    float phi;
    float dphi;
    float velocity;
}  Bike_cb;

//定义leg_state结构体
typedef struct{
    CBData cbdata[6];
    float varphi;
    float dvarphi;
    float accx;
    float optmT;
}  Leg_state;
//定义leg_cmd结构体
typedef struct{
    float torque[6];
}  Leg_command;
//定义bike_state结构体
typedef struct{
    Bike_cb bike_msg;
    float varphi;
    float dvarphi;
    float psi;
    float pitch;
    float dpitch;
    float dpsi;
    float accx;
    float accy;
    float accz;
}  Bike_state;
//定义bike_cmd结构体
typedef struct{
    float phi;
    uint16_t speed;
}  Bike_command;

typedef struct{
	double varphi;
	double psi;
	double dvarphi;
	double dpsi;
	double T0;
    double y;
    double dx;
    double dy;
    double phi;
} Input;

void thread_setup(void);
void control_threadcreate(void);

void can0_tx(uint8_t tdata[],uint8_t id);
void can1_tx(uint8_t tdata[],uint8_t id);
void legstate_update();
void bikestate_update();
void motor_control(Leg_command leg_cmd);
void CAN_init();
void Sleep_us(int us);
void reset_motors();
void setup_motors();
void drive_bike(Bike_command msg);
void setpoint(float x,float y,float z);
void setpoint1(float x,float y,float z);

#endif