#ifndef __BICYCLE_CONTROL_H
#define __BICYCLE_CONTROL_H

#include "opmath.h"
#include "bikebot_control.h"

typedef struct{
/*自行车常量*/
	double mb, hb, Jb, R;//质量、重心高度、转动惯量、车轮半径
	double g;
	double l, lb, lt;//前后轮距、重心至后轮距离、前轮尾迹
	double psi_offset;//偏航角的偏差 (rad)
	double pitch_offset;//俯仰角的偏差 (rad)
	double angle_G;//自行车重心的偏移角度 (rad)
	double epsilon;
	double Jt;	
/*位姿*/
	double x, y;
	double dx, dy;
	double xe, ye;
	double vr;		//前轮速度(m/s)
	double last_vr;
	double varphi, dvarphi;	//倾斜角、倾斜角速度
	double psi, dpsi;//偏航角、偏航角速度
	double phi, dphi;//转向角、转向角速度	
/*计算过程*/
	double pitch;
	double varphie;	
	double dvarphie;
	double dpitch;
	double Tau_s;
	double Tau_g;
/*控制参数*/
	double b2, b1, b0;
	double k, a1, a0;	
/*控制输出*/
	double new_phi;	//算出到转向角
	double last_target, target;//上一次目标转向角、当前目标转向角
/*加速度*/
	double accx,accy,accz;
/*运行时间*/
	double timer;
}Balance_data;

class bicycle_controller{

public:
	bicycle_controller(Bike_state *robot,char *ch);
	void initial_param();
	int get_action(Bike_command *cmd,int eic_able);
	double get_timer();
	Input get_input();
	void balanceCalc(int flag);
	void balance();
	void stateUpdate();
	void reset();
	double get_height();
private:
	Bike_state *bike;
	Balance_data bikebot;
	float phi_cmd;
};

double solution(double vr, double dpsi, double u_psi);
double rad2deg(double rad);
#endif