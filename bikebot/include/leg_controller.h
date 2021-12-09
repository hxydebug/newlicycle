#ifndef __LEG_CONTROL_H
#define __LEG_CONTROL_H

#include "opmath.h"
#include "bikebot_control.h"

#define l_leg 0
#define r_leg 1

#define trytest 2

//单位米
#define Len0 0.0755f
#define Len1 0.21f
#define Len2 0.21f

#define wid 0.1f
#define dL 0.15f
#define hG 0.365f

#define downstepnum 300
#define upstepnum 300
#define l_force_period 60
#define r_force_period 50
#define stepnum 300

#define INITIAL 0
#define SWING_DOWN 1
#define SWING_UP 2
#define STANCE 3
#define TWO_LEGS 4
#define SWING 5

typedef int Leg_StateName;

typedef int Leg;

typedef struct{
	float q[3];
} Angle;

typedef struct{
	float error[6];
} Posdiff;

typedef struct{
	float x;
	float y;
	float z;
} Position;

typedef struct{
	float q[6];
} Inform;

class swing_control{

public:
	swing_control();
	void get_cmd(Angle *angle,Leg leg);
	void check_Transition(void);
	void get_start(Eigen::VectorXd angle, Position posi, Leg leg);
private:
	int count;
};

class swingdown_control{

public:
	swingdown_control();
	int get_cmd(Angle *angle,Leg leg,float torque,float vangle);
	void check_Transition(void);
	void get_start(Eigen::VectorXd angle, Position posi, Leg leg);
private:
	int count;
	float angle_a;
	float last_vangle;
};

class swingup_control{

public:
	swingup_control();
	int get_cmd(Angle *angle,Leg leg);
	void check_Transition(void);
	void get_start(Eigen::VectorXd angle, Position posi, Leg leg);
private:
	int count;
};

class stance_control{

public:
	stance_control();
	Eigen::VectorXd get_cmd(Leg leg,float roll,Eigen::VectorXd angles);
	void check_Transition(void);
	void get_start(Leg leg,float roll,float dvarphi);
	double get_force(void);
private:
	int count;
	Eigen::VectorXd F;
};

class leg_controller{

public:
	leg_controller(Leg_state *robot);
	int get_action(Leg_command *cmd,int impact_happen);
	void goto_xyz(float xx,float yy,float zz,Leg direction);
	void get_inform(double i0,double i1,double i2,double i3,double i4,double i5);
	void get_Tau(double Tau);
	int get_stance_ifend(void);
	int get_stance_ifbegin(void);
	int get_swdown_ifbegin(void);
	double get_global_force(void);
	void set_height(double height);
	double get_F(void);
	Posdiff get_error(void);

private:
	Leg_state *leg;

	Angle angle[2];
	Angle angleV[2];
	Position position[2];
	Position velocity[2];
	Posdiff poserror;
	

	int stance_end;
	int stance_begin;
	int leg_down;

	std::vector<float> x_position;
	std::vector<float> z_position;

	swing_control swing_contr;
	swingdown_control swdown_contr;
	swingup_control swup_contr;
	stance_control stance_contr;
};

//leg model
void set_xyz(Leg leg,Angle *angle,float xx,float yy,float zz);
void Kinematics(Angle *angle,Position *position,Leg leg);
void Inv_kinematics(Angle *angle,Position *position,Leg leg);
void Kinematics_global(Angle *angle,Position *position,Leg leg);
void Inv_kinematics_global(Angle *angle,Position *position,Leg leg);
void init_chabu(Position *pdes,Position *vdes,Position *pini,Position *vini,float step,Leg leg);
void chabu(Position *pos,float step,Leg leg);
void init_linear(Position *pdes,Position *pini,float tf,Leg leg);
void linear(Position *pos,float step,Leg leg);
float linear(float ini,float des,float tf,float step);
Eigen::MatrixXd calcu_JacoV(Eigen::Vector3d angle,Leg leg);
Eigen::MatrixXd calcu_Jaco(Angle angle,Leg leg);
Position pos_control(Leg leg, double roll);


#endif