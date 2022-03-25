#ifndef __LEG_CONTROL_H
#define __LEG_CONTROL_H

#include "opmath.h"
#include "bikebot_control.h"
#include "swing_leg_controller.h"
#include "stance_leg_controller.h"
#include "control.h"

#define trytest 2

//单位米
#define wide 0.1f
#define dL 0.15f

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

typedef struct{
	float error[6];
} Posdiff;

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
	leg_controller(Leg_state *robot,gait_generator *gait_gen,swing_leg_controller *swc, stance_leg_controller *stc);
	void get_action(Leg_command *cmd,int Run_mode);
	int get_action1(Leg_command *cmd,int impact_happen);
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
	void set_PDGain();
	Eigen::VectorXd tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT);

private:
	Leg_state *leg;

	Angle angle[2];
	Angle angleV[2];
	Position position[2];
	Position velocity[2];
	Posdiff poserror;
	Eigen::VectorXd pGain,dGain;
	Eigen::VectorXd posT,angT;

	int stance_end;
	int stance_begin;
	int leg_down;

	std::vector<float> x_position;
	std::vector<float> z_position;

	swing_control swing_contr;
	swingdown_control swdown_contr;
	swingup_control swup_contr;
	stance_control stance_contr;

	gait_generator *gait_generate;
 	swing_leg_controller *swctr;
	stance_leg_controller *stctr;
};

//leg model
void set_xyz(Leg leg,Angle *angle,float xx,float yy,float zz);
void init_chabu(Position *pdes,Position *vdes,Position *pini,Position *vini,float step,Leg leg);
void chabu(Position *pos,float step,Leg leg);
void init_linear(Position *pdes,Position *pini,float tf,Leg leg);
void linear(Position *pos,float step,Leg leg);
float linear(float ini,float des,float tf,float step);
Position pos_control(Leg leg, double roll);


#endif