#include "leg_controller.h"
using namespace std;

float a0[2][3] = {0};
float a1[2][3] = {0};
float a2[2][3] = {0};
float a3[2][3] = {0};

float kk[2][3] = {0};
float bb[2][3] = {0};

double rollMax = 2.0*PI/180;

float kp = 8;
float kd = 0.2;
float vel_ref = 0;

Leg_StateName last_state[2];
Leg_StateName current_state[2];
Leg_StateName pre_state[2];

Inform information;
float global_force;
float global_Tau;
int if_getTau = 0;
double collision_height;

int impact_happened = 0;
int timeflag = 0;
int timecount = 0;
int start_count = 0;

Position init_pos[2];
Angle init_angle[2];

leg_controller::leg_controller(Leg_state *robot){
	leg = robot;

	//set the initial position
	set_xyz(l_leg,&init_angle[0],0.16,0.08,-0.12);
  	set_xyz(r_leg,&init_angle[1],0.16,-0.08,-0.12);

	Kinematics(&init_angle[0],&init_pos[0],0);
    Kinematics(&init_angle[1],&init_pos[1],1);

	current_state[l_leg] = INITIAL;
    current_state[r_leg] = INITIAL;

	this->stance_end = 0;
	this->stance_begin = 0;
}
void leg_controller::get_inform(double i0,double i1,double i2,double i3,double i4,double i5){
	information.q[0] = i0;
	information.q[1] = i1;
	information.q[2] = i2;
	information.q[3] = i3;
	information.q[4] = i4;
	information.q[5] = i5;
}
void leg_controller::get_Tau(double Tau){
	if_getTau = 1;
	global_Tau = fabs(Tau);
}

Posdiff leg_controller::get_error(void){
	return poserror;
}


int leg_controller::get_action(Leg_command *cmd,int impact_happen){

	int impulse_able = 0;
	this->stance_end = 0;
	this->stance_begin = 0;
	
	// static int stance_flag[2] = {1,1};
	static int swdown_flag[2] = {1,1};
	// update state
	Eigen::VectorXd angles(6);
	for(int i(0);i<6;i++){
		angles[i] = leg->cbdata[i].p;
	}
	// float roll = leg->varphi-1.0 * PI / 180.0;
	float roll = leg->varphi;
	float dvarphi = leg->dvarphi;
	global_Tau = leg->optmT;
	// double roll = rollMax;
	float torque2 = leg->cbdata[2].t*30/19;
	float torque5 = leg->cbdata[5].t*30/19;
	float w1 = leg->cbdata[1].v;
	float w4 = leg->cbdata[4].v;

	// if(leg->accx<-1 && timeflag == 0){
	// 	impact_happened = 1;
	// 	timeflag = 1;
	// 	timecount = 0;
	// 	start_count = 1;
	// 	cout<<"start timer!"<<endl;
	// }
	// if(start_count==1){
	// 	timecount ++;
	// 	if(timecount>700){//370
	// 		timecount=0;
	// 		impact_happened=0;
	// 		start_count=0;
	// 		this->stance_end = 1;
	// 		timeflag = 0;
	// 	}
	// }

	if(leg->accx<-0.5 && timeflag == 0){
		impact_happened = 1;
		timeflag = 1;
		timecount = 0;
		start_count=1;
	}

	if(start_count==1){
		timecount++;
		// timeflag=1;
		if(timecount>1200){
			timecount=0;
			timeflag=0;
			start_count=0;
		}
	}
	// infer the leg to run
	Leg leg_dir;
	if(impact_happened==1){

		// cout<<"oooooo"<<endl;
		if((dvarphi < -8.0*PI/180.0 && roll-1.0 * PI / 180.0 < -1.2*PI/180.0) || roll-1.0 * PI / 180.0 < -1.8*PI/180.0){
			if(current_state[1]==INITIAL){
				leg_dir = l_leg;
				if(swdown_flag[leg_dir]==1){
					Position pos = pos_control(leg_dir,roll);
					cout<<pos.x <<pos.y <<pos.z <<endl;
					cout<<roll*180/PI<<", "<<dvarphi*180/PI<<endl;

					//verify correct
					Angle an;
					Inv_kinematics(&an,&pos,0);
					cout<<an.q[0]<<an.q[1]<<an.q[2]<<endl;
					
					pre_state[leg_dir] = SWING_DOWN;
					swdown_contr.get_start(angles,pos,leg_dir);
					swdown_flag[leg_dir] = 0;
					// swdown_flag[r_leg] = 0;
					impact_happened=0;
					// timeflag = 0;
				}
			}
		}
		else if(roll-1.0 * PI / 180.0 > rollMax && current_state[0]==INITIAL && dvarphi > 1.0*PI/180.0){
			leg_dir = r_leg;
			if(swdown_flag[leg_dir]==1){
				Position pos = pos_control(leg_dir,roll);
				cout<<pos.x <<pos.y <<pos.z <<endl;
				cout<<roll*180/PI<<", "<<dvarphi*180/PI<<endl;

				//verify correct
				Angle an;
				Inv_kinematics(&an,&pos,1);
				cout<<an.q[0]<<an.q[1]<<an.q[2]<<endl;

				pre_state[leg_dir] = SWING_DOWN;
				swdown_contr.get_start(angles,pos,leg_dir);
				swdown_flag[leg_dir] = 0;
				// swdown_flag[l_leg] = 0;
				impact_happened=0;
				// timeflag = 0;
			}
		}
		else{
			// leg_dir = r_leg;
			swdown_flag[0] = 1;
			swdown_flag[1] = 1;
			// if(current_state[0]!=INITIAL) leg_dir = l_leg;
			// if(current_state[1]!=INITIAL) leg_dir = r_leg;
		}
	}

	for(int i(0);i<2;i++){
		if(pre_state[i]==SWING_UP){
			swup_contr.get_start(angles,init_pos[i],i);
			this->stance_end = 1;
		}
		if(pre_state[i]==STANCE){
			stance_contr.get_start(i,roll,dvarphi);
		}
	}

	//判断state
	Angle pos_angle;
	Eigen::Vector3d Tau_r,Tau_l;
	switch(current_state[0]){
		case INITIAL:
			// std::cout<<"left  INITIAL "<<std::endl;

			for(int i(0);i<3;i++){
				cmd->torque[i] = kp*(init_angle[l_leg].q[i] - leg->cbdata[i].p) + kd*(vel_ref - leg->cbdata[i].v);
			}

			break;

		case SWING_DOWN:
			// std::cout<<"left  SWING_DOWN"<<std::endl;
			this->stance_begin = swdown_contr.get_cmd(&pos_angle,0,torque2,w1);
			// cout<<pos_angle.q[0]<<pos_angle.q[1]<<pos_angle.q[2]<<endl;

			for(int i(0);i<3;i++){
				poserror.error[i] = pos_angle.q[i] - leg->cbdata[i].p;
				cmd->torque[i] = kp*poserror.error[i] + kd*(vel_ref - leg->cbdata[i].v);
			}
			
			break;

		case SWING_UP:
			// std::cout<<"left  SWING_UP"<<std::endl;
			impulse_able = swup_contr.get_cmd(&pos_angle,0);
			// cout<<pos_angle.q[0]<<pos_angle.q[1]<<pos_angle.q[2]<<endl;

			for(int i(0);i<3;i++){
				cmd->torque[i] = kp*(pos_angle.q[i] - leg->cbdata[i].p) + kd*(vel_ref - leg->cbdata[i].v);
			}
			
			break;

		case STANCE:
			// std::cout<<"left  STANCE"<<std::endl;
			Tau_l = stance_contr.get_cmd(0,roll,angles);

			cmd->torque[0] = Tau_l[0];
			cmd->torque[1] = Tau_l[1];
			cmd->torque[2] = Tau_l[2]/30*19;
			
			break;
	}
	
	switch(current_state[1]){
		case INITIAL:
			// std::cout<<"right  INITIAL"<<std::endl;
			
			for(int i(0);i<3;i++){
				cmd->torque[i+3] = kp*(init_angle[r_leg].q[i] - leg->cbdata[i+3].p) + kd*(vel_ref - leg->cbdata[i+3].v);
			}

			break;

		case SWING_DOWN:
			// std::cout<<"right  SWING_DOWN"<<std::endl;
			this->stance_begin = swdown_contr.get_cmd(&pos_angle,1,torque5,w4);
			// cout<<pos_angle.q[0]<<pos_angle.q[1]<<pos_angle.q[2]<<endl;

			for(int i(0);i<3;i++){
				poserror.error[i+3] = pos_angle.q[i] - leg->cbdata[i+3].p;
				cmd->torque[i+3] = kp*poserror.error[i+3] + kd*(vel_ref - leg->cbdata[i+3].v);
			}
			
			break;

		case SWING_UP:
			// std::cout<<"right  SWING_UP"<<std::endl;
			impulse_able = swup_contr.get_cmd(&pos_angle,1);
			// cout<<pos_angle.q[0]<<pos_angle.q[1]<<pos_angle.q[2]<<endl;
			
			for(int i(0);i<3;i++){
				cmd->torque[i+3] = kp*(pos_angle.q[i] - leg->cbdata[i+3].p) + kd*(vel_ref - leg->cbdata[i+3].v);
			}
			
			break;

		case STANCE:
			// std::cout<<"right  STANCE"<<std::endl;
			Tau_r = stance_contr.get_cmd(1,roll,angles);

			cmd->torque[3] = Tau_r[0];
			cmd->torque[4] = Tau_r[1];
			cmd->torque[5] = Tau_r[2]/30*19;

			break;
	}
	
	//torque_limit
	for(int i(0);i<6;i++){
		if(cmd->torque[i] < -18.0) cmd->torque[i] = -18.0;
		if(cmd->torque[i] > 18.0) cmd->torque[i] = 18.0;
	}

	return impulse_able;

}

int leg_controller::get_stance_ifend(void){
	return this->stance_end;
}

int leg_controller::get_stance_ifbegin(void){
	return this->stance_begin;
}

double leg_controller::get_global_force(void){
	return global_force;
}

double leg_controller::get_F(void){
	return stance_contr.get_force();
}

void leg_controller::set_height(double height){
	collision_height = height;
}

void leg_controller::goto_xyz(float xx,float yy,float zz,Leg direction){
  //define the variables
  Angle now_angle;
  Position now_position;
  Position now_velocity;

  //get the joint angle position and velocity
  
  if(direction == l_leg){
    for(int i(0);i<3;i++){
        now_angle.q[i] = leg->cbdata[i].p;
    }
  }
  else{
    for(int i(0);i<3;i++){
        now_angle.q[i] = leg->cbdata[i+3].p;
    }
  }

  //get the end point position and velocity using forward kinematics and jacobian
  Kinematics(&now_angle,&now_position,direction);

  now_velocity.x = 0;
  now_velocity.y = 0;
  now_velocity.z = 0;

  //input the desire position and velocity
  Position pdes;
  Position vdes;

  pdes.x = xx;
  pdes.y = yy;
  pdes.z = zz;

  vdes.x = 0;
  vdes.y = 0;
  vdes.z = 0;

  //init chabu
  init_chabu(&pdes,&vdes,&now_position,&now_velocity,stepnum,direction);

}

/*****************  state 实现  ****************/
swing_control::swing_control(){
  this->count = 0;
}
void swing_control::get_cmd(Angle *angle,Leg leg){
  this->count ++;
  if(this->count > stepnum){
	  this->count = stepnum;
	  pre_state[leg] = STANCE;
  }
  std::cout<<this->count<<std::endl;
  Position pos;
  chabu(&pos,this->count,leg);
  Inv_kinematics(angle,&pos,leg);
  
}
void swing_control::check_Transition(void){

}
void swing_control::get_start(Eigen::VectorXd angle, Position posi, Leg leg){
  if(current_state[leg] == INITIAL){
    //define the variables
    Angle now_angle;
    Position now_position;
	Position now_velocity;
    //get the joint angle position and velocity
    auto pos = angle;
    if(leg == l_leg){
      for(int i(0);i<3;i++){
          now_angle.q[i] = pos[i];
      }
    }
    else{
      for(int i(0);i<3;i++){
          now_angle.q[i] = pos[i+3];
      }
    }

    //get the end point position and velocity
    Kinematics(&now_angle,&now_position,leg);
	now_velocity.x = 0;
    now_velocity.y = 0;
    now_velocity.z = 0;

    //input the desire position
    Position pdes;
	Position vdes;

    pdes.x = posi.x;
    pdes.y = posi.y;
    pdes.z = posi.z;

    vdes.x = 0;
    vdes.y = 0;
    vdes.z = 0;

    //init chabu
    init_chabu(&pdes,&vdes,&now_position,&now_velocity,stepnum,leg);
    last_state[leg] = current_state[leg];
    current_state[leg] = SWING;
    this->count = 0;
    // std::cout<<"no"<<std::endl;
  }
}


swingdown_control::swingdown_control(){
  this->count = 0;
  this->angle_a = 0;
  this->last_vangle = 0;
}
int swingdown_control::get_cmd(Angle *angle,Leg leg,float torque,float vangle){
  int st_begin = 0;
  this->count ++;

  this->angle_a = vangle - this->last_vangle;

  //pretouched
  if(USE_Touch_detect==1){
	// if(fabs(torque)>2.0){
	// 	pre_state[leg] = STANCE;
	// 	st_begin = 1;
	// 	std::cout<<"pre_touched"<<std::endl;
	// } 
	if(this->count>180 && fabs(this->angle_a)>0.6){
		pre_state[leg] = STANCE;
		st_begin = 1;
		// std::cout<<this->angle_a<<std::endl;
		std::cout<<"pre_touched"<<std::endl;
	}
  }

  if(this->count > downstepnum){
	  this->count = downstepnum;
	  pre_state[leg] = STANCE;
	  st_begin = 1;
  } 
//   std::cout<<this->count<<std::endl;
  
  Position pos;
  chabu(&pos,this->count,leg);
  Inv_kinematics(angle,&pos,leg);

  
  this->last_vangle = vangle;
  
  return st_begin;
}
void swingdown_control::check_Transition(void){

}
void swingdown_control::get_start(Eigen::VectorXd angle, Position posi, Leg leg){
  if((current_state[leg] != pre_state[leg]) && (current_state[leg] == INITIAL)){
    //define the variables
    Angle now_angle;
    Position now_position;
    Position now_velocity;

    //get the joint angle position and velocity
    auto pos = angle;
    if(leg == l_leg){
      for(int i(0);i<3;i++){
          now_angle.q[i] = pos[i];
      }
    }
    else{
      for(int i(0);i<3;i++){
          now_angle.q[i] = pos[i+3];
      }
    }

    //get the end point position and velocity using forward kinematics and jacobian
    Kinematics(&now_angle,&now_position,leg);

    now_velocity.x = 0;
    now_velocity.y = 0;
    now_velocity.z = 0;

    //input the desire position and velocity
    Position pdes;
    Position vdes;

    pdes.x = posi.x;
    pdes.y = posi.y;
    pdes.z = posi.z;

    vdes.x = 0;
    vdes.y = 0;
    vdes.z = 0;

    //init chabu
    init_chabu(&pdes,&vdes,&now_position,&now_velocity,downstepnum,leg);
    last_state[leg] = current_state[leg];
    current_state[leg] = SWING_DOWN;
    this->count = 0;
	this->angle_a = 0;
  }
}

swingup_control::swingup_control(){
  this->count = 0;
}
int swingup_control::get_cmd(Angle *angle,Leg leg){
  int down = 0;
  this->count ++;
  if(this->count > upstepnum){
     this->count = upstepnum;
     current_state[leg] = INITIAL;
	 pre_state[leg] = SWING_DOWN;
	 down = 1;
  }
//   std::cout<<this->count<<std::endl;
  pre_state[leg] = INITIAL;
  Position pos;
  chabu(&pos,this->count,leg);
  Inv_kinematics(angle,&pos,leg);
  return down;
}
void swingup_control::check_Transition(void){

}
void swingup_control::get_start(Eigen::VectorXd angle, Position posi, Leg leg){
  if((pre_state[leg] != current_state[leg]) && (current_state[leg] == STANCE)){
    //define the variables
    Angle now_angle;
    Position now_position;
    Position now_velocity;

    //get the joint angle position and velocity
    auto pos = angle;
    if(leg == l_leg){
      for(int i(0);i<3;i++){
          now_angle.q[i] = pos[i];
      }
    }
    else{
      for(int i(0);i<3;i++){
          now_angle.q[i] = pos[i+3];
      }
    }

    //get the end point position and velocity using forward kinematics and jacobian
    Kinematics(&now_angle,&now_position,leg);

    now_velocity.x = 0;
    now_velocity.y = 0;
    now_velocity.z = 0;

    //input the desire position and velocity
    Position pdes;
    Position vdes;

    pdes.x = posi.x;
    pdes.y = posi.y;
    pdes.z = posi.z;

    vdes.x = 0;
    vdes.y = 0;
    vdes.z = 0;

    //init chabu
    init_chabu(&pdes,&vdes,&now_position,&now_velocity,upstepnum,leg);
    last_state[leg] = current_state[leg];
    current_state[leg] = SWING_UP;
    this->count = 0;
  }
}

stance_control::stance_control(){
  this->F.resize(3);
  this->F<<0,0,0;
  this->count = 0;
}
Eigen::VectorXd stance_control::get_cmd(Leg leg,float roll,Eigen::VectorXd angles){
  this->count ++;
  int force_period=0;
  if(leg == 0) force_period = l_force_period;
  else force_period = r_force_period;
  if(this->count >= force_period){
    pre_state[leg] = SWING_UP;
  }
//   std::cout<<this->count<<std::endl;

  auto rot_matrix = rpy2romatrix(roll,0,0);
  Eigen::VectorXd tau(3);
  if(leg==l_leg)  tau = calcu_JacoV(angles.head(3),leg).transpose() * rot_matrix.transpose() * F;
  else tau = calcu_JacoV(angles.tail(3),leg).transpose() * rot_matrix.transpose() * F;
  return tau;
}
void stance_control::check_Transition(void){

}
double stance_control::get_force(void){
	return this->F[2];
}
void stance_control::get_start(Leg leg,float roll,float dvarphi){
  if((current_state[leg] != pre_state[leg]) && (current_state[leg] == SWING_DOWN)){
	  
	if(USE_Optimizer==1){

		float L = hL*sin(fabs(roll))+wid/2*cos(roll)+dL;
		float Fz = fabs(global_Tau)/L*4/5;
		cout<<Fz<<endl;
		cout<<L<<endl;
		
		float Maxforce = 0;
		if(leg == 0) Maxforce = 190;
  		else Maxforce = 180;

		if(Fz<Maxforce && Fz>0){
			this->F << 0,0,-Fz;
		}
		else{
			this->F << 0,0,-Maxforce;
		}
		global_force = -Fz;

	}
	else{
		float L = hL*sin(fabs(roll))+wid/2*cos(roll)+dL;
		float Jt = 4.1306;
		float detaT = 0.05;
		float dvarphi_p = sqrt(43.3*(1-cos(roll)));
		float Fz = Jt*(dvarphi_p+fabs(dvarphi))/L/detaT*2/3;
		cout<<Fz<<endl;
		cout<<L<<endl;

		float Maxforce = 0;
		if(leg == 0) Maxforce = 200;
  		else Maxforce = 190;

		if(Fz<Maxforce && Fz>0){
			this->F << 0,0,-Fz;
		}
		else{
			this->F << 0,0,-Maxforce;
		}
		// this->F << 0,0,-160;
		global_force = -Fz;
		
	}
	
	cout<<F<<endl;
	cout<<roll*180/PI<<",  "<<dvarphi*180/PI<<endl;
	if_getTau = 0;
    this->count = 0;
    last_state[leg] = current_state[leg];
    current_state[leg] = STANCE;
  }
}

Position pos_control(Leg leg, double roll){
	Eigen::Vector3d pos_B,pos_H;
	double legbase_H;
	if(USE_Collision_model==1){
		legbase_H = collision_height;
		cout<<"USE_Collision_model:"<<legbase_H<<endl;
	}
	else{
		legbase_H = hL;
	}
	if(leg == l_leg){
		pos_H[0] = 0.20;
		pos_H[1] = dL+0.01;
		pos_H[2] = -legbase_H*cos(roll)+wid/2*sin(fabs(roll)); 
	}
	else{
		pos_H[0] = 0.20;
		pos_H[1] = -dL;
		pos_H[2] = -legbase_H*cos(roll)+wid/2*sin(fabs(roll));
	}

	auto rot_matrix = rpy2romatrix(roll,0,0);
	pos_B = rot_matrix.transpose() * pos_H;

	Position position_B;
	position_B.x = pos_B[0];
	position_B.y = pos_B[1];
	position_B.z = pos_B[2];
	// if(leg == l_leg){
	// 	position_B.y -= wid/2;
	// }
	// else{
	// 	position_B.y += wid/2;
	// }

	return position_B;
}

void set_xyz(Leg leg,Angle *angle,float xx,float yy,float zz){
  Position pos;
  pos.x = xx;
  pos.y = yy;
  pos.z = zz;

  Inv_kinematics(angle,&pos,leg);
}

void Kinematics(Angle *angle,Position *position,Leg direction){
	float q0 = angle->q[0];
	float q1 = angle->q[1];
	float q2 = angle->q[2];
	float L0 = Len0;
	float L1 = Len1;
	float L2 = Len2;

	if(direction == l_leg){
		position->x = L2*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L1*sin(q1);
		position->y = L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + L0*cos(q0) - L1*cos(q1)*sin(q0);
		position->z = - L0*sin(q0) - L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - L1*cos(q0)*cos(q1);
	}
	else if(direction == r_leg){
		position->x = - L2*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - L1*sin(q1);
		position->y = L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) - L0*cos(q0) - L1*cos(q1)*sin(q0);
		position->z = L0*sin(q0) - L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - L1*cos(q0)*cos(q1);
	}
}

void Inv_kinematics(Angle *angle,Position *position,Leg direction){
	float x = position->x;
	float y = position->y;
	float z = position->z;
	float L0 = Len0;
	float L1 = Len1;
	float L2 = Len2;
	if(direction == l_leg){
		angle->q[2] = -acos((x*x + y*y + z*z - L0*L0 - L1*L1 - L2*L2)/(2*L1*L2));
		angle->q[0] = 2*atan((-z-sqrt(y*y+z*z-L0*L0))/(y+L0));
		float a = L2*sin(angle->q[2]);
		float b = L1+L2*cos(angle->q[2]);
		float c = x;
		angle->q[1] = 2*atan((b-sqrt(b*b+a*a-c*c))/(a+c));
	}
	else if(direction == r_leg){
		angle->q[2] = acos((x*x + y*y + z*z - L0*L0 - L1*L1 - L2*L2)/(2*L1*L2));
		angle->q[0] = 2*atan((-z-sqrt(y*y+z*z-L0*L0))/(y-L0));
		float a = L2*sin(angle->q[2]);
		float b = L1+L2*cos(angle->q[2]);
		float c = -x;
		angle->q[1] = 2*atan((b-sqrt(b*b+a*a-c*c))/(a+c));
	}
}

void Kinematics_global(Angle *angle,Position *position,Leg direction){
	float q0 = angle->q[0];
	float q1 = angle->q[1];
	float q2 = angle->q[2];
	float L0 = Len0;
	float L1 = Len1;
	float L2 = Len2;

	if(direction == l_leg){
		position->x = L2*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L1*sin(q1);
		position->y = L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + L0*cos(q0) - L1*cos(q1)*sin(q0);
		position->z = - L0*sin(q0) - L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - L1*cos(q0)*cos(q1);
		position->y += wid/2;
	}
	else if(direction == r_leg){
		position->x = - L2*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - L1*sin(q1);
		position->y = L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) - L0*cos(q0) - L1*cos(q1)*sin(q0);
		position->z = L0*sin(q0) - L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - L1*cos(q0)*cos(q1);
		position->y -= wid/2;
	}
}

void Inv_kinematics_global(Angle *angle,Position *position,Leg direction){
	float x = position->x;
	float y = position->y;
	float z = position->z;
	float L0 = Len0;
	float L1 = Len1;
	float L2 = Len2;
	if(direction == l_leg){
		y -= wid/2;

		angle->q[2] = -acos((x*x + y*y + z*z - L0*L0 - L1*L1 - L2*L2)/(2*L1*L2));
		angle->q[0] = 2*atan((-z-sqrt(y*y+z*z-L0*L0))/(y+L0));
		float a = L2*sin(angle->q[2]);
		float b = L1+L2*cos(angle->q[2]);
		float c = x;
		angle->q[1] = 2*atan((b-sqrt(b*b+a*a-c*c))/(a+c));
	}
	else if(direction == r_leg){
		y += wid/2;

		angle->q[2] = acos((x*x + y*y + z*z - L0*L0 - L1*L1 - L2*L2)/(2*L1*L2));
		angle->q[0] = 2*atan((-z-sqrt(y*y+z*z-L0*L0))/(y-L0));
		float a = L2*sin(angle->q[2]);
		float b = L1+L2*cos(angle->q[2]);
		float c = -x;
		angle->q[1] = 2*atan((b-sqrt(b*b+a*a-c*c))/(a+c));
	}
}

Eigen::MatrixXd calcu_JacoV(Eigen::Vector3d angle,Leg leg){
	Eigen::MatrixXd Mat(3,3);
	if(leg == l_leg){
		float q0 = angle[0];
		float q1 = angle[1];
		float q2 = angle[2];
		float L0 = Len0;
		float L1 = Len1;
		float L2 = Len2;

		Mat << 0                                                                                         , L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L1*cos(q1)                        , L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)),
			   - L0*sin(q0) - L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - L1*cos(q0)*cos(q1), L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) + L1*sin(q0)*sin(q1), L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)),
			   L1*cos(q1)*sin(q0) - L0*cos(q0) - L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0))  , L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)) + L1*cos(q0)*sin(q1), L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1));
	}
	else{
		float q0 = angle[0];
		float q1 = angle[1];
		float q2 = angle[2];
		float L0 = Len0;
		float L1 = Len1;
		float L2 = Len2;

		Mat << 0																					   , - L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L1*cos(q1)						, -L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)),
			   L0*sin(q0) - L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - L1*cos(q0)*cos(q1), L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) + L1*sin(q0)*sin(q1), L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)),
			   L0*cos(q0) - L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + L1*cos(q1)*sin(q0), L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)) + L1*cos(q0)*sin(q1), L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1));
	}
	return Mat;
}

Eigen::MatrixXd calcu_Jaco(Angle angle,Leg leg){
	Eigen::MatrixXd Mat(3,3);
	if(leg == l_leg){
		float q0 = angle.q[0];
		float q1 = angle.q[1];
		float q2 = angle.q[2];
		float L0 = Len0;
		float L1 = Len1;
		float L2 = Len2;

		Mat << 0                                                                                         , L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L1*cos(q1)                        , L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)),
			   - L0*sin(q0) - L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - L1*cos(q0)*cos(q1), L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) + L1*sin(q0)*sin(q1), L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)),
			   L1*cos(q1)*sin(q0) - L0*cos(q0) - L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0))  , L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)) + L1*cos(q0)*sin(q1), L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1));
	}
	else{
		float q0 = angle.q[0];
		float q1 = angle.q[1];
		float q2 = angle.q[2];
		float L0 = Len0;
		float L1 = Len1;
		float L2 = Len2;

		Mat << 0																					   , - L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L1*cos(q1)						, -L2*(cos(q1)*cos(q2) - sin(q1)*sin(q2)),
			   L0*sin(q0) - L2*(cos(q0)*cos(q1)*cos(q2) - cos(q0)*sin(q1)*sin(q2)) - L1*cos(q0)*cos(q1), L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) + L1*sin(q0)*sin(q1), L2*(cos(q1)*sin(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)),
			   L0*cos(q0) - L2*(sin(q0)*sin(q1)*sin(q2) - cos(q1)*cos(q2)*sin(q0)) + L1*cos(q1)*sin(q0), L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1)) + L1*cos(q0)*sin(q1), L2*(cos(q0)*cos(q1)*sin(q2) + cos(q0)*cos(q2)*sin(q1));
	}
	return Mat;
}

void init_chabu(Position *pdes,Position *vdes,Position *pini,Position *vini,float tf,Leg leg){
	
	float p0[3];
	float pf[3];
	float v0[3];
	float vf[3];
	
	p0[0] = pini->x;
	p0[1] = pini->y;
	p0[2] = pini->z;
	
	pf[0] = pdes->x;
	pf[1] = pdes->y;
	pf[2] = pdes->z;
	
	v0[0] = vini->x;
	v0[1] = vini->y;
	v0[2] = vini->z;
	
	vf[0] = vdes->x;
	vf[1] = vdes->y;
	vf[2] = vdes->z;
	
	for(int i=0;i<3;i++){
		a0[leg][i] = p0[i];
		a1[leg][i] = v0[i];
		a2[leg][i] = 3.0/(tf*tf)*(pf[i]-p0[i])-2.0/tf*v0[i]-1.0/tf*vf[i];
		a3[leg][i] = -2.0/(tf*tf*tf)*(pf[i]-p0[i])+1.0/(tf*tf)*(vf[i]+v0[i]);
	}
}

void chabu(Position *pos,float step,Leg leg){
	float p[3];
	for(int i=0;i<3;i++){
		p[i] = a0[leg][i] + a1[leg][i]*step + a2[leg][i]*step*step +a3[leg][i]*step*step*step;
	}
	pos->x = p[0];
	pos->y = p[1];
	pos->z = p[2];
	
}

void init_linear(Position *pdes,Position *pini,float tf,Leg leg){

	float p0[3];
	float pf[3];

	p0[0] = pdes->x;
	p0[1] = pini->y;
	p0[2] = pini->z;

	pf[0] = pdes->x;
	pf[1] = pdes->y;
	pf[2] = pdes->z;

	for(int i=0;i<3;i++){
		bb[leg][i] = p0[i];
		kk[leg][i] = (pf[i]-p0[i])/tf;
		
	}
}

void linear(Position *pos,float step,Leg leg){
	float p[3];
	for(int i=0;i<3;i++){
		p[i] = bb[leg][i] + kk[leg][i]*step;
	}
	pos->x = p[0];
	pos->y = p[1];
	pos->z = p[2];
	
}

float linear(float ini,float des,float tf,float step){
	float p0 = ini;
	float pf = des;
	float b0,b1;
	b0 = p0;
	b1 = (pf-p0)/tf;
	float p = b0+b1*step;
	return p;
}