#include "bicycle_controller.h"

using namespace std;
ofstream dataFile;

double rollMaxing = 2.0*PI/180;
int onlyonce = 1;
int eic_flag = 1;
float leg_t = 0;
int t_last = 0;
float t_record = 0;
Eigen::Vector3d velocity;
int countt=0;
///定义常数
double body_m = 25;
double body_h = 0.365;
double body_Jb = 0.8;
double body_l = 0.87;
double body_lb = 0.42;

double bike_a = 0.47;
double bike_b = 0.4;
double leg_height = hL;

double body_epsilon = 17.0 * PI / 180.0;
double angle_G = 1.0 * PI / 180.0;
double body_R = 0.21;
double body_lt = body_R * tan(body_epsilon);
double body_g = 9.8;

double body_Jt = body_m*body_h*body_h + body_Jb;

double body_b2 = 10, body_b1 = 6, body_b0 =3;
// double body_b2 = 3, body_b1 = 13, body_b0 = 10;
double body_a1 = 25, body_a0 = 180;
// double body_a1 = 10, body_a0 = 50;
// double body_a1 = 5, body_a0 = 11;

// double body_b2 = 10, body_b1 = 6, body_b0 = 3;
// double body_a1 = 1, body_a0 = 1;

double body_k = 1.05;
double limit = 0.2;

double body_vr = 1.2;
double tire_omega = body_vr/body_R;
// double LIMIT_STEER = 50.0 * PI / 180.0;
double LIMIT_STEER = 3;
double new_Wt = -PI/2;
double dt = 0.02;

///定义变量
double xe = 0; 			double ye = 0;
double dxe = 0;   	 	double dye = 0;
double ddxe = 0;   	 	double ddye = 0;
double dddxe = 0;   	double dddye = 0;
double ddddxe = 0;  	double ddddye = 0;
double dddddxe = 0; 	double dddddye = 0;

bicycle_controller::bicycle_controller(Bike_state *robot,char *ch){
	bike = robot;
	bicycle_controller::initial_param();
	char result[100] = {0};
	sprintf(result, "/home/hxy/0318/bike_data%s.txt", ch);
	dataFile.open(result, ofstream::app);
}

void bicycle_controller::initial_param(){
	/*自行车常量*/
	bikebot.mb = body_m;	bikebot.hb = body_h;
	bikebot.Jb = body_Jb;	bikebot.R  = body_R;
	bikebot.g  = body_g;	bikebot.l  = body_l;
	bikebot.lb = body_lb;	bikebot.lt = body_lt;
	bikebot.Jt = body_Jt;
	bikebot.epsilon = body_epsilon;
	bikebot.angle_G = angle_G;
	
	/*控制参数*/
	bikebot.b2 = body_b2;	bikebot.b1 = body_b1;	bikebot.b0 = body_b0;	
	bikebot.a1 = body_a1;	bikebot.a0 = body_a0;
	bikebot.k  = body_k;
	
	/*其他初值*/
	bikebot.psi_offset = bike->psi;
	bikebot.pitch_offset = bike->pitch;
	bikebot.vr = body_vr;
	bikebot.psi = 0;
	bikebot.x = 0;
	bikebot.y = 0;
	bikebot.last_target = 0;
	phi_cmd = 0;
	bikebot.accx = 0;
	bikebot.accy = 0;
	bikebot.accz = 0;
	bikebot.timer = 0;
	velocity << 0,0,0;
	// bikebot.pitch = -0.02;
	bikebot.pitch = bike->pitch ;
}

void bicycle_controller::reset(){
	bikebot.x = bikebot.xe;
}

void bicycle_controller::get_action(Bike_command *cmd,int eic_able,int include_u,int eic_unable,float leg_torque){
	
	if(USE_Leg==1){
		if(eic_unable==1){
			limit = 0.5;
			eic_flag = 0;
			leg_t = leg_torque;
		}
		if(eic_able==1){
			leg_t = 0;
		}
		// if(include_u==1){
		// 	eic_flag = 1;
			
		// }

	}
	
	///run the controller
	bicycle_controller::stateUpdate();
	bicycle_controller::balanceCalc();
	// bicycle_controller::balance();

	// cout<<leg_t<<endl;

	///get the control input
	cmd->phi = phi_cmd;
	cmd->speed = 1550;

	///some limit
	// stop the car
	if(fabs(bikebot.varphi)>10.0*PI/180){
		cmd->speed = 0;
	}
	// prevent out of control
	if(bikebot.timer<0.16){
		cmd->phi = 0;
	}
}

double bicycle_controller::get_height(void){
	return leg_height;
}

Input bicycle_controller::get_input(){
	Input inp;
	inp.psi = bikebot.psi;
	inp.varphi = bikebot.varphi;
	inp.dpsi = bikebot.dpsi;
	inp.dvarphi = bikebot.dvarphi;
	inp.T0 = bikebot.timer;
	inp.y = bikebot.y;
	inp.dx = bikebot.dx;
	inp.dy = bikebot.dy;
	inp.phi = bikebot.phi;
	return inp;
}
double bicycle_controller::get_timer(){
	return bikebot.timer;
}
void bicycle_controller::stateUpdate(){

	bikebot.accx = bike->accx;
	bikebot.accy = bike->accy;
	bikebot.accz = bike->accz;

	bikebot.varphi = bike->varphi - bikebot.angle_G;
	bikebot.dvarphi = bike->dvarphi;

	bikebot.phi = bike->bike_msg.phi;
	bikebot.dphi = bike->bike_msg.dphi;

	bikebot.psi = bike->psi - bikebot.psi_offset;
	if(bikebot.psi>200*PI/180) bikebot.psi = bikebot.psi - 2*PI;
	if(bikebot.psi<-200*PI/180) bikebot.psi = bikebot.psi + 2*PI;
	bikebot.dpsi = cos(bikebot.epsilon)/bikebot.l*bikebot.vr*tan(bikebot.phi)/cos(bikebot.varphi);

	// bikebot.pitch = bike->pitch - bikebot.pitch_offset;
	
	bikebot.dpitch = bike->dpitch;
	bikebot.pitch += bikebot.dpitch*dt;

	// bikebot.vr = bike->bike_msg.velocity * cos(bikebot.phi);

	bikebot.vr = bike->bike_msg.velocity;
	if(bikebot.timer==0) bikebot.vr=0.5;
	if(bikebot.vr>1.4) bikebot.vr=1.4;
	if(isnan(bikebot.vr)==1) bikebot.vr=1.3;

	// bikebot.last_vr = bikebot.vr;
	// bikebot.vr = 1.15;

	auto rot_matrix = rpy2romatrix(bikebot.varphi,bikebot.pitch,bikebot.psi);
	Eigen::Vector3d acc;
	acc << bikebot.accx, bikebot.accy, bikebot.accz;
	auto acceleration = rot_matrix*acc;
	velocity += bikebot.g * acceleration * dt;

	bikebot.dx = bikebot.vr * cos(bikebot.psi);
	bikebot.dy = bikebot.vr * sin(bikebot.psi);
	
	bikebot.x += bikebot.dx * dt;
	bikebot.y += bikebot.dy * dt;

	bikebot.timer += dt;

	bikebot.Tau_s = bikebot.mb*bikebot.hb*cos(bikebot.epsilon)*bikebot.vr*bikebot.vr*tan(bikebot.phi)/bikebot.l;
	bikebot.Tau_g = -bikebot.mb*bikebot.hb*bikebot.g*sin(bikebot.varphi);


	/*** record the data ***/
	// 朝TXT文档中写入数据
	dataFile << bikebot.varphi << ", " << bikebot.dvarphi << ", " << bikebot.phi << ", " 
			<< bikebot.dphi<< ", " << bikebot.psi << ", " << bikebot.dpsi << ", " 
			<< bikebot.vr << ", " << bikebot.x << ", " << bikebot.y << ", " 
			<< bikebot.accx << ", " << bikebot.accy << ", " << bikebot.accz << ", " 
			<< bikebot.varphie << ", " << bikebot.dvarphie << ", " << bikebot.dpitch << ", " 
			<< bikebot.new_phi << ", " << bikebot.dx << ", " << bikebot.dy << ", " 
			<< bikebot.xe << ", " << bikebot.ye << ", " << bikebot.timer << ", " 
			<< bikebot.Tau_s << ", " << bikebot.Tau_g << ", " << bike->dpsi << ", " 
			<< bikebot.vr << ", " << bikebot.pitch << ", " 
			<< velocity[0] << ", " << velocity[1] <<", "<< bike->pitch<< ", " << bikebot.psi_offset << ", " << bike->psi <<", "<<phi_cmd
			<< std::endl;

	// bikebot.vr = 1.0;
	// std::cout<<"varphi: "<<rad2deg(bikebot.varphi)<<std::endl;
	// std::cout<<"dvarphi: "<<rad2deg(bikebot.dvarphi)<<std::endl;
	// std::cout<<"phi: "<<rad2deg(bikebot.phi)<<std::endl;
	// std::cout<<"dphi: "<<rad2deg(bikebot.dphi)<<std::endl;
	// std::cout<<"psi: "<<rad2deg(bikebot.psi)<<std::endl;
	// std::cout<<"dpsi: "<<rad2deg(bikebot.dpsi)<<std::endl;
	// std::cout<<"x: "<<bikebot.x<<std::endl;
	// std::cout<<"y: "<<bikebot.y<<std::endl;
	// std::cout<<"vr: "<<bikebot.vr<<std::endl;
}

void bicycle_controller::balanceCalc(){
	/*变量读取*/
	double vr = bikebot.vr;		double dvr = 0;
	double psi = bikebot.psi;
	double dpsi = bikebot.dpsi;
	double dpsi3 = dpsi*dpsi*dpsi;//dpsi的三次方
	double sinpsi = sin(psi);		double cospsi = cos(psi);
	double x = bikebot.x;	double dx = vr * cospsi;	double ddx = -vr * sinpsi * dpsi;
	double y = bikebot.y;	double dy = vr * sinpsi;	double ddy = vr * cospsi * dpsi;
	double ve = vr;
	/*trajectory*/
	// 走圆	
	// double cr = 4.0; double w = ve/cr;
	// double x_offset = 0; double y_offset = cr;
	// new_Wt += w*dt; 
	// xe = x_offset + cr * cos(new_Wt); 	ye =  y_offset + cr*sin(new_Wt);
	// dxe = -cr * sin(new_Wt) * w;		dye = cr * cos(new_Wt) * w;	
	// ddxe = -dye * w; 		 ddye = dxe * w;
	// dddxe = -ddye * w; 		 dddye = ddxe * w;
	// ddddxe = -dddye * w; 	 ddddye = dddxe * w;
	// dddddxe = -ddddye * w; 	 dddddye = ddddxe * w;

	//走直线
	dxe = ve; dye = 0; 
	// xe = x;
	xe += ve*dt; 
	ye = 0;

	bikebot.xe = xe;
	bikebot.ye = ye;
	// std::cout<<"v: "<<bike->get_base_v()<<std::endl;
	// std::cout<<"x: "<<bikebot.x<<"xe: "<<bikebot.xe<<std::endl;
	// std::cout<<"y: "<<bikebot.y<<"ye: "<<bikebot.ye<<std::endl;
	/*External*/
	//利用控制律计算出前轮轨迹r(3)及其一次导和二次导
	double u_w_ext1 = dddxe + bikebot.b2 * (ddxe - ddx) + bikebot.b1 * (dxe - dx) + bikebot.b0 * (xe - x);
	double u_w_ext2 = dddye + bikebot.b2 * (ddye - ddy) + bikebot.b1 * (dye - dy) + bikebot.b0 * (ye - y);
	double du_w_ext1 = ddddxe + bikebot.b2 * (dddxe - u_w_ext1) + bikebot.b1 * (ddxe - ddx) + bikebot.b0 * (dxe - dx);
	double du_w_ext2 = ddddye + bikebot.b2 * (dddye - u_w_ext2) + bikebot.b1 * (ddye - ddy) + bikebot.b0 * (dye - dy);
	double ddu_w_ext1 = dddddxe + bikebot.b2 * (ddddxe - du_w_ext1) + bikebot.b1 * (dddxe - u_w_ext1) + bikebot.b0 * (ddxe - ddx);
	double ddu_w_ext2 = dddddye + bikebot.b2 * (ddddye - du_w_ext2) + bikebot.b1 * (dddye - u_w_ext2) + bikebot.b0 * (ddye - ddy);
	// double ur = vr*dpsi*dpsi+cospsi*u_w_ext1+sinpsi*u_w_ext2;
	//这里速度到二次导我们设为0
	double ur = 0;
	//计算出满足external条件的u_psi及其一阶和二阶导
	double u_psi = -2*dvr*dpsi/vr-sinpsi/vr*u_w_ext1+cospsi/vr*u_w_ext2;
	double du_psi = -2*((ur*vr-dvr*dvr)/vr/vr*dpsi+dvr/vr*u_psi) - (cospsi*dpsi*vr-sinpsi*dvr)/vr/vr*u_w_ext1 - sinpsi/vr*du_w_ext1
            + (-sinpsi*dpsi*vr-cospsi*dvr)/vr/vr*u_w_ext2 + cospsi/vr*du_w_ext2;
    double ddu_psi = (sinpsi*dpsi*dpsi-cospsi*u_psi)/vr*u_w_ext1 - 2*cospsi*dpsi/vr*du_w_ext1 - sinpsi/vr*ddu_w_ext1
            - (cospsi*dpsi*dpsi+sinpsi*u_psi)/vr*u_w_ext2 - 2*sinpsi*dpsi/vr*du_w_ext2 + cospsi/vr*ddu_w_ext2;
	
	/*Internal*/
	//将满足external条件的u_psi代入求解器，求解期望倾斜角
	double varphie = solution(vr, dpsi, u_psi);//牛顿迭代法求方程根
	bikebot.varphie = varphie;
	//求解期望倾斜角的一阶和二阶导
	double cosvare = cos(varphie);
	double sinvare = sin(varphie);
	double M1 = bikebot.hb*cosvare*dpsi*dpsi+bikebot.g/cosvare/cosvare;
	double hhh = bikebot.g*bikebot.lt*bikebot.lb*cos(bikebot.epsilon)/bikebot.hb;
    double M2 = (u_psi*vr+dpsi*dvr+2*bikebot.hb*dpsi*u_psi*sinvare+hhh*(u_psi*vr-dpsi*dvr)/vr/vr+bikebot.lb*du_psi);	
	double dvarphie = -M2/M1;
	bikebot.dvarphie = dvarphie;	
	double dM1 = (2*bikebot.hb*dpsi*u_psi*cosvare-bikebot.hb*dpsi*dpsi*sinvare+2*bikebot.g/cosvare/cosvare*sinvare/cosvare)*dvarphie;
	double dM2 = du_psi*vr+2*u_psi*dvr+dpsi*ur+2*bikebot.hb*(u_psi*u_psi*sinvare+dpsi*du_psi*sinvare+dpsi*u_psi*cosvare*dvarphie)
            + hhh * (du_psi*vr-2*u_psi*dvr-dpsi*ur+2*dpsi*dvr*dvr/vr)/vr/vr + bikebot.lb*ddu_psi;    
	double ddvarphie = (dM1*M2/M1 - dM2)/M1;
	//根据控制律求解合适到控制输入:varphi的二阶导
	double v_psi_int = ddvarphie + bikebot.a1*(dvarphie-bikebot.dvarphi)+ bikebot.a0*(varphie-bikebot.varphi);
	//将控制输入代入动力学模型方程，计算得到控制量u_psi
	double cosvar = cos(bikebot.varphi);
	double sinvar = sin(bikebot.varphi);
	double xxx = bikebot.mb*bikebot.g*bikebot.lt*bikebot.lb*cos(bikebot.epsilon);
	double f_varphi = (bikebot.mb*bikebot.hb*vr + bikebot.mb*bikebot.hb*bikebot.hb*sinvar*dpsi)*cosvar*dpsi
				+bikebot.mb*bikebot.hb*bikebot.g*sinvar + xxx*dpsi*cosvar/vr;
	double g_psi = bikebot.mb*bikebot.hb*bikebot.lb*cosvar;
	double u_psi_int = (-f_varphi + bikebot.Jt*v_psi_int+leg_t)/g_psi;
	//自此，以外部得到的u_r和内部得到的u_psi得到最终控制输入

	/*计算转向角*/
	double ddpsi;
	// if(eic_flag==1){
	// 	ddpsi=u_psi_int;
	// }
	// else{
	// 	ddpsi=u_psi;
	// }
	ddpsi=u_psi_int;
	bikebot.new_phi = atan((dpsi+ddpsi*dt)*bikebot.l/cos(bikebot.epsilon)*cosvar/vr)*bikebot.k;
	double target = -bikebot.new_phi*5.73;
	if(target > LIMIT_STEER) target = LIMIT_STEER;
	else if(target < -LIMIT_STEER) target = -LIMIT_STEER;
	bikebot.target = target;
	
	//控制限制
	
	target = bikebot.target;
	double last = bikebot.last_target;
	if(target > last) phi_cmd = (target - last) < limit ? target : last + limit;
	else phi_cmd = (last - target) < limit ? target : last - limit;
	bikebot.last_target = phi_cmd;

}

double solution(double vr, double dpsi, double u_psi){
	int cnt = 0;
	double init = 0;
	double res = 0;
	double fun, dfun;
	double tmp;
	double second = (vr + body_g*body_lt*body_lb*cos(body_epsilon)/body_h/vr)*dpsi + body_lb * u_psi;
	do{
		init = res;
		fun = body_h*dpsi*dpsi*sin(init)+body_g*tan(init)+ second;
		tmp = cos(init);
		dfun = body_h*dpsi*dpsi*tmp+body_g/tmp/tmp;
		res = init - fun/dfun;
		if(++cnt > 10){//至多迭代10次
			break;
		}
	}while(fabs(res-init)>1e-3);
	return res;
}

double rad2deg(double rad){
	return rad/PI*180.0;
}

void bicycle_controller::balance(){
    /*变量读取*/
	double vr = bikebot.vr;		double dvr = 0;
	double psi = bikebot.psi;
	double dpsi = bikebot.dpsi;
	double dpsi3 = dpsi*dpsi*dpsi;//dpsi的三次方
	double sinpsi = sin(psi);		double cospsi = cos(psi);
	double x = bikebot.x;	double dx = vr * cospsi;	double ddx = -vr * sinpsi * dpsi;
	double y = bikebot.y;	double dy = vr * sinpsi;	double ddy = vr * cospsi * dpsi;
	double ve = vr;
    double cosvar = cos(bikebot.varphi);
	double sinvar = sin(bikebot.varphi);

	/*trajectory*/
	//走直线
	dxe = ve; dye = 0; 
	// xe = x;
	xe += ve*dt; 
	ye = 0;
	bikebot.xe = xe;
	bikebot.ye = ye;

	/*External*/
	//利用控制律计算出前轮轨迹r(3)及其一次导和二次导
    double u_ext1 = ddxe + bikebot.b1 * (dxe - dx) + bikebot.b0 * (xe - x);
	double u_ext2 = ddye + bikebot.b1 * (dye - dy) + bikebot.b0 * (ye - y);
	double u_w_ext1 = dddxe + bikebot.b2 * (ddxe - ddx) + bikebot.b1 * (dxe - dx) + bikebot.b0 * (xe - x);
	double u_w_ext2 = dddye + bikebot.b2 * (ddye - ddy) + bikebot.b1 * (dye - dy) + bikebot.b0 * (ye - y);
	double du_w_ext1 = ddddxe + bikebot.b2 * (dddxe - u_w_ext1) + bikebot.b1 * (ddxe - ddx) + bikebot.b0 * (dxe - dx);
	double du_w_ext2 = ddddye + bikebot.b2 * (dddye - u_w_ext2) + bikebot.b1 * (ddye - ddy) + bikebot.b0 * (dye - dy);
	double ddu_w_ext1 = dddddxe + bikebot.b2 * (ddddxe - du_w_ext1) + bikebot.b1 * (dddxe - u_w_ext1) + bikebot.b0 * (ddxe - ddx);
	double ddu_w_ext2 = dddddye + bikebot.b2 * (ddddye - du_w_ext2) + bikebot.b1 * (dddye - u_w_ext2) + bikebot.b0 * (ddye - ddy);
	// double ur = vr*dpsi*dpsi+cospsi*u_w_ext1+sinpsi*u_w_ext2;
	//这里速度到二次导我们设为0
	double ur = 0;
	//计算出满足external条件的u_psi及其一阶和二阶导
    double u_ipsi = u_ext2/vr/cospsi; //dpsi
	double u_psi = -2*dvr*dpsi/vr-sinpsi/vr*u_w_ext1+cospsi/vr*u_w_ext2; //ddpsi
	double du_psi = -2*((ur*vr-dvr*dvr)/vr/vr*dpsi+dvr/vr*u_psi) - (cospsi*dpsi*vr-sinpsi*dvr)/vr/vr*u_w_ext1 - sinpsi/vr*du_w_ext1
            + (-sinpsi*dpsi*vr-cospsi*dvr)/vr/vr*u_w_ext2 + cospsi/vr*du_w_ext2; //dddpsi
    double ddu_psi = (sinpsi*dpsi*dpsi-cospsi*u_psi)/vr*u_w_ext1 - 2*cospsi*dpsi/vr*du_w_ext1 - sinpsi/vr*ddu_w_ext1 //ddddpsi
            - (cospsi*dpsi*dpsi+sinpsi*u_psi)/vr*u_w_ext2 - 2*sinpsi*dpsi/vr*du_w_ext2 + cospsi/vr*ddu_w_ext2;
	
	/*Internal*/
    //求解varphie，dvarphie，ddvarphie
    double varphie = atan(-vr*u_ipsi/bikebot.g);
    double cosvare = cos(varphie);
	double sinvare = sin(varphie);
    double dvarphie = -vr/bikebot.g*u_psi*cosvare*cosvare;
    double ddvarphie = -vr/bikebot.g*(du_psi*cosvare*cosvare-2*u_psi*cosvare*sinvare*dvarphie);
    bikebot.varphie = varphie;
    bikebot.dvarphie = dvarphie;
    //根据控制律求解合适到控制输入:varphi的二阶导
	double u_w_int = ddvarphie + bikebot.a1*(dvarphie-bikebot.dvarphi)+ bikebot.a0*(varphie-bikebot.varphi);
    //代入动力学方程求解转向角
    double steer_phi = atan(bikebot.l/(bikebot.mb*bikebot.hb*cos(bikebot.epsilon)*vr*vr)*(bikebot.Jt*u_w_int-bikebot.mb*bikebot.hb*bikebot.g*sinvar));
	/********自此，以外部得到的u_r和内部得到的u_psi得到最终控制输入********/

	/*计算转向角*/
	bikebot.new_phi = steer_phi;
	double target = -bikebot.new_phi*5.73;
	if(target > LIMIT_STEER) target = LIMIT_STEER;
	else if(target < -LIMIT_STEER) target = -LIMIT_STEER;
	bikebot.target = target;
	
	//控制限制
	double limit = 0.2;
	target = bikebot.target;
	double last = bikebot.last_target;
	if(target > last) phi_cmd = (target - last) < limit ? target : last + limit;
	else phi_cmd = (last - target) < limit ? target : last - limit;
	bikebot.last_target = phi_cmd;

}

