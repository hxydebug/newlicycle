#include "swing_leg_controller.h"

float _KP = 0.03;
float foot_clearance = 0.01;
float desired_height = 0.34;


swing_leg_controller::swing_leg_controller(Leg_state *bike,gait_generator *gait_generator,float desired_speed){
  licycle = bike;
  _gait_generator = gait_generator;

  last_leg_state = _gait_generator->desired_leg_state;

  Eigen::VectorXd nowang(6);
  for(int i(0);i<6;i++){
        nowang[i] = licycle->cbdata[i].p;
  }
  phase_switch_foot_local_position[0] = getFootPositionInBaswFrame(nowang,0);
  phase_switch_foot_local_position[1] = getFootPositionInBaswFrame(nowang,1);
  desired_xspeed = desired_speed;
  angles.resize(6);
  _desired_height.resize(3);
  action.resize(6);
  hip_positions[0].resize(3);
  hip_positions[1].resize(3);
  _desired_height << 0,0,desired_height-foot_clearance;
  angles.setConstant(0);
  action.setConstant(0);
  hip_positions[0] << detx+0.02,wid/2+0.13,detz;
  hip_positions[1] << detx+0.02,-wid/2-0.13,detz;

  swing_leg_controller::set_PDGain();

}

void swing_leg_controller::update(float current_time){
  std::vector<int> new_leg_state = _gait_generator->desired_leg_state;
  // can optimize better!
  Eigen::VectorXd nowang(6);
  for(int i(0);i<6;i++){
        nowang[i] = licycle->cbdata[i].p;
  }
  // Detects phase switch for each leg so we can remember the feet position at
  // the beginning of the swing phase.
  for(int i(0);i<2;i++){
    if(new_leg_state[i]==swing_leg && new_leg_state[i] != last_leg_state[i]){
      phase_switch_foot_local_position[i] = getFootPositionInBaswFrame(nowang,i);
    }
  }

  last_leg_state = new_leg_state;

}

Eigen::VectorXd swing_leg_controller::get_action(void){

  Eigen::VectorXd com_velocity(3);
  Eigen::VectorXd anglesV(6);
  anglesV.setConstant(0);

  com_velocity << licycle->body_v,0,0;
  // std::cout<<"velocity:"<<licycle->get_base_v()<<std::endl;
  for(int i(0);i<2;i++){
    if(_gait_generator->leg_state[i]==stance_leg || _gait_generator->leg_state[i]==Early_Contact){
      continue;
    }
  
    auto hip_horizontal_velocity = com_velocity;

    Eigen::VectorXd target_hip_horizontal_velocity(3);
    target_hip_horizontal_velocity << desired_xspeed,0,0;
    // std::cout<<"666"<<std::endl;
    Eigen::VectorXd foot_target_position = (hip_horizontal_velocity * _gait_generator->stance_duration[i])/2 - _KP*
                                (target_hip_horizontal_velocity - hip_horizontal_velocity) - _desired_height 
                                + hip_positions[i];
    // foot_target_position[0]=0.2;
    // std::cout<<foot_target_position[0]<<std::endl;
    Position end_position;
    end_position.x = foot_target_position[0];
    end_position.y = foot_target_position[1];
    end_position.z = foot_target_position[2];

    Position foot_position = gen_swing_foot_trajectory(_gait_generator->normalized_phase[i],phase_switch_foot_local_position[i],end_position);
    postarget[i] = foot_position;
    //get joint[i] angles
    Angle ans;
    Inv_kinematics_global(&ans,&foot_position,i);
    // std::cout<<"666:"<<std::endl;
    for(int j(0);j<3;j++){
      angles[3*i+j] = ans.q[j];
      // std::cout<<ans.q[j]<<std::endl;
    }
    

  }
  Eigen::VectorXd nowang(6);
  Eigen::VectorXd nowangV(6);
  for(int i(0);i<6;i++){
        nowang[i] = licycle->cbdata[i].p;
        nowangV[i] = licycle->cbdata[i].v;
  }

  return swing_leg_controller::tau(nowang,nowangV,angles,anglesV);


}

void swing_leg_controller::set_PDGain(){
	pGain.resize(6);
	dGain.resize(6);
	pGain.setConstant(20.0);
	dGain.setConstant(0.5);

}

Eigen::VectorXd swing_leg_controller::tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT){
  
  return dGain.cwiseProduct(vT-vA) + pGain.cwiseProduct(pT-pA);

}

float gen_parabola(float phase, float start, float mid, float end){
  /*** Gets a point on a parabola y = a x^2 + b x + c.

  The Parabola is determined by three points (0, start), (0.5, mid), (1, end) in
  the plane.

  Args:
    phase: Normalized to [0, 1]. A point on the x-axis of the parabola.
    start: The y value at x == 0.
    mid: The y value at x == 0.5.
    end: The y value at x == 1.

  Returns:
    The y value at x == phase.
  ***/
  float mid_phase = 0.5;
  float delta_1 = mid - start;
  float delta_2 = end - start;
  float delta_3 = mid_phase*mid_phase - mid_phase;
  float coef_a = (delta_1 - delta_2 * mid_phase) / delta_3;
  float coef_b = (delta_2 * mid_phase*mid_phase - delta_1) / delta_3;
  float coef_c = start;

  return coef_a * phase * phase + coef_b * phase + coef_c;
}

Position gen_swing_foot_trajectory(float input_phase, Position start_pos, Position end_pos){
  /*** Generates the swing trajectory using a parabola.

  Args:
    input_phase: the swing/stance phase value between [0, 1].
    start_pos: The foot's position at the beginning of swing cycle.
    end_pos: The foot's desired position at the end of swing cycle.

  Returns:
    The desired foot position at the current phase.
  ***/

  float phase = input_phase;
  if(input_phase <= 0.5) phase = 0.8 * sin(input_phase * PI);
  else phase = 0.8 + (input_phase - 0.5) * 0.4;

  Position pos;
  pos.x = (1 - phase) * start_pos.x + phase * end_pos.x;
  pos.y = (1 - phase) * start_pos.y + phase * end_pos.y;
  float max_clearance = 0.1;
  float mid = std::max(end_pos.z, start_pos.z) + max_clearance;
  pos.z = gen_parabola(phase, start_pos.z, mid, end_pos.z);

  return pos;
}