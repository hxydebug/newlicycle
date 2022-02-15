#ifndef __SWING_LEG_CONTROL_H
#define __SWING_LEG_CONTROL_H

#include "gait_generator.h"
#include "control.h"
#include "bikebot_control.h"


class swing_leg_controller{
public:
    swing_leg_controller(Leg_state *bike,gait_generator *gait_generator,float desired_speed);
    void update(float current_time);
    Eigen::VectorXd get_action(void);
    void set_PDGain();
    Eigen::VectorXd tau(Eigen::VectorXd pA,Eigen::VectorXd vA,Eigen::VectorXd pT,Eigen::VectorXd vT);
    Position postarget[2];
    float desired_xspeed;
private:
    Eigen::VectorXd pGain,dGain;
    gait_generator *_gait_generator;
    Leg_state *licycle;
    std::vector<int> last_leg_state = {0,0};
    Position phase_switch_foot_local_position[2];
    Eigen::VectorXd _desired_height;
    Eigen::VectorXd hip_positions[2];
    Eigen::VectorXd angles;
    Eigen::VectorXd action;

};


float gen_parabola(float phase, float start, float mid, float end);
Position gen_swing_foot_trajectory(float input_phase, Position start_pos, Position end_pos);

#endif