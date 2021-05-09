#include "Sub.h"

// manual_init - initialise manual controller
bool Sub::bottom_manual_init()
{
    // set target altitude to zero for reporting
	// 保持当前高度
    pos_control.set_alt_target(inertial_nav.get_altitude());
    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    set_neutral_controls();

    return true;
}

// manual_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void Sub::bottom_manual_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        return;
    }

    /*参考以前manual模式
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    motors.set_roll(channel_roll->norm_input());
    motors.set_pitch(channel_pitch->norm_input());
    motors.set_yaw(channel_yaw->norm_input() * g.acro_yaw_p / ACRO_YAW_P);
    motors.set_throttle(channel_throttle->norm_input());
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());*/
}
