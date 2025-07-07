#include "Copter.h"
#include "PIDController.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAuto_takeoff::init(bool ignore_checks)
{

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAuto_takeoff::run()
{
    float current_height=0;
    float pid_output=0;
    float pid_output_yaw=0;
    float yaw_target=0;
    float front_distance=copter.x1_value-copter.x2_value;//*前向距离差
    float MY_dt=copter.scheduler.MY_LOOP();
    //*MYP.S. 初始化pid对象     参数：p:0.005 |i:0 |d:0.0001 |最大速度:0.3 |最大变化值:0.5
    static MY_PIDController pid(0.005, 0, 0.0001, 0.3, 0.5); 
    float target_height;
    target_height = 150.0f; //*MYP.S. 定高目标高度（cm）
    current_height = copter.z_value; //*MYP.S. 当前高度（cm）
    int yaw_dt=MY_dt;
    //*高度PID
    pid_output = pid.compute(target_height, current_height, yaw_dt);
    //*角度PID
    pid_output_yaw=pid.compute(yaw_target,front_distance,yaw_dt);
    //*MYP.S. 转换为cm/s
    pid_output=pid_output*100;
    //*MYP.S. 转换为cm/s
    pid_output_yaw=pid_output_yaw*100;
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());

    //*pid接管yaw角控制，yaw角输入被注释掉
    // get pilot's desired yaw rate
    //float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    // get pilot desired climb rate
    float target_climb_rate = pid_output;

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // call attitude controller
    //!注意测试yaw角加速度方向
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, pid_output_yaw);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}