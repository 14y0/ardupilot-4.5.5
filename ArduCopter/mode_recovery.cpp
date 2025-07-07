#include "Copter.h"
#include "PIDController.h"

#if MODE_RECOVERY_ENABLED == ENABLED

/*
 * Init and run calls for flip flight mode
 *      original implementation in 2010 by Jose Julio
 *      Adapted and updated for AC2 in 2011 by Jason Short
 *
 *      Controls:
 *          RC7_OPTION - RC12_OPTION parameter must be set to "Flip" (AUXSW_FLIP) which is "2"
 *          Pilot switches to Stabilize, Acro or AltHold flight mode and puts ch7/ch8 switch to ON position
 *          Vehicle will Roll right by default but if roll or pitch stick is held slightly left, forward or back it will flip in that direction
 *          Vehicle should complete the roll within 2.5sec and will then return to the original flight mode it was in before flip was triggered
 *          Pilot may manually exit flip by switching off ch7/ch8 or by moving roll stick to >40deg left or right
 *
 *      State machine approach:
 *          FlipState::Start (while copter is leaning <45deg) : roll right at 400deg/sec, increase throttle
 *          FlipState::Roll (while copter is between +45deg ~ -90) : roll right at 400deg/sec, reduce throttle
 *          FlipState::Recover (while copter is between -90deg and original target angle) : use earth frame angle controller to return vehicle to original attitude
 */

#define RECOVERY_THR_INC        0.15f   // throttle increase during FlipState::Start stage (under 45deg lean angle)
#define RECOVERY_THR_DEC        0.30f   // throttle decrease during FlipState::Roll stage (between 45deg ~ -90deg roll)
#define RECOVERY_ROTATION_RATE  40000   // rotation rate request in centi-degrees / sec (i.e. 400 deg/sec)
#define RECOVERY_TIMEOUT_MS     1500    // timeout after 2.5sec.  Vehicle will switch back to original flight mode

// flip_init - initialise flip controller
bool ModeRecovery::init(bool ignore_checks)
{
    // capture original flight mode so that we can return to it after completion
    desired_control_mode = copter.desired_mode;

    // initialise state
    _state = RecoveryState::Start;
    start_time_ms = millis();
    pitch_dir = -1;
    throttle_out = 1.0f;
    motors->rc_write(4,0);
    motors->rc_write(5,0);
    motors->rc_write(6,0);
    motors->rc_write(7,0);
    // capture current attitude which will be used during the FlipState::Recovery stage
    return true;
}


// run - runs the flip controller
// should be called at 100hz or more
void ModeRecovery::run()
{   
    MY_i++;
    if(MY_i==101) MY_i=0;
    
    // // get pilot's desired throttle
    // if (!motors->armed() || ((millis() - start_time_ms) > RECOVERY_TIMEOUT_MS)) {
    //     _state = RecoveryState::Abandon;
    // }
    
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get corrected angle based on direction and axis of rotation
    // we flip the sign of flip_angle to minimize the code repetition
    int32_t flip_angle;

     

    
    flip_angle = ahrs.pitch_sensor * pitch_dir;
    

    // state machine
    switch (_state) {

    case RecoveryState::Start:
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f , 0.0f);

        // increase throttle
        throttle_out -= RECOVERY_THR_DEC;
        throttle_out = MAX(0.0f,throttle_out);
        // beyond 80deg lean angle move to next stage
        if (flip_angle < 2500) {
            _state = RecoveryState::MY_mid;
            MY_TIMER=0;
        }
        // output pilot's throttle without angle boost
        attitude_control->set_throttle_out(0.3f,false, g.throttle_filt);
        break;

    case RecoveryState::MY_mid:{
        MY_TIMER+=1;
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 2000.0f , 0.0f);
        if (MY_TIMER>=50) {
            _state = RecoveryState::Recovery;
            MY_TIMER=0;
        }
        attitude_control->set_throttle_out(0.6f,false, g.throttle_filt);
        break;
    }


    case RecoveryState::Recovery:{
        // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f * pitch_dir, 0.0f);
        // throttle_out += RECOVERY_THR_INC;
        // throttle_out = MIN(0.50f,throttle_out);
        // float recovery_angle;

        float current_height=0;
        float pid_output=0;

        float MY_dt=copter.scheduler.MY_LOOP();
        // if(MY_i==100)
        //     hal.console->printf("dt:%f",MY_dt);
        //*MYP.S. 初始化pid对象     参数：p:0.005 |i:0 |d:0.0001 |最大速度:3.0 |最大变化值:0.5
        static MY_PIDController pid(0.005, 0, 0.0001, 3.0, 0.5); 
        // if(MY_i==100)
        //     hal.console->printf("pid initial");
        float target_height;
      
        // recovery_angle = fabsf(0.0f - (float)ahrs.pitch_sensor);
    

        // check for successful recovery

        // if (fabsf(recovery_angle) <= 2500) {
            // if(MY_i==100)
            //     hal.console->printf("\ninif\n");        
            // MYP.S. PID处理
            target_height = 150.0f; //*MYP.S. 定高目标高度（cm）
            // hal.console->printf("target_height");
            current_height = copter.z_value; //*MYP.S. 当前高度（cm）
            // if(MY_i==100)
            //     hal.console->printf("  height:%f  ",current_height);            
            // hal.console->printf("current_height:%f",current_height);
            pid_output = pid.compute(target_height, current_height, MY_dt);
            // if(MY_i==100)
            //     hal.console->printf("pid_output:%f\n",pid_output);
            if (!pos_control->is_active_z()) {
                pos_control->init_z_controller();
            }//*MYP.S. 检测z控制器是否初始化成功，未初始化则初始化
            pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
            pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
            //*MYP.S. 设置加速度和速度的最大值
            pid_output=pid_output*100;
            pos_control->input_vel_accel_z(pid_output, 0.0);
            // pos_control->set_pos_target_z_from_climb_rate_cm(pid_output);
            // if(MY_i==100)
            //     hal.console->printf("z_value");
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
            pos_control->update_z_controller();
            // if(MY_i==100)
            //     hal.console->printf("z_update\n"); //MYP.S. 和角度自稳抢控制，丢失连接

            // restore original flight mode
            if(current_height<=165&&current_height>=135){
                MY_TIMER++;
                if(MY_TIMER==500){
                    if (!copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::UNKNOWN)) {
                        // this should never happen but just in case
                        copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
                    }
                }//*MYP.S. 最好加蜂鸣提示音指示是否自稳完成，由遥控器接管。
            }else{
                MY_TIMER=0;
            }
        // }
        


        break;
        }
    case RecoveryState::Abandon:
        // restore original flight mode
        hal.console->printf("\n\nabandon\n\n");
        if (!copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN)) {
            // this should never happen but just in case
            copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
        }
        break;
    default:
        break;
    }


}
#endif
