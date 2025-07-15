#include "Copter.h"

#if MODE_CLIMB_ENABLED == ENABLED

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

#define CLIMB_THR_INC        0.25f   // throttle increase during FlipState::Start stage (under 45deg lean angle)
#define CLIMB_THR_DEC        0.24f   // throttle decrease during FlipState::Roll stage (between 45deg ~ -90deg roll)
#define CLIMB_ROTATION_RATE  8500   // rotation rate request in centi-degrees / sec (i.e. 400 deg/sec)
#define CLIMB_TIMEOUT_MS     2500    // timeout after 2.5sec.  Vehicle will switch back to original flight mode
#define Turn_Channel (int16_t)last_value[0]
#define Thr_Channel (int16_t)last_value[2]
#define Lock_Channel (int16_t)last_value[5]
#define Turn_Mid 1500
#define Thr_Mid 1500
#define Threshold 350
#define Up_Rate 35
// flip_init - initialise flip controller
bool ModeClimb::init(bool ignore_checks)
{
    // only allow flip from some flight modes, for example ACRO, Stabilize, AltHold or FlowHold flight modes

    // if in acro or stabilize ensure throttle is above zero
    if (copter.ap.throttle_zero && (copter.flightmode->mode_number() == Mode::Number::ACRO || copter.flightmode->mode_number() == Mode::Number::STABILIZE )) {
        return false;
    }

    if (copter.flightmode->mode_number() != Mode::Number::STABILIZE && copter.flightmode->mode_number() != Mode::Number::ALT_HOLD){
        return false;
    }

    // ensure roll input is less than 40deg
    if (abs(channel_roll->get_control_in()) >= 4000) {
        return false;
    }

    // only allow flip when flying
    if (!motors->armed() || copter.ap.land_complete) {
        return false;
    }

    // capture original flight mode so that we can return to it after completion
    orig_control_mode = copter.flightmode->mode_number();

    // initialise state
    _state = ClimbState::Start;
    start_time_ms = millis();
    pitch_dir = -1;
    throttle_out = 0.0f;

    // capture current attitude which will be used during the FlipState::Recovery stage
    const float angle_max = copter.aparm.angle_max;
    orig_attitude.x = constrain_float(ahrs.roll_sensor, -angle_max, angle_max);
    orig_attitude.y = constrain_float(ahrs.pitch_sensor, -angle_max, angle_max);
    orig_attitude.z = ahrs.yaw_sensor;

    return true;
}

// run - runs the flip controller
// should be called at 100hz or more
void ModeClimb::run()
{
    // if pilot inputs roll > 40deg or timeout occurs abandon flip
    if (!motors->armed()) {
        _state = ClimbState::Abandon;
    }
    read_channels();
    // get pilot's desired throttle
    

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get corrected angle based on direction and axis of rotation
    // we flip the sign of flip_angle to minimize the code repetition
    // int32_t flip_angle;

    
    // flip_angle = ahrs.pitch_sensor * pitch_dir;
    

    // state machine
    switch (_state) {

    case ClimbState::Start:
        // under 45 degrees request 400deg/sec roll or pitch
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, CLIMB_ROTATION_RATE * pitch_dir, 0.0f);

        // increase throttle
        throttle_out = 1.0f;

        // beyond 80deg lean angle move to next stage
        if (Lock_Channel>1500) {
            _state = ClimbState::Climb;
        }
         break;


    case ClimbState::Climb:
        // between 45deg ~ -90deg request 400deg/sec roll
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, CLIMB_ROTATION_RATE * pitch_dir, 0.0f);
        update_Thr();
        // decrease throttle
        throttle_out = 1.0f;

        break;


    case ClimbState::Abandon:
        // restore original flight mode
        if (!copter.set_mode(orig_control_mode, ModeReason::UNKNOWN)) {
            // this should never happen but just in case
            copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
        }

        break;
    }

    // output pilot's throttle without angle boost
    attitude_control->set_throttle_out(throttle_out, true, g.throttle_filt);
}

void ModeClimb::output_to_motors()
{

    if (_state == ClimbState::Climb){
       if (Lock_Channel<1500){
        motors->rc_write(4,0);
        motors->rc_write(5,0);
        motors->rc_write(6,0);
        motors->rc_write(7,0);
        motors->rc_write(0,2000);
        motors->rc_write(1,2000);
        motors->rc_write(2,2000);
        motors->rc_write(3,2000);
        }
        else{
            motors->rc_write(6,Thr_Forward_L);
            motors->rc_write(7,Thr_Backup_L);
            motors->rc_write(5,Thr_Forward_R);
            motors->rc_write(4,Thr_Backup_R);
            motors->rc_write(0,2000);
            motors->rc_write(1,2000);
            motors->rc_write(2,2000);
            motors->rc_write(3,2000);
        }
    }
    else{
        motors->output();
    }

}

void ModeClimb::update_Thr(void)
{
    int16_t Thr_Fix = 0;
		Differ_Speed = Turn_Channel - Turn_Mid;
	    //printf(" Differ_Speed:%d",Differ_Speed);
		if (Differ_Speed>Threshold)//上边界 threshold=1000
		{
			Differ_Speed = Threshold;
		}
		if (Differ_Speed<-Threshold)//下边界
		{
			Differ_Speed = -Threshold;
		}
		//left
		Thr_Fix = Thr_Channel+Differ_Speed;
		if ((Thr_Fix-Thr_Mid)>0)
		{
			if ((Thr_Fix-Thr_Mid)>500)
			{
				Thr_Fix = 500+Thr_Mid;
			}
			Thr_Forward_L = (Thr_Fix-Thr_Mid)*Up_Rate;//保证在0——10000之间 ，前轮
			Thr_Backup_L = 0; 
		}
		else
		{
			if ((Thr_Fix-Thr_Mid)<-500)
			{
				Thr_Fix = Thr_Mid-500;
			}
			Thr_Backup_L = (Thr_Fix-Thr_Mid)*-Up_Rate;
			Thr_Forward_L = 0;
		}
		//right
		Thr_Fix = Thr_Channel-Differ_Speed;
		if ((Thr_Fix-Thr_Mid)>0)
		{
			if ((Thr_Fix-Thr_Mid)>1500)
			{
				Thr_Fix = 1500+Thr_Mid;
			}
			Thr_Forward_R = (Thr_Fix-Thr_Mid)*Up_Rate;
			Thr_Backup_R = 0;
		}
		else
		{
			if ((Thr_Fix-Thr_Mid)<-1500)
			{
				Thr_Fix = Thr_Mid-1500;
			}
			Thr_Backup_R = (Thr_Fix-Thr_Mid)*-Up_Rate;
			Thr_Forward_R = 0;
		}
}
void ModeClimb::read_channels(void)
{
    uint8_t nchannels = hal.rcin->num_channels();  // Get the numbers channels detected by RC_INPUT.
    if (nchannels == 0) {
        return;
    }

    if (nchannels > 8) {
        nchannels = 8;
    }

    for (uint8_t i = 0; i < nchannels; i++) {
        uint16_t v = hal.rcin->read(i);
        if (last_value[i] != v) {
            last_value[i] = v;
        }
    }

}

#endif
