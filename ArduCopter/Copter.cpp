/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  ArduCopter (also known as APM, APM:Copter or just Copter)
 *  Wiki:           copter.ardupilot.org
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen,
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to: Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Andy Piper          :Harmonic notch, In-flight FFT, Bi-directional DShot, various drivers
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel         :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland :PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  Sebastian Quilter   :SmartRTL
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/ArduPilot/ardupilot/graphs/contributors
 *  Wiki: https://copter.ardupilot.org/
 *
 */

#include "Copter.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, _interval_ticks, _max_time_micros, _prio) SCHED_TASK_CLASS(Copter, &copter, func, _interval_ticks, _max_time_micros, _prio)
#define FAST_TASK(func) FAST_TASK_CLASS(Copter, &copter, func)

/*
  scheduler table - all tasks should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table in AP_Vehicle to determine
  the order in which tasks are run.  Convenience methods SCHED_TASK
  and SCHED_TASK_CLASS are provided to build entries in this structure:

SCHED_TASK arguments:
 - name of static function to call
 - rate (in Hertz) at which the function should be called
 - expected time (in MicroSeconds) that the function should take to run
 - priority (0 through 255, lower number meaning higher priority)

SCHED_TASK_CLASS arguments:
 - class name of method to be called
 - instance on which to call the method
 - method to call on that instance
 - rate (in Hertz) at which the method should be called
 - expected time (in MicroSeconds) that the method should take to run
 - priority (0 through 255, lower number meaning higher priority)

 */
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    // update INS immediately to get current gyro data populated
    FAST_TASK_CLASS(AP_InertialSensor, &copter.ins, update),
    // run low level rate controllers that only require IMU data
    FAST_TASK(run_rate_controller),
#if AC_CUSTOMCONTROL_MULTI_ENABLED == ENABLED
    FAST_TASK(run_custom_controller),
#endif
#if FRAME_CONFIG == HELI_FRAME
    FAST_TASK(heli_update_autorotation),
#endif //HELI_FRAME
    // send outputs to the motors library immediately
    FAST_TASK(motors_output),
     // run EKF state estimator (expensive)
    FAST_TASK(read_AHRS),
#if FRAME_CONFIG == HELI_FRAME
    FAST_TASK(update_heli_control_dynamics),
#endif //HELI_FRAME
    // Inertial Nav
    FAST_TASK(read_inertia),
    // check if ekf has reset target heading or position
    FAST_TASK(check_ekf_reset),
    // run the attitude controllers
    FAST_TASK(update_flight_mode),
    // update home from EKF if necessary
    FAST_TASK(update_home_from_EKF),
    // check if we've landed or crashed
    FAST_TASK(update_land_and_crash_detectors),
    // surface tracking update
    FAST_TASK(update_rangefinder_terrain_offset),
#if HAL_MOUNT_ENABLED
    // camera mount's fast update
    FAST_TASK_CLASS(AP_Mount, &copter.camera_mount, update_fast),
#endif
#if HAL_LOGGING_ENABLED
    FAST_TASK(Log_Video_Stabilisation),
#endif

    SCHED_TASK(rc_loop,              250,    130,  3),
    SCHED_TASK(throttle_loop,         50,     75,  6),
#if AP_FENCE_ENABLED
    SCHED_TASK(fence_check,           25,    100,  7),
#endif
    SCHED_TASK_CLASS(AP_GPS,               &copter.gps,                 update,          50, 200,   9),
    SCHED_TASK(update_swarm_message,50,200,10),
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(AP_OpticalFlow,          &copter.optflow,             update,         200, 160,  12),
#endif
    SCHED_TASK(update_batt_compass,   10,    120, 15),
    SCHED_TASK_CLASS(RC_Channels, (RC_Channels*)&copter.g2.rc_channels, read_aux_all,    10,  50,  18),
    SCHED_TASK(arm_motors_check,      10,     50, 21),
#if TOY_MODE_ENABLED == ENABLED
    SCHED_TASK_CLASS(ToyMode,              &copter.g2.toy_mode,         update,          10,  50,  24),
#endif
    SCHED_TASK(auto_disarm_check,     10,     50,  27),
    SCHED_TASK(auto_trim,             10,     75,  30),
#if RANGEFINDER_ENABLED == ENABLED
    SCHED_TASK(read_rangefinder,      20,    100,  33),
#endif
#if HAL_PROXIMITY_ENABLED
    SCHED_TASK_CLASS(AP_Proximity,         &copter.g2.proximity,        update,         200,  50,  36),
#endif
#if AP_BEACON_ENABLED
    SCHED_TASK_CLASS(AP_Beacon,            &copter.g2.beacon,           update,         400,  50,  39),
#endif
    SCHED_TASK(update_altitude,       10,    100,  42),
    SCHED_TASK(run_nav_updates,       50,    100,  45),
    SCHED_TASK(update_throttle_hover,100,     90,  48),
#if MODE_SMARTRTL_ENABLED == ENABLED
    SCHED_TASK_CLASS(ModeSmartRTL,         &copter.mode_smartrtl,       save_position,    3, 100,  51),
#endif
#if HAL_SPRAYER_ENABLED
    SCHED_TASK_CLASS(AC_Sprayer,           &copter.sprayer,               update,         3,  90,  54),
#endif
    SCHED_TASK(three_hz_loop,          3,     75, 57),
#if AP_SERVORELAYEVENTS_ENABLED
    SCHED_TASK_CLASS(AP_ServoRelayEvents,  &copter.ServoRelayEvents,      update_events, 50,  75,  60),
#endif
    SCHED_TASK_CLASS(AP_Baro,              &copter.barometer,             accumulate,    50,  90,  63),
#if AC_PRECLAND_ENABLED
    SCHED_TASK(update_precland,      400,     50,  69),
#endif
#if FRAME_CONFIG == HELI_FRAME
    SCHED_TASK(check_dynamic_flight,  50,     75,  72),
#endif
#if HAL_LOGGING_ENABLED
    SCHED_TASK(loop_rate_logging, LOOP_RATE,    50,  75),
#endif
    SCHED_TASK(one_hz_loop,            1,    100,  81),
    SCHED_TASK(ekf_check,             10,     75,  84),
    SCHED_TASK(check_vibration,       10,     50,  87),
    SCHED_TASK(gpsglitch_check,       10,     50,  90),
    SCHED_TASK(takeoff_check,         50,     50,  91),
#if AP_LANDINGGEAR_ENABLED
    SCHED_TASK(landinggear_update,    10,     75,  93),
#endif
    SCHED_TASK(standby_update,        100,    75,  96),
    SCHED_TASK(lost_vehicle_check,    10,     50,  99),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_receive, 400, 180, 102),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_send,    400, 550, 105),
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount,             &copter.camera_mount,        update,          50,  75, 108),
#endif
#if AP_CAMERA_ENABLED
    SCHED_TASK_CLASS(AP_Camera,            &copter.camera,              update,          50,  75, 111),
#endif
#if HAL_LOGGING_ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350, 114),
    SCHED_TASK(twentyfive_hz_logging, 25,    110, 117),
    SCHED_TASK_CLASS(AP_Logger,            &copter.logger,              periodic_tasks, 400, 300, 120),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,    &copter.ins,                 periodic,       400,  50, 123),

#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler,         &copter.scheduler,           update_logging, 0.1,  75, 126),
#endif
#if AP_RPM_ENABLED
    SCHED_TASK_CLASS(AP_RPM,               &copter.rpm_sensor,          update,          40, 200, 129),
#endif
#if AP_TEMPCALIBRATION_ENABLED
    SCHED_TASK_CLASS(AP_TempCalibration,   &copter.g2.temp_calibration, update,          10, 100, 135),
#endif
#if HAL_ADSB_ENABLED
    SCHED_TASK(avoidance_adsb_update, 10,    100, 138),
#endif
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,          10,    100, 141),
#endif
#if AP_TERRAIN_AVAILABLE
    SCHED_TASK(terrain_update,        10,    100, 144),
#endif
#if AP_GRIPPER_ENABLED
    SCHED_TASK_CLASS(AP_Gripper,           &copter.g2.gripper,          update,          10,  75, 147),
#endif
#if AP_WINCH_ENABLED
    SCHED_TASK_CLASS(AP_Winch,             &copter.g2.winch,            update,          50,  50, 150),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75, 153),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75, 156),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75, 159),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,      3.3,   75, 162),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,     75, 165),
#endif
#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button,            &copter.button,              update,           5, 100, 168),
#endif
#if STATS_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Stats,             &copter.g2.stats,            update,           1, 100, 171),
#endif
};

void Copter::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Copter::_failsafe_priorities[7];

#if AP_SCRIPTING_ENABLED
#if MODE_GUIDED_ENABLED == ENABLED
// start takeoff to given altitude (for use by scripting)
bool Copter::start_takeoff(float alt)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    if (mode_guided.do_user_takeoff_start(alt * 100.0f)) {
        copter.set_auto_armed(true);
        return true;
    }
    return false;
}

// set target location (for use by scripting)
bool Copter::set_target_location(const Location& target_loc)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    return mode_guided.set_destination(target_loc);
}

// set target position (for use by scripting)
bool Copter::set_target_pos_NED(const Vector3f& target_pos, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative, bool terrain_alt)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);

    return mode_guided.set_destination(pos_neu_cm, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, yaw_relative, terrain_alt);
}

// set target position and velocity (for use by scripting)
bool Copter::set_target_posvel_NED(const Vector3f& target_pos, const Vector3f& target_vel)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);

    return mode_guided.set_destination_posvelaccel(pos_neu_cm, vel_neu_cms, Vector3f());
}

// set target position, velocity and acceleration (for use by scripting)
bool Copter::set_target_posvelaccel_NED(const Vector3f& target_pos, const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);
    const Vector3f accel_neu_cms(target_accel.x * 100.0f, target_accel.y * 100.0f, -target_accel.z * 100.0f);

    return mode_guided.set_destination_posvelaccel(pos_neu_cm, vel_neu_cms, accel_neu_cms, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, yaw_relative);
}

bool Copter::set_target_velocity_NED(const Vector3f& vel_ned)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // convert vector to neu in cm
    const Vector3f vel_neu_cms(vel_ned.x * 100.0f, vel_ned.y * 100.0f, -vel_ned.z * 100.0f);
    mode_guided.set_velocity(vel_neu_cms);
    return true;
}

// set target velocity and acceleration (for use by scripting)
bool Copter::set_target_velaccel_NED(const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool relative_yaw)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // convert vector to neu in cm
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);
    const Vector3f accel_neu_cms(target_accel.x * 100.0f, target_accel.y * 100.0f, -target_accel.z * 100.0f);

    mode_guided.set_velaccel(vel_neu_cms, accel_neu_cms, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, relative_yaw);
    return true;
}

bool Copter::set_target_angle_and_climbrate(float roll_deg, float pitch_deg, float yaw_deg, float climb_rate_ms, bool use_yaw_rate, float yaw_rate_degs)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    Quaternion q;
    q.from_euler(radians(roll_deg),radians(pitch_deg),radians(yaw_deg));

    mode_guided.set_angle(q, Vector3f{}, climb_rate_ms*100, false);
    return true;
}
#endif

#if MODE_CIRCLE_ENABLED == ENABLED
// circle mode controls
bool Copter::get_circle_radius(float &radius_m)
{
    radius_m = circle_nav->get_radius() * 0.01f;
    return true;
}

bool Copter::set_circle_rate(float rate_dps)
{
    circle_nav->set_rate(rate_dps);
    return true;
}
#endif

// set desired speed (m/s). Used for scripting.
bool Copter::set_desired_speed(float speed)
{
    return flightmode->set_speed_xy(speed * 100.0f);
}

#if MODE_AUTO_ENABLED == ENABLED
// returns true if mode supports NAV_SCRIPT_TIME mission commands
bool Copter::nav_scripting_enable(uint8_t mode)
{
    return mode == (uint8_t)mode_auto.mode_number();
}

// lua scripts use this to retrieve the contents of the active command
bool Copter::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4)
{
    if (flightmode != &mode_auto) {
        return false;
    }

    return mode_auto.nav_script_time(id, cmd, arg1, arg2, arg3, arg4);
}

// lua scripts use this to indicate when they have complete the command
void Copter::nav_script_time_done(uint16_t id)
{
    if (flightmode != &mode_auto) {
        return;
    }

    return mode_auto.nav_script_time_done(id);
}
#endif

// returns true if the EKF failsafe has triggered.  Only used by Lua scripts
bool Copter::has_ekf_failsafed() const
{
    return failsafe.ekf;
}

#endif // AP_SCRIPTING_ENABLED

// returns true if vehicle is landing. Only used by Lua scripts
bool Copter::is_landing() const
{
    return flightmode->is_landing();
}

// returns true if vehicle is taking off. Only used by Lua scripts
bool Copter::is_taking_off() const
{
    return flightmode->is_taking_off();
}

bool Copter::current_mode_requires_mission() const
{
#if MODE_AUTO_ENABLED == ENABLED
        return flightmode == &mode_auto;
#else
        return false;
#endif
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    rc().read_mode_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Copter::throttle_loop()
{
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_mix();

    // check auto_armed status
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // update rotor speed
    heli_update_rotor_speed_targets();

    // update trad heli swash plate movement
    heli_update_landing_swash();
#endif

    // compensate for ground effect (if enabled)
    update_ground_effect_detector();
    update_ekf_terrain_height_stable();
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Copter::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if(AP::compass().available()) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors->get_throttle());
        compass.set_voltage(battery.voltage());
        compass.read();
    }
}

#if HAL_LOGGING_ENABLED
// Full rate logging of attitude, rate and pid loops
// should be run at loop rate
void Copter::loop_rate_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        Log_Write_Attitude();
        Log_Write_PIDS(); // only logs if PIDS bitmask is set
    }
    if (should_log(MASK_LOG_FTN_FAST)) {
        AP::ins().write_notch_log_messages();
    }
    if (should_log(MASK_LOG_IMU_FAST)) {
        AP::ins().Write_IMU();
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Copter::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        Log_Write_Attitude();
    }
    if (!should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
    // log at 10Hz if PIDS bitmask is selected, even if no ATT bitmask is selected; logs at looprate if ATT_FAST and PIDS bitmask set
        Log_Write_PIDS();
    }
    // log EKF attitude data always at 10Hz unless ATTITUDE_FAST, then do it in the 25Hz loop
    if (!should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        motors->Log_Write();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
        if (rssi.enabled()) {
            logger.Write_RSSI();
        }
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS() || !flightmode->has_manual_throttle())) {
        pos_control->write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control->control_monitor_log();
#if HAL_PROXIMITY_ENABLED
        g2.proximity.log();  // Write proximity sensor distances
#endif
#if AP_BEACON_ENABLED
        g2.beacon.log();
#endif
    }
#if FRAME_CONFIG == HELI_FRAME
    Log_Write_Heli();
#endif
#if AP_WINCH_ENABLED
    if (should_log(MASK_LOG_ANY)) {
        g2.winch.write_log();
    }
#endif
#if HAL_MOUNT_ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif
}

// twentyfive_hz_logging - should be run at 25hz
void Copter::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }

    if (should_log(MASK_LOG_IMU) && !(should_log(MASK_LOG_IMU_FAST))) {
        AP::ins().Write_IMU();
    }

#if MODE_AUTOROTATE_ENABLED == ENABLED
    if (should_log(MASK_LOG_ATTITUDE_MED) || should_log(MASK_LOG_ATTITUDE_FAST)) {
        //update autorotation log
        g2.arot.Log_Write_Autorotation();
    }
#endif
#if HAL_GYROFFT_ENABLED
    if (should_log(MASK_LOG_FTN_FAST)) {
        gyro_fft.write_log_messages();
    }
#endif
}
#endif  // HAL_LOGGING_ENABLED

// three_hz_loop - 3hz loop
void Copter::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

    // check for deadreckoning failsafe
    failsafe_deadreckon_check();

    // update ch6 in flight tuning
    tuning();

    // check if avoidance should be enabled based on alt
    low_alt_avoidance();
}

// one_hz_loop - runs at 1Hz
void Copter::one_hz_loop()
{
#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap.value);
    }
#endif

    if (!motors->armed()) {
        update_using_interlock();

        // check the user hasn't updated the frame class or type
        motors->set_frame_class_and_type((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());

#if FRAME_CONFIG != HELI_FRAME
        // set all throttle channel settings
        motors->update_throttle_range();
#endif
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

#if HAL_LOGGING_ENABLED
    // log terrain data
    terrain_logging();
#endif

#if HAL_ADSB_ENABLED
    adsb.set_is_flying(!ap.land_complete);
#endif

    AP_Notify::flags.flying = !ap.land_complete;

    // slowly update the PID notches with the average loop rate
    attitude_control->set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
    pos_control->get_accel_z_pid().set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
#if AC_CUSTOMCONTROL_MULTI_ENABLED == ENABLED
    custom_control.set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
#endif
}

void Copter::init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

#if HAL_LOGGING_ENABLED
    // log the simple bearing
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
#endif
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    // �жϷ�����״̬
    if (simple_mode == SimpleMode::NONE || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;
    //ת������������ϵ
    if (simple_mode == SimpleMode::SIMPLE) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = channel_roll->get_control_in()*simple_cos_yaw - channel_pitch->get_control_in()*simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*simple_sin_yaw + channel_pitch->get_control_in()*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = channel_roll->get_control_in()*super_simple_cos_yaw - channel_pitch->get_control_in()*super_simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*super_simple_sin_yaw + channel_pitch->get_control_in()*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    channel_roll->set_control_in(rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw());
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void Copter::update_super_simple_bearing(bool force_update)
{
    if (!force_update) {
        if (simple_mode != SimpleMode::SUPERSIMPLE) {
            return;
        }
        if (home_distance() < SUPER_SIMPLE_RADIUS) {
            return;
        }
    }

    const int32_t bearing = home_bearing();

    // check the bearing to home has changed by at least 5 degrees
    if (labs(super_simple_last_bearing - bearing) < 500) {
        return;
    }

    super_simple_last_bearing = bearing;
    const float angle_rad = radians((super_simple_last_bearing+18000)/100);
    super_simple_cos_yaw = cosf(angle_rad);
    super_simple_sin_yaw = sinf(angle_rad);
}

void Copter::read_AHRS(void)
{
    // we tell AHRS to skip INS update as we have already done it in FAST_TASK.
    ahrs.update(true);
}

// read baro and log control tuning
void Copter::update_altitude()
{
    // read in baro altitude
    read_barometer();

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
        if (!should_log(MASK_LOG_FTN_FAST)) {
            AP::ins().write_notch_log_messages();
#if HAL_GYROFFT_ENABLED
            gyro_fft.write_log_messages();
#endif
        }
    }
#endif
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    distance = flightmode->wp_distance() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    bearing = flightmode->wp_bearing() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    xtrack_error = flightmode->crosstrack_error() * 0.01;
    return true;
}

// get the target earth-frame angular velocities in rad/s (Z-axis component used by some gimbals)
bool Copter::get_rate_ef_targets(Vector3f& rate_ef_targets) const
{
    // always returns zero vector if landed or disarmed
    if (copter.ap.land_complete) {
        rate_ef_targets.zero();
    } else {
        rate_ef_targets = attitude_control->get_rate_ef_targets();
    }
    return true;
}

void Copter::init_swarm()
{
    swarm_uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Swarm, 0);
    if (swarm_uart == nullptr)
    {
        return;
    }
    swarm_uart->begin(57600);
    swarm_is_leader = false;
    message_state = MessageState_WaitingForHeader;
}

void Copter::update_swarm_message()
{
    if (swarm_uart == nullptr)
    {
        return;
    }
    if (swarm_is_leader == false)
    {
        int16_t nbytes = swarm_uart->available();
        while (nbytes-- > 0)
        {
            char c = swarm_uart->read();
            switch (message_state) {

                default:
                case MessageState_WaitingForHeader:
                    if (c == AP_SWARM_HEADER) {
                        message_state = MessageState_WaitingForMsgId;
                        swarmbuf_len = 0;
                    }
                    break;

                case MessageState_WaitingForMsgId:
                    swarm_msg_id = c;
                    switch (swarm_msg_id) {
                    case AP_SWARM_MSGID_CONTENT:
                        message_state = MessageState_WaitingForLen;
                        break;
                    default:
                        // invalid message id
                        message_state = MessageState_WaitingForHeader;
                        break;
                    }
                    break;

                case MessageState_WaitingForLen:
                    swarm_msg_len = c;
                    if (swarm_msg_len > AP_SWARM_MSG_LEN_MAX) {
                        // invalid message length
                        message_state = MessageState_WaitingForHeader;
                    } else {
                        message_state = MessageState_WaitingForContents;
                    }
                    break;

                case MessageState_WaitingForContents:
                    // add to buffer
                    swarmbuf[swarmbuf_len++] = c;
                    if ((swarmbuf_len == swarm_msg_len) || (swarmbuf_len == sizeof(swarmbuf))) {
                        // process buffer
                        swarm_buffer();
                        // reset state for next message
                        message_state = MessageState_WaitingForHeader;
                    }
                    break;
            }
        }
    }
    else{
        hal.console->print("start_send");
        uint8_t send_buf[28];
        uint8_t crc = 0;
        Location send_loc;
        Vector3f send_vel;
        if (ahrs.get_location(send_loc) && ahrs.get_velocity_NED(send_vel))
        {
            send_buf[0] = AP_SWARM_HEADER;
            send_buf[1] = AP_SWARM_MSGID_CONTENT;
            crc ^= send_buf[1];
            send_buf[2] = 25;
            crc ^= send_buf[2];
            float data_F;
            int32_t data_I;
            data_I = send_loc.lat;
            // hal.console->printf("lat:%ld\n",data_I);
            memcpy(&send_buf[3], &data_I, sizeof(float));
            data_I = send_loc.lng;
            // hal.console->printf("lng:%ld\n",data_I);
            memcpy(&send_buf[7], &data_I, sizeof(float));
            data_I = send_loc.alt * 10UL;
            // hal.console->printf("alt:%ld\n",data_I);
            memcpy(&send_buf[11], &data_I, sizeof(float));
            data_F = send_vel.x * 100;
            // hal.console->printf("vx:%f\n",data_F);
            memcpy(&send_buf[15], &data_F, sizeof(float));
            data_F = send_vel.y * 100;
            // hal.console->printf("vy:%f\n",data_F);
            memcpy(&send_buf[19], &data_F, sizeof(float));
            data_F = send_vel.z * 100;
            // hal.console->printf("vz:%f\n",data_F);
            memcpy(&send_buf[23], &data_F, sizeof(float));

            for (uint8_t i=0; i<24; i++) {
                crc ^= send_buf[3+i];
            }
            send_buf[27] = crc;
            swarm_uart->write(send_buf,sizeof(send_buf));  
        }
        
    }
}

void Copter::swarm_buffer()
{
    uint8_t checksum = 0;
    checksum ^= swarm_msg_id;
    checksum ^= swarm_msg_len;
    for (uint8_t i=0; i<swarmbuf_len; i++) {
        checksum ^= swarmbuf[i];
    }
    // return if failed checksum check
    if (checksum != 0) {
        hal.console->printf("crc");
        return;
    }

    bool parsed = false;
    switch (swarm_msg_id) {

        case AP_SWARM_MSGID_CONTENT:
            {
                uint32_t out;
                out = ((uint32_t)swarmbuf[3] << 24 | (uint32_t)swarmbuf[2] << 16 | (uint32_t)swarmbuf[1] << 8 | (uint32_t)swarmbuf[0]);
                memcpy(&swarm_pos.x,&out,sizeof(int32_t));
                // hal.console->printf("lat:%ld\n",swarm_pos.x);
                out = ((uint32_t)swarmbuf[7] << 24 | (uint32_t)swarmbuf[6] << 16 | (uint32_t)swarmbuf[5] << 8 | (uint32_t)swarmbuf[4]);
                memcpy(&swarm_pos.y,&out,sizeof(int32_t));
                // hal.console->printf("lng:%ld\n",swarm_pos.y);
                out = ((uint32_t)swarmbuf[11] << 24 | (uint32_t)swarmbuf[10] << 16 | (uint32_t)swarmbuf[9] << 8 | (uint32_t)swarmbuf[8]);
                memcpy(&swarm_pos.z,&out,sizeof(int32_t));
                // hal.console->printf("alt:%ld\n",swarm_pos.z);
                out = ((uint32_t)swarmbuf[15] << 24 | (uint32_t)swarmbuf[14] << 16 | (uint32_t)swarmbuf[13] << 8 | (uint32_t)swarmbuf[12]);
                memcpy(&swarm_vel.x,&out,sizeof(float));
                // hal.console->printf("vx:%f\n",swarm_vel.x);
                out = ((uint32_t)swarmbuf[19] << 24 | (uint32_t)swarmbuf[18] << 16 | (uint32_t)swarmbuf[17] << 8 | (uint32_t)swarmbuf[16]);
                memcpy(&swarm_vel.y,&out,sizeof(float));
                // hal.console->printf("vy:%f\n",swarm_vel.y);
                out = ((uint32_t)swarmbuf[23] << 24 | (uint32_t)swarmbuf[22] << 16 | (uint32_t)swarmbuf[21] << 8 | (uint32_t)swarmbuf[20]);
                memcpy(&swarm_vel.z,&out,sizeof(float));
                // hal.console->printf("vz:%f\n",swarm_vel.z);
                parsed = true;
            }
            break;
        default:
            // unrecognised message id
            break;
    }
    
    // record success
    if (parsed) {
        swarm_update_ms = AP_HAL::millis();
    }
}

/*
  constructor for main Copter class
 */
Copter::Copter(void)
    :
#if HAL_LOGGING_ENABLED
    logger(g.log_bitmask),
#endif
    flight_modes(&g.flight_mode1),
    simple_cos_yaw(1.0f),
    super_simple_cos_yaw(1.0),
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    rc_throttle_control_in_filter(1.0f),
    inertial_nav(ahrs),
    param_loader(var_info),
    flightmode(&mode_stabilize),
    pos_variance_filt(FS_EKF_FILT_DEFAULT),
    vel_variance_filt(FS_EKF_FILT_DEFAULT),
    hgt_variance_filt(FS_EKF_FILT_DEFAULT)
{
}

Copter copter;
AP_Vehicle& vehicle = copter;

AP_HAL_MAIN_CALLBACKS(&copter);
