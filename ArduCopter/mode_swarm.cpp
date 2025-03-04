#include "Copter.h"

#if MODE_SWARM_ENABLED == ENABLED

bool ModeSwarm::init(const bool ignore_checks)
{
    if (copter.is_leader)
    {
        return false;
    }
    // re-use guided mode
    position_offset.x = 0;
    position_offset.y = 4;
    position_offset.z = 0;
    return ModeGuided::init(ignore_checks);
}
// perform cleanup required when leaving follow mode
void ModeSwarm::exit()
{
    g2.follow.clear_offsets_if_required();
}

void ModeSwarm::run()
{
    if (is_disarmed_or_landed()) {
        return;
    }
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    Vector3f desired_velocity_neu_cms;
    bool use_yaw = false;
    float yaw_cd = 0.0f;

    Vector3f dist_vec;  // vector to lead vehicle
    Vector3f dist_vec_offs;  // vector to lead vehicle + offset
    Vector3f vel_of_target;  // velocity of lead vehicle
    if (get_ned_target_dist_and_vel(dist_vec, dist_vec_offs, vel_of_target)) {
        // convert dist_vec_offs to cm in NEU
        const Vector3f dist_vec_offs_neu(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f, -dist_vec_offs.z * 100.0f);

        // calculate desired velocity vector in cm/s in NEU
        const float kp = 0.1;
        desired_velocity_neu_cms.x = (vel_of_target.x) + (dist_vec_offs_neu.x * kp);
        desired_velocity_neu_cms.y = (vel_of_target.y) + (dist_vec_offs_neu.y * kp);
        desired_velocity_neu_cms.z = (-vel_of_target.z) + (dist_vec_offs_neu.z * kp);//686879

        // scale desired velocity to stay within horizontal speed limit
        float desired_speed_xy = safe_sqrt(sq(desired_velocity_neu_cms.x) + sq(desired_velocity_neu_cms.y));
        if (!is_zero(desired_speed_xy) && (desired_speed_xy > pos_control->get_max_speed_xy_cms())) {
            const float scalar_xy = pos_control->get_max_speed_xy_cms() / desired_speed_xy;
            desired_velocity_neu_cms.x *= scalar_xy;
            desired_velocity_neu_cms.y *= scalar_xy;
            desired_speed_xy = pos_control->get_max_speed_xy_cms();
        }

        // limit desired velocity to be between maximum climb and descent rates
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -fabsf(pos_control->get_max_speed_down_cms()), pos_control->get_max_speed_up_cms());
        Vector2f desired_velocity_xy_cms(desired_velocity_neu_cms.x, desired_velocity_neu_cms.y);
        desired_velocity_neu_cms.x = desired_velocity_xy_cms.x;
        desired_velocity_neu_cms.y = desired_velocity_xy_cms.y;

        desired_velocity_neu_cms.z = 0;
    }
    else
    {
        desired_velocity_neu_cms.x = 0;
        desired_velocity_neu_cms.y = 0;
        desired_velocity_neu_cms.z = 0;
    }
    ModeGuided::set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false, false);

    ModeGuided::run();
}

bool ModeSwarm::get_ned_target_dist_and_vel(Vector3f &dist_ned, Vector3f &dist_with_offs, Vector3f &vel_ned)
{
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
         return false;
    }

    // get target location and velocity
    Location target_loc;
    Vector3f veh_vel;
    if (!get_target_loc_and_vel(target_loc, veh_vel)) {
        return false;
    }

    // change to altitude above home if relative altitude is being used
    if (target_loc.relative_alt == 1) {
        current_loc.alt -= AP::ahrs().get_home().alt;
    }

    // calculate difference
    const Vector3f dist_vec = current_loc.get_distance_NED(target_loc)+position_offset;
    // const Vector3f print_vec = current_loc.get_distance_NED(target_loc);
    // hal.console->printf("ned(%f,%f,%f)\n",print_vec.x,print_vec.y,print_vec.z);
    // hal.console->printf("vel(%f,%f,%f)\n",veh_vel.x,veh_vel.y,veh_vel.z);
    // fail if too far
    if (dist_vec.length() > 20) {
        return false;
    }

    Vector3f offsets(0,0,0);

    // calculate results
    wp_dist_swarm = safe_sqrt(sq(dist_vec.x) + sq(dist_vec.y)+sq(dist_vec.z));;
    dist_ned = dist_vec;
    dist_with_offs = dist_vec + offsets;
    vel_ned = veh_vel;
    return true;
}

bool ModeSwarm::get_target_loc_and_vel(Location &loc, Vector3f &vel_ned)
{

    // check for timeout
    if ((copter.swarm_update_ms == 0) || (AP_HAL::millis() - copter.swarm_update_ms > 1000)) {
        return false;
    }

    // calculate time since last actual position update
    // get velocity estimate
    // if (!get_velocity_ned(vel_ned, dt)) {
    //     return false;
    // }
    vel_ned = copter.swarm_vel;
    // project the vehicle position
    Location last_loc;
    last_loc.lat = copter.swarm_pos.x;
    last_loc.lng = copter.swarm_pos.y;
    last_loc.set_alt_cm(copter.swarm_pos.z / 10, Location::AltFrame::ABSOLUTE);
    last_loc.offset(0, 0);
    last_loc.alt -= 0; // convert m/s to cm/s, multiply by dt.  minus because NED

    // return latest position estimate
    loc = last_loc;
    return true;
}

uint32_t ModeSwarm::wp_distance() const
{
    return wp_dist_swarm * 100;
}

int32_t ModeSwarm::wp_bearing() const
{
    return 0;
}

/*
  get target position for mavlink reporting
 */
bool ModeSwarm::get_wp(Location &loc) const
{
    float dist = wp_dist_swarm;
    float bearing = 0;
    loc = copter.current_loc;
    loc.offset_bearing(bearing, dist);
    return true;
}

#endif // MODE_SWARM_ENABLED == ENABLED
