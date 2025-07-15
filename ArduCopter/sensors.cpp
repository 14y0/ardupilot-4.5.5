#include "Copter.h"

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;
}

void Copter::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
   rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
   rangefinder.init(ROTATION_PITCH_270);
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(g2.rangefinder_filt);
   rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);

   // upward facing range finder
   rangefinder_up_state.alt_cm_filt.set_cutoff_frequency(g2.rangefinder_filt);
   rangefinder_up_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_90);
#endif
}

//*MYP.S. 激光雷达滤波函数
float get_x1_weighted_average() {
    const float weights[10] = {0.30, 0.22, 0.16, 0.12, 0.10, 0.04, 0.02, 0.02, 0.01, 0.01};
    float sum = 0;
    float weight_sum = 0;
    int count = x1_history_full ? 10 : x1_index;
    for (int i = 0; i < count; i++) {
        // idx指向最新的、次新的...最老的数据
        int idx = (x1_index - i + 10) % 10;
        sum += x1_history[idx] * weights[i];
        weight_sum += weights[i];
    }
    if (weight_sum > 0) {
        return sum / weight_sum;
    } else {
        return 0;
    }
}

//*MYP.S. 激光雷达滤波函数
float get_x2_weighted_average() {
    const float weights[10] = {0.30, 0.22, 0.16, 0.12, 0.10, 0.04, 0.02, 0.02, 0.01, 0.01};
    float sum = 0;
    float weight_sum = 0;
    int count = x2_history_full ? 10 : x2_index;
    for (int i = 0; i < count; i++) {
        // idx指向最新的、次新的...最老的数据
        int idx = (x2_index - i + 10) % 10;
        sum += x2_history[idx] * weights[i];
        weight_sum += weights[i];
    }
    if (weight_sum > 0) {
        return sum / weight_sum;
    } else {
        return 0;
    }
}


// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

#if RANGEFINDER_TILT_CORRECTION == ENABLED
    const float tilt_correction = MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
#else
    const float tilt_correction = 1.0f;
#endif

    // iterate through downward and upward facing lidar
    struct {
        RangeFinderState &state;
        enum Rotation orientation;
    } rngfnd[3] = {{rangefinder_state, ROTATION_PITCH_270}, {rangefinder_up_state, ROTATION_PITCH_90},{rangefinder_front_state, ROTATION_NONE}};

    for (uint8_t i=0; i < ARRAY_SIZE(rngfnd); i++) {
        // local variables to make accessing simpler
        RangeFinderState &rf_state = rngfnd[i].state;
        enum Rotation rf_orient = rngfnd[i].orientation;

        // update health
        rf_state.alt_healthy = ((rangefinder.status_orient(rf_orient) == RangeFinder::Status::Good) &&
                                (rangefinder.range_valid_count_orient(rf_orient) >= RANGEFINDER_HEALTH_MAX));

        // tilt corrected but unfiltered, not glitch protected alt
        rf_state.alt_cm = tilt_correction * rangefinder.distance_cm_orient(rf_orient);
        z_value=tilt_correction * rangefinder.distance_cm_orient(ROTATION_PITCH_270);//*MYP.S. 给竖直距离赋值
        x1_value=ahrs.cos_pitch()*rangefinder.distance_cm_orient(ROTATION_NONE);//*MYP.S. yaw雷达1赋值
        x2_value=ahrs.cos_pitch()*rangefinder.distance_cm_orient(ROTATION_PITCH_90);//*MYP.S. yaw雷达2赋值

        //*MYP.S.新的x1_value计算出来后
        x1_history[x1_index] = x1_value;
        x1_index = (x1_index + 1) % 10;
        if (x1_index >= 9) x1_history_full = true;
        x1_filtered = get_x1_weighted_average();
        //*MYP.S.新的x2_value计算出来后
        x2_history[x2_index] = x2_value;
        x2_index = (x2_index + 1) % 10;
        if (x2_index >= 9) x2_history_full = true;
        x2_filtered = get_x2_weighted_average();

        // remember inertial alt to allow us to interpolate rangefinder
        rf_state.inertial_alt_cm = inertial_nav.get_position_z_up_cm();

        // glitch handling.  rangefinder readings more than RANGEFINDER_GLITCH_ALT_CM from the last good reading
        // are considered a glitch and glitch_count becomes non-zero
        // glitches clear after RANGEFINDER_GLITCH_NUM_SAMPLES samples in a row.
        // glitch_cleared_ms is set so surface tracking (or other consumers) can trigger a target reset
        const int32_t glitch_cm = rf_state.alt_cm - rf_state.alt_cm_glitch_protected;
        bool reset_terrain_offset = false;
        if (glitch_cm >= RANGEFINDER_GLITCH_ALT_CM) {
            rf_state.glitch_count = MAX(rf_state.glitch_count+1, 1);
        } else if (glitch_cm <= -RANGEFINDER_GLITCH_ALT_CM) {
            rf_state.glitch_count = MIN(rf_state.glitch_count-1, -1);
        } else {
            rf_state.glitch_count = 0;
            rf_state.alt_cm_glitch_protected = rf_state.alt_cm;
        }
        if (abs(rf_state.glitch_count) >= RANGEFINDER_GLITCH_NUM_SAMPLES) {
            // clear glitch and record time so consumers (i.e. surface tracking) can reset their target altitudes
            rf_state.glitch_count = 0;
            rf_state.alt_cm_glitch_protected = rf_state.alt_cm;
            rf_state.glitch_cleared_ms = AP_HAL::millis();
            reset_terrain_offset = true;
        }

        // filter rangefinder altitude
        uint32_t now = AP_HAL::millis();
        const bool timed_out = now - rf_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS;
        if (rf_state.alt_healthy) {
            if (timed_out) {
                // reset filter if we haven't used it within the last second
                rf_state.alt_cm_filt.reset(rf_state.alt_cm);
                reset_terrain_offset = true;

            } else {
                rf_state.alt_cm_filt.apply(rf_state.alt_cm, 0.05f);
            }
            rf_state.last_healthy_ms = now;
        }

        // handle reset of terrain offset
        if (reset_terrain_offset) {
            if (rf_orient == ROTATION_PITCH_90) {
                // upward facing
                rf_state.terrain_offset_cm = rf_state.inertial_alt_cm + rf_state.alt_cm;
            } else {
                // assume downward facing
                rf_state.terrain_offset_cm = rf_state.inertial_alt_cm - rf_state.alt_cm;
            }
        }

        // send downward facing lidar altitude and health to the libraries that require it
#if HAL_PROXIMITY_ENABLED
        if (rf_orient == ROTATION_PITCH_270) {
            if (rangefinder_state.alt_healthy || timed_out) {
                g2.proximity.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
            }
        }
#endif
    }

#else
    // downward facing rangefinder
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;

    // upward facing rangefinder
    rangefinder_up_state.enabled = false;
    rangefinder_up_state.alt_healthy = false;
    rangefinder_up_state.alt_cm = 0;
#endif
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok() const
{
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_up_ok() const
{
    return (rangefinder_up_state.enabled && rangefinder_up_state.alt_healthy);
}

// update rangefinder based terrain offset
// terrain offset is the terrain's height above the EKF origin
void Copter::update_rangefinder_terrain_offset()
{
    float terrain_offset_cm = rangefinder_state.inertial_alt_cm - rangefinder_state.alt_cm_glitch_protected;
    rangefinder_state.terrain_offset_cm += (terrain_offset_cm - rangefinder_state.terrain_offset_cm) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    terrain_offset_cm = rangefinder_up_state.inertial_alt_cm + rangefinder_up_state.alt_cm_glitch_protected;
    rangefinder_up_state.terrain_offset_cm += (terrain_offset_cm - rangefinder_up_state.terrain_offset_cm) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    if (rangefinder_state.alt_healthy || (AP_HAL::millis() - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS)) {
        wp_nav->set_rangefinder_terrain_offset(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.terrain_offset_cm);
#if MODE_CIRCLE_ENABLED
        circle_nav->set_rangefinder_terrain_offset(rangefinder_state.enabled && wp_nav->rangefinder_used(), rangefinder_state.alt_healthy, rangefinder_state.terrain_offset_cm);
#endif
    }
}

/*
  get inertially interpolated rangefinder height. Inertial height is
  recorded whenever we update the rangefinder height, then we use the
  difference between the inertial height at that time and the current
  inertial height to give us interpolation of height from rangefinder
 */
bool Copter::get_rangefinder_height_interpolated_cm(int32_t& ret) const
{
    if (!rangefinder_alt_ok()) {
        return false;
    }
    ret = rangefinder_state.alt_cm_filt.get();
    float inertial_alt_cm = inertial_nav.get_position_z_up_cm();
    ret += inertial_alt_cm - rangefinder_state.inertial_alt_cm;
    return true;
}
