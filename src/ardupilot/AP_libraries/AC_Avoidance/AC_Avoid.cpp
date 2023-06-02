#include "AC_Avoid.h"

/// Constructor
AC_Avoid::AC_Avoid(const AP_AHRS& ahrs, const RangeFinder& rangefinder)
    : _ahrs(ahrs),
      _rangefinder(rangefinder)
{
    ;
}

void AC_Avoid::adjust_velocity(float kP, float accel_cmss, Vector2f &desired_vel)
{
    // exit immediately if disabled
    if (_enabled == AC_AVOID_DISABLED) {
        return;
    }

    // limit acceleration
    float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    adjust_velocity_proximity(kP, accel_cmss_limited, desired_vel);
}

// convenience function to accept Vector3f.  Only x and y are adjusted
void AC_Avoid::adjust_velocity(float kP, float accel_cmss, Vector3f &desired_vel)
{
    Vector2f des_vel_xy(desired_vel.x, desired_vel.y);
    adjust_velocity(kP, accel_cmss, des_vel_xy);
    desired_vel.x = des_vel_xy.x;
    desired_vel.y = des_vel_xy.y;
}

// adjust roll-pitch to push vehicle away from objects
// roll and pitch value are in centi-degrees
void AC_Avoid::adjust_roll_pitch(float &pitch, float veh_angle_max)
{
    // exit immediately if proximity based avoidance is disabled
    if (!_enabled) {
        return;
    }

    if (_rangefinder.status_orient(ROTATION_NONE) != RangeFinder::RangeFinder_Good) {
        return;
    }

    // exit immediately if angle max is zero
    if (_angle_max <= 0.0f || veh_angle_max <= 0.0f) {
        return;
    }

    float pitch_positive = 0.0f;   // maximum position pitch value
    float pitch_negative = 0.0f;   // minimum negative pitch value

    float distance_m = _rangefinder.distance_cm_orient(ROTATION_NONE);
    // get maximum positive and negative roll and pitch percentages from proximity sensor
    get_proximity_roll_pitch_pct(distance_m, pitch_positive, pitch_negative);

    // add maximum positive and negative percentages together for roll and pitch, convert to centi-degrees
    Vector2f rp_out(0.0f, (pitch_positive + pitch_negative) * 4500.0f);

    // apply avoidance angular limits
    // the object avoidance lean angle is never more than 75% of the total angle-limit to allow the pilot to override
    const float angle_limit = constrain_float(_angle_max, 0.0f, veh_angle_max * AC_AVOID_ANGLE_MAX_PERCENT);
    float vec_len = rp_out.length();
    if (vec_len > angle_limit) {
        rp_out *= (angle_limit / vec_len);
    }

    // add passed in roll, pitch angles
    rp_out.y += pitch;

    // apply total angular limits
    vec_len = rp_out.length();
    if (vec_len > veh_angle_max) {
        rp_out *= (veh_angle_max / vec_len);
    }

    // return adjusted roll, pitch
    pitch = rp_out.y;
}

/*
 * Adjusts the desired velocity based on output from the proximity sensor
 */
void AC_Avoid::adjust_velocity_proximity(float kP, float accel_cmss, Vector2f &desired_vel)
{
    // exit immediately if proximity sensor is not present
    if (_rangefinder.status_orient(ROTATION_NONE) != RangeFinder::RangeFinder_Good) {
        return;
    }

    // exit immediately if no desired velocity
    if (desired_vel.is_zero()) {
        return;
    }

    float distance_m = _rangefinder.distance_cm_orient(ROTATION_NONE);
    adjust_velocity_polygon(kP, accel_cmss, desired_vel, distance_m, false, _margin);
}

/*
 * Adjusts the desired velocity for the polygon fence.
 */
void AC_Avoid::adjust_velocity_polygon(float kP, float accel_cmss, Vector2f &desired_vel, const float distance_m, bool earth_frame, float margin)
{
    // Safe_vel will be adjusted to remain within fence.
    // We need a separate vector in case adjustment fails,
    // e.g. if we are exactly on the boundary.
    Vector2f safe_vel(desired_vel);

    // if boundary points are in body-frame, rotate velocity vector from earth frame to body-frame
    if (!earth_frame) {
        safe_vel.x = desired_vel.y * _ahrs.sin_yaw() + desired_vel.x * _ahrs.cos_yaw(); // right
        safe_vel.y = desired_vel.y * _ahrs.cos_yaw() - desired_vel.x * _ahrs.sin_yaw(); // forward
    }

    // calc margin in cm
    float margin_cm = MAX(margin * 100.0f, 0);

    Vector2f limit_direction = Vector2f(distance_m * 100.f, 0.0f);
    const float limit_distance = limit_direction.length();
    if (!is_zero(limit_distance)) {
        // We are strictly inside the given edge.
        // Adjust velocity to not violate this edge.
        limit_direction /= limit_distance;
        limit_velocity(kP, accel_cmss, safe_vel, limit_direction, MAX(limit_distance - margin_cm,0.0f));
    } else {
        // We are exactly on the edge - treat this as a fence breach.
        // i.e. do not adjust velocity.
        return;
    }

    // set modified desired velocity vector
    if (earth_frame) {
        desired_vel = safe_vel;
    } else {
        // if points were in body-frame, rotate resulting vector back to earth-frame
        desired_vel.x = safe_vel.x * _ahrs.cos_yaw() - safe_vel.y * _ahrs.sin_yaw();
        desired_vel.y = safe_vel.x * _ahrs.sin_yaw() + safe_vel.y * _ahrs.cos_yaw();
    }
}

/*
 * Limits the component of desired_vel in the direction of the unit vector
 * limit_direction to be at most the maximum speed permitted by the limit_distance.
 *
 * Uses velocity adjustment idea from Randy's second email on this thread:
 * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
 */
void AC_Avoid::limit_velocity(float kP, float accel_cmss, Vector2f &desired_vel, const Vector2f& limit_direction, float limit_distance) const
{
    const float max_speed = get_max_speed(kP, accel_cmss, limit_distance);
    // project onto limit direction
    const float speed = desired_vel * limit_direction;
    if (speed > max_speed) {
        // subtract difference between desired speed and maximum acceptable speed
        desired_vel += limit_direction*(max_speed - speed);
    }
}

/*
 * Computes the speed such that the stopping distance
 * of the vehicle will be exactly the input distance.
 */
float AC_Avoid::get_max_speed(float kP, float accel_cmss, float distance) const
{
    return AC_AttitudeControl::sqrt_controller(distance, kP, accel_cmss);
}

// convert distance (in meters) to a lean percentage (in 0~1 range) for use in manual flight modes
float AC_Avoid::distance_to_lean_pct(float dist_m)
{
    // ignore objects beyond DIST_MAX
    if (dist_m < 0.0f || dist_m >= _dist_max || _dist_max <= 0.0f) {
        return 0.0f;
    }
    // inverted but linear response
    return 1.0f - (dist_m / _dist_max);
}

// returns the maximum positive and negative roll and pitch percentages (in -1 ~ +1 range) based on the proximity sensor
void AC_Avoid::get_proximity_roll_pitch_pct(float distance_m, float &pitch_positive, float &pitch_negative)
{
    float dist_m;
    if (dist_m < _dist_max) {
        // convert distance to lean angle (in 0 to 1 range)
        const float lean_pct = distance_to_lean_pct(dist_m);
        // convert angle to roll and pitch lean percentages
        const float pitch_pct = 1.0f * lean_pct;
        // update roll, pitch maximums
        if (pitch_pct > 0.0f) {
            pitch_positive = MAX(pitch_positive, pitch_pct);
        }
        if (pitch_pct < 0.0f) {
            pitch_negative = MIN(pitch_negative, pitch_pct);
        }
    }
}
