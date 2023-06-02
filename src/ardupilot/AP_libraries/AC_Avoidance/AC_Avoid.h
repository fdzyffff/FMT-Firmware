#pragma once

#include "AP_Common.h"
#include "AP_Math.h"
#include "AP_AHRS.h"     // AHRS library
#include "AC_AttitudeControl.h"// Attitude controller library for sqrt controller
#include "AP_RangeFinder.h"

#define AC_AVOID_ACCEL_CMSS_MAX         100.0f  // maximum acceleration/deceleration in cm/s/s used to avoid hitting fence

// bit masks for enabled fence types.
#define AC_AVOID_DISABLED               0       // avoidance disabled
#define AC_AVOID_STOP_AT_FENCE          1       // stop at fence
#define AC_AVOID_USE_PROXIMITY_SENSOR   2       // stop based on proximity sensor output
#define AC_AVOID_ALL                    3       // use fence and promiximity sensor

// definitions for non-GPS avoidance
#define AC_AVOID_NONGPS_DIST_MAX_DEFAULT    10.0f   // objects over 10m away are ignored (default value for DIST_MAX parameter)
#define AC_AVOID_ANGLE_MAX_PERCENT          0.75f   // object avoidance max lean angle as a percentage (expressed in 0 ~ 1 range) of total vehicle max lean angle

/*
 * This class prevents the vehicle from leaving a polygon fence in
 * 2 dimensions by limiting velocity (adjust_velocity).
 */
class AC_Avoid {
public:

    /// Constructor
    AC_Avoid(const AP_AHRS& ahrs, const RangeFinder& rangefinder);

    /*
     * Adjusts the desired velocity so that the vehicle can stop
     * before the fence/object.
     * Note: Vector3f version is for convenience and only adjusts x and y axis
     */
    void adjust_velocity(float kP, float accel_cmss, Vector2f &desired_vel);
    void adjust_velocity(float kP, float accel_cmss, Vector3f &desired_vel);

    // adjust roll-pitch to push vehicle away from objects
    // roll and pitch value are in centi-degrees
    // angle_max is the user defined maximum lean angle for the vehicle in centi-degrees
    void adjust_roll_pitch(float &pitch, float angle_max);

    // parameters
    int8_t _enabled;
    int16_t _angle_max;        // maximum lean angle to avoid obstacles (only used in non-GPS flight modes)
    float _dist_max;         // distance (in meters) from object at which obstacle avoidance will begin in non-GPS modes
    float _margin;           // vehicle will attempt to stay this distance (in meters) from objects while in GPS modes
private:

    /*
     * Adjusts the desired velocity based on output from the proximity sensor
     */
    void adjust_velocity_proximity(float kP, float accel_cmss, Vector2f &desired_vel);

    /*
     * Adjusts the desired velocity given an array of boundary points
     *   earth_frame should be true if boundary is in earth-frame, false for body-frame
     *   margin is the distance (in meters) that the vehicle should stop short of the polygon
     */
    void adjust_velocity_polygon(float kP, float accel_cmss, Vector2f &desired_vel, const float distance_m, bool earth_frame, float margin);

    /*
     * Limits the component of desired_vel in the direction of the unit vector
     * limit_direction to be at most the maximum speed permitted by the limit_distance.
     *
     * Uses velocity adjustment idea from Randy's second email on this thread:
     * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
     */
    void limit_velocity(float kP, float accel_cmss, Vector2f &desired_vel, const Vector2f& limit_direction, float limit_distance) const;

    /*
     * Computes the speed such that the stopping distance
     * of the vehicle will be exactly the input distance.
     */
    float get_max_speed(float kP, float accel_cmss, float distance) const;

    /*
     * methods for avoidance in non-GPS flight modes
     */

    // convert distance (in meters) to a lean percentage (in 0~1 range) for use in manual flight modes
    float distance_to_lean_pct(float dist_m);

    // returns the maximum positive and negative roll and pitch percentages (in -1 ~ +1 range) based on the proximity sensor
    void get_proximity_roll_pitch_pct(float distance_m, float &pitch_positive, float &pitch_negative);

    // external references
    const AP_AHRS& _ahrs;
    const RangeFinder& _rangefinder;

};
