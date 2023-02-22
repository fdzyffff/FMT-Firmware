#include "Copter.h"

/*
 * Init and run calls for althold, flight mode
 */
// flowhold_init - initialise flowhold controller
bool Copter::flowhold_init(bool ignore_checks)
{
// #if FRAME_CONFIG == HELI_FRAME
//     // do not allow helis to enter Flow Hold if the Rotor Runup is not complete
//     if (!ignore_checks && !motors->rotor_runup_complete()){
//         return false;
//     }
// #endif

    if (!optflow.enabled() || !optflow.healthy()) {
        console_printf("Fail Flow-hold Mode\n");
        return false;
    }

    // initialize vertical speeds and leash lengths
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    flowhold_t.flow_filter.set_cutoff_frequency(scheduler.get_loop_rate_hz(), flowhold_t.flow_filter_hz);

    flowhold_t.quality_filtered = 0;
    flowhold_t.flow_pi_xy.reset_I();
    flowhold_t.limited = false;

    flowhold_t.flow_pi_xy.set_dt(1.0/scheduler.get_loop_rate_hz());

    // start with INS height
    // flowhold_t.last_ins_height = inertial_nav.get_altitude() * 0.01;
    // flowhold_t.height_offset = 0;

    console_printf("Flow-hold Mode\n");

    return true;
}

/*
  calculate desired attitude from flow sensor. Called when flow sensor is healthy
 */
void Copter::flowhold_flow_to_angle(Vector2f &bf_angles, float rngfnd_height, bool stick_input)
{
    uint32_t now = millis();

    // get corrected raw flow rate
    Vector2f raw_flow = optflow.flowRate() - optflow.bodyRate();

    // limit sensor flow, this prevents oscillation at low altitudes
    raw_flow.x = constrain_float(raw_flow.x, -flowhold_t.flow_max, flowhold_t.flow_max);
    raw_flow.y = constrain_float(raw_flow.y, -flowhold_t.flow_max, flowhold_t.flow_max);

    // filter the flow rate
    Vector2f sensor_flow = flowhold_t.flow_filter.apply(raw_flow);

    // compensate for height, this converts to (approx) m/s
    sensor_flow *= rngfnd_height;

    // rotate controller input to earth frame
    Vector2f input_ef = ahrs.rotate_body_to_earth2D(sensor_flow);

    // run PI controller
    flowhold_t.flow_pi_xy.set_input(input_ef);

    // get earth frame controller attitude in centi-degrees
    Vector2f ef_output;

    // get P term
    ef_output = flowhold_t.flow_pi_xy.get_p();

    if (stick_input) {
        flowhold_t.last_stick_input_ms = now;
        flowhold_t.braking = true;
    }
    if (!stick_input && flowhold_t.braking) {
        // stop flowhold_t.braking if either 3s has passed, or we have slowed below 0.3m/s
        if (now - flowhold_t.last_stick_input_ms > 3000 || sensor_flow.length() < 0.3) {
            flowhold_t.braking = false;
#if 0
            printf("flowhold_t.braking done at %u vel=%f\n", now - flowhold_t.last_stick_input_ms,
                   (double)sensor_flow.length());
#endif
        }
    }

    if (!stick_input && !flowhold_t.braking) {
        // get I term
        if (flowhold_t.limited) {
            // only allow I term to shrink in length
            flowhold_t.xy_I = flowhold_t.flow_pi_xy.get_i_shrink();
        } else {
            // normal I term operation
            flowhold_t.xy_I = flowhold_t.flow_pi_xy.get_pi();
        }
    }

    if (!stick_input && flowhold_t.braking) {
        // calculate brake angle for each axis separately
        if (true) {
            float velocity = sensor_flow.x;
            float abs_vel_cms = fabsf(velocity)*100;
            const float brake_gain = (15.0f * flowhold_t.brake_rate_dps + 95.0f) / 100.0f;
            float lean_angle_cd = brake_gain * abs_vel_cms * (1.0f+500.0f/(abs_vel_cms+60.0f));
            if (velocity < 0) {
                lean_angle_cd = -lean_angle_cd;
            }
            bf_angles.x = lean_angle_cd;
        }
        if (true) {
            float velocity = sensor_flow.y;
            float abs_vel_cms = fabsf(velocity)*100;
            const float brake_gain = (15.0f * flowhold_t.brake_rate_dps + 95.0f) / 100.0f;
            float lean_angle_cd = brake_gain * abs_vel_cms * (1.0f+500.0f/(abs_vel_cms+60.0f));
            if (velocity < 0) {
                lean_angle_cd = -lean_angle_cd;
            }
            bf_angles.y = lean_angle_cd;
        }
        ef_output.zero();
    }

    ef_output += flowhold_t.xy_I;
    ef_output *= aparm.angle_max;

    // convert to body frame
    bf_angles += ahrs.rotate_earth_to_body2D(ef_output);

    // set flowhold_t.limited flag to prevent integrator windup
    flowhold_t.limited = fabsf(bf_angles.x) > aparm.angle_max || fabsf(bf_angles.y) > aparm.angle_max;

    // constrain to angle limit
    bf_angles.x = constrain_float(bf_angles.x, -aparm.angle_max, aparm.angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -aparm.angle_max, aparm.angle_max);
}

// flowhold_run - runs the flowhold controller
// should be called at 100hz or more
void Copter::flowhold_run()
{
    FlowHoldModeState flowhold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // check for filter change
    if (!is_equal(flowhold_t.flow_filter.get_cutoff_freq(), flowhold_t.flow_filter_hz)) {
        flowhold_t.flow_filter.set_cutoff_frequency(flowhold_t.flow_filter_hz);
    }

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = apm_constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);

    if (!motors->armed() || !motors->get_interlock()) {
        flowhold_state = FlowHold_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        flowhold_state = FlowHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        flowhold_state = FlowHold_Landed;
    } else {
        flowhold_state = FlowHold_Flying;
    }

    if (optflow.healthy()) {
        const float filter_constant = 0.95;
        flowhold_t.quality_filtered = filter_constant * flowhold_t.quality_filtered + (1-filter_constant) * optflow.quality();
    } else {
        flowhold_t.quality_filtered = 0;
    }

    Vector2f bf_angles;

    // calculate alt-hold angles
    int16_t roll_in = channel_roll->get_control_in();
    int16_t pitch_in = channel_pitch->get_control_in();
    float angle_max = attitude_control->get_althold_lean_angle_max();
    flowhold_get_desired_lean_angles(bf_angles.x, bf_angles.y, angle_max, attitude_control->get_althold_lean_angle_max());

    if (flowhold_t.quality_filtered >= flowhold_t.flow_min_quality &&
        millis() - arm_time_ms > 3000 &&
        rangefinder_alt_ok()) {
        // don't use for first 3s when we are just taking off
        Vector2f flow_angles;

        flowhold_flow_to_angle(flow_angles, 0.01f*(float)rangefinder_state.alt_cm, (roll_in != 0) || (pitch_in != 0));
        flow_angles.x = constrain_float(flow_angles.x, -angle_max/2, angle_max/2);
        flow_angles.y = constrain_float(flow_angles.y, -angle_max/2, angle_max/2);
        bf_angles += flow_angles;
    }
    bf_angles.x = constrain_float(bf_angles.x, -angle_max, angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -angle_max, angle_max);

    // Flow Hold State Machine
    switch (flowhold_state) {

    case FlowHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x, bf_angles.y, target_yaw_rate, get_smoothing_gain());
// #if FRAME_CONFIG == HELI_FRAME    
//         // force descent rate and call position controller
//         pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
// #else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
// #endif
        flowhold_t.flow_pi_xy.reset_I();
        pos_control->update_z_controller();
        break;

    case FlowHold_Takeoff:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(apm_constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x, bf_angles.y, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();

        break;

    case FlowHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x, bf_angles.y, target_yaw_rate, get_smoothing_gain());
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case FlowHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

// #if AC_AVOID_ENABLED == ENABLED
//         // apply avoidance
//         avoid.adjust_roll_pitch(bf_angles.x, bf_angles.y, aparm.angle_max);
// #endif

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x, bf_angles.y, target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void Copter::flowhold_get_desired_lean_angles(float &roll_out, float &pitch_out, float angle_max, float angle_limit)
{
    // fetch roll and pitch inputs
    roll_out = channel_roll->get_control_in();
    pitch_out = channel_pitch->get_control_in();

    // limit max lean angle
    angle_limit = constrain_float(angle_limit, 1000.0f, angle_max);

    // scale roll and pitch inputs to ANGLE_MAX parameter range
    float scaler = angle_max/(float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_out *= scaler;
    pitch_out *= scaler;

    // do circular limit
    float total_in = norm(pitch_out, roll_out);
    if (total_in > angle_limit) {
        float ratio = angle_limit / total_in;
        roll_out *= ratio;
        pitch_out *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_out = (18000/M_PI) * atanf(cosf(pitch_out*(M_PI/18000))*tanf(roll_out*(M_PI/18000)));

    // roll_out and pitch_out are returned
}
