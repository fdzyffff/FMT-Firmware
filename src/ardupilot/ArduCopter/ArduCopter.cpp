#include "Copter.h"

void Copter::setup() 
{
    init_ardupilot();
    fast_loopTimer = micro64();
}


void Copter::loop()
{
    uint64_t timer = micro64();

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.0f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    rc_loop();
    fast_loop();
    throttle_loop();
}


// Main loop - 400hz
void Copter::fast_loop()
{    
    // run low level rate controllers that only require IMU data
    attitude_control->rate_controller_run();

    // send outputs to the motors library immediately
    motors_output();

    // run EKF state estimator (expensive)
    // --------------------
    read_AHRS(); // hack

    // Inertial Nav
    // --------------------
    read_inertia(); // keep

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've landed or crashed
    update_land_and_crash_detectors();

    update_throttle_hover();
}

// rc_loops - reads user input from transmitter/receiver
// called at 400hz
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();
}

// throttle_loop - should be run at 400 hz
// ---------------------------
void Copter::throttle_loop()
{
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_thr_mix();

    // check auto_armed status
    update_auto_armed();
}

// ten_hz_loop - runs at 10Hz
void Copter::ten_hz_loop()
{
    if (!motors->armed()) {

        update_using_interlock();

        // check the user hasn't updated the frame class or type
        motors->set_frame_class_and_type((AP_Motors::motor_frame_class)g2.frame_class, (AP_Motors::motor_frame_type)g.frame_type);

        // set all throttle channel settings
        motors->set_throttle_range(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();
    

    //auto_disarm_check();
    //arm_motors_check();
    run_nav_updates();
    
    test_star_updates();
}

void Copter::read_AHRS(void)
{
    // we tell AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
}

// checks if we should update ahrs/RTL home position from the EKF
void Copter::update_home_from_EKF()
{
    // exit immediately if home already set
    if (ap.home_state != HOME_UNSET) {
        return;
    }

    // special logic if home is set in-flight
    if (motors->armed()) {
        set_home_to_current_location_inflight();
    } else {
        // move home to current ekf location (this will set home_state to HOME_SET)
        set_home_to_current_location();
    }
}
