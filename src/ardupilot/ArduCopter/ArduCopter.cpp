#include "Copter.h"

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Copter, copter, func, rate_hz, max_time_micros)


void Copter_pre::setup()
{
    // static_cast<Copter&>(copter).setup();
}

void Copter_pre::loop()
{
    // static_cast<Copter&>(copter).loop();
}

void Copter_pre::rc_loop()
{
    // static_cast<Copter&>(copter).rc_loop();
}

// void Copter_pre::throttle_loop()
// {
//     static_cast<Copter&>(copter).throttle_loop();
// }


void Copter::setup() 
{
/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
    static const AP_Scheduler::Task copter_scheduler_tasks[] = {
        SCHED_TASK(update_throttle_hover,  100,    90),
        SCHED_TASK(throttle_loop,          50,     75),
        SCHED_TASK(arm_motors_check,       10,     50),
        SCHED_TASK(run_nav_updates,        50,    100),
        SCHED_TASK(ten_hz_loop,            10,    100),
        SCHED_TASK(one_hz_loop,            1,     100),
    };
    scheduler.init(&copter_scheduler_tasks[0], ARRAY_SIZE(copter_scheduler_tasks));

    init_ardupilot();
    fast_loopTimer = micro64();
}


void Copter::loop()
{
    // printf(" wp_navalt_min: %f\n",g2.wp_navalt_min);
    // printf(" test_value_p1: %f\n",test_value_p1);
    uint64_t timer = micro64();
    // printf(" This is APM LOOP 1\n");

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.0f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - micro64();
    scheduler.run(time_available > MAIN_LOOP_MICROS ? 0u : time_available);
}


// Main loop - 500hz
void Copter::fast_loop()
{    

    rc_loop();

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

    update_gcs_cmd();

    update_fmt_bus();
}

// rc_loops - reads user input from transmitter/receiver
// called at 400hz
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();
    // printf(" rc_loop millis() %ld,\n", millis());
}

// throttle_loop - should be run at 50 hz
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
}

// one_hz_loop - runs at 10Hz
void Copter::one_hz_loop()
{
    // console_printf(" hal.sitl_state.altitude: %f\n", hal.sitl_state.altitude);
    // printf(" hal.rcin._rc_in_data[0,1,2,3,4]: [%d,%d,%d,%d,%d]\n", hal.rcin._rc_in_data[0],hal.rcin._rc_in_data[1],hal.rcin._rc_in_data[2],hal.rcin._rc_in_data[3],hal.rcin._rc_in_data[4]);
    // printf(" hal.rcout._rc_out_data[0,1,2,3]: [%d,%d,%d,%d]\n", hal.rcout._rc_out_data[0],hal.rcout._rc_out_data[1],hal.rcout._rc_out_data[2],hal.rcout._rc_out_data[3]);
    // printf(" channel_throttle->get_radio_min(): %d\n", channel_throttle->get_radio_min());
    // printf(" motors->get_pwm_output_min(): %d\n", motors->get_pwm_output_min());
    // printf(" ap.rc_receiver_present: %d\n", ap.rc_receiver_present);
    // console_printf(" copter->g2.frame_class: %d\n", copter->g2.frame_class);

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

void Copter::update_gcs_cmd() 
{
    static uint8_t last_ctrl_mode = 0;
    static uint8_t last_state_cmd = 0;
    if (hal.apm_gcs_cmd_updated) {
        printf("mode:%d, submod:%d \n",hal.gcs_cmd_msg.mode, hal.gcs_cmd_msg.cmd_1);
        if (last_state_cmd != hal.gcs_cmd_msg.cmd_1) {
            switch (hal.gcs_cmd_msg.cmd_1)
            {
                case FMS_Cmd_PreArm:
                    printf ("Do PreArm\n");
                    init_arm_motors(true);
                    break;
                case FMS_Cmd_Arm:
                    printf ("[no]Do Arm\n");
                    break;
                case FMS_Cmd_Disarm:
                    printf ("Do Disarm\n");
                    init_disarm_motors();
                    break;
                case FMS_Cmd_Takeoff:
                    printf ("[no]Do Takeoff\n");
                    break;
                case FMS_Cmd_Land:
                    printf ("Do Land\n");
                    set_mode(LAND, MODE_REASON_GCS_COMMAND);
                    break;
                case FMS_Cmd_Return:
                    printf ("Do RTL\n");
                    set_mode(RTL, MODE_REASON_GCS_COMMAND);
                    break;
                case FMS_Cmd_Pause:
                    printf ("[no]Do Pause\n");
                    break;
                case FMS_Cmd_Continue:
                    printf ("[no]Do Continue\n");
                    break;
                case FMS_Cmd_None:
                default:
                    break;
            }
            last_state_cmd = hal.gcs_cmd_msg.cmd_1;
        }

        if (last_ctrl_mode != hal.gcs_cmd_msg.mode) {
            switch (hal.gcs_cmd_msg.mode)
            {
                default:
                    printf ("Invalid mode\n");
                    break;
                case PilotMode_Acro:
                    printf ("[no]APM Acro mode\n");
                    // set_mode(ACRO, MODE_REASON_GCS_COMMAND);
                    break;
                case PilotMode_Stabilize:
                    printf ("APM Stabilize mode\n");
                    set_mode(STABILIZE, MODE_REASON_GCS_COMMAND);
                    break;
                case PilotMode_Altitude:
                    printf ("APM Alt_hold mode\n");
                    set_mode(ALT_HOLD, MODE_REASON_GCS_COMMAND);
                    break;
                case PilotMode_Position:
                    printf ("APM Loiter mode\n");
                    set_mode(LOITER, MODE_REASON_GCS_COMMAND);
                    break;
                case PilotMode_Mission:
                    printf ("APM Auto mode\n");
                    set_mode(AUTO, MODE_REASON_GCS_COMMAND);
                    break;        
            }
            last_ctrl_mode = hal.gcs_cmd_msg.mode;
        }
        hal.apm_gcs_cmd_updated = 0;
        hal.apm_gcs_cmd_log = 1;
    }
}

void Copter::update_fmt_bus() 
{
    // for FMS_OUT_BUS
    if (motors->armed()) {
        if (!ap.land_complete) {
            hal.fms_out_msg.status = VehicleStatus_Arm;
        } else {
            hal.fms_out_msg.status = VehicleStatus_Standby;
        }
    } else {
        hal.fms_out_msg.status = VehicleStatus_Disarm;
    }
    hal.fms_out_msg.ctrl_mode = control_mode;
    switch (control_mode) {
        case AUTO:
            hal.fms_out_msg.mode = PilotMode_Mission;
            switch (auto_mode) {
                case Auto_TakeOff:
                    hal.fms_out_msg.state = VehicleState_Takeoff;
                    break;
                case Auto_WP:
                case Auto_CircleMoveToEdge:
                case Auto_Circle:
                case Auto_Spline:
                    hal.fms_out_msg.state = VehicleState_Mission;
                    break;
                case Auto_Land:
                    hal.fms_out_msg.state = VehicleState_Land;
                    break;
                case Auto_RTL:
                    hal.fms_out_msg.state = VehicleState_Return;
                    break;
                case Auto_NavGuided:
                    hal.fms_out_msg.state = VehicleState_Offboard;
                    break;
                case Auto_Loiter:
                    hal.fms_out_msg.state = VehicleState_Hold;
                    break;
                default:
                    hal.fms_out_msg.state = VehicleState_Hold;
                    break;
            }
            break;
        case STABILIZE:
            hal.fms_out_msg.mode = PilotMode_Stabilize;
            hal.fms_out_msg.state = VehicleState_Stabilize;
            break;
        case ALT_HOLD:
            hal.fms_out_msg.mode = PilotMode_Altitude;
            hal.fms_out_msg.state = VehicleState_Altitude;
            break;
        case LOITER:
            hal.fms_out_msg.mode = PilotMode_Position;
            hal.fms_out_msg.state = VehicleState_Position;
            break;
        case RTL:
            hal.fms_out_msg.mode = PilotMode_Mission;
            hal.fms_out_msg.state = VehicleState_Return;
            break;
        case LAND:
            hal.fms_out_msg.mode = PilotMode_Mission;
            hal.fms_out_msg.state = VehicleState_Land;
            break;
        case ACRO:
            hal.fms_out_msg.mode = PilotMode_Acro;
            hal.fms_out_msg.state = VehicleState_Acro;
            break;
        case GUIDED:
            hal.fms_out_msg.mode = PilotMode_Offboard;
            hal.fms_out_msg.state = VehicleState_Offboard;
            break;
        default:
            hal.fms_out_msg.mode = PilotMode_None;
            hal.fms_out_msg.state = VehicleState_None;
            break;
    }

    // hal.fms_out_msg.reserved1;
    for (uint8_t i = 0; i < 16; i++){
        hal.fms_out_msg.actuator_cmd[i] = hal.control_out_msg.actuator_cmd[i];
    }

}

