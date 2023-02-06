#include "Copter.h"
#include "APM.h"

// extern uint32_t APM_millis(void);
// extern APM_Params_t apm_params;

Copter* copter;
AP_HAL hal;
my_temp_log_t my_temp_log;

// int16_t RC_in_data[20];
// int16_t RC_out_data[20];

uint32_t millis() {return systime_now_ms();}
uint64_t micro64() {return systime_now_us();} 

extern "C" {

void APM_Copter_Init(void)  //飞控初始化
{
    // Copter copter_temp;
    printf("----------------------------------------------------------------------------------------------------\n");
    printf("                                               APM init                                             \n");
    printf("----------------------------------------------------------------------------------------------------\n");
    copter = new Copter();
    hal =*(new AP_HAL());
}

void APM_Copter_Setup(void)  //飞控初始化
{
    copter->setup();
    copter->test_value_p1 = 100.f;
    // memset(RC_in_data, 0, sizeof(RC_in_data));
    // memset(RC_out_data, 0, sizeof(RC_out_data));
    printf("----------------------------------------------------------------------------------------------------\n");
}

void APM_Copter_Main(void)  //飞控主循环，不小于400Hz
{
    // param_set_val(param_get_by_full_name("APM","USER_TEST_P1"), &tmp_1);

    memcpy(&hal.rcChannel_msg,&rcChannel_msg,sizeof(rcChannel_msg));
    memcpy(&hal.ins_out_msg,&ins_out_msg,sizeof(ins_out_msg));
    memcpy(&hal.mission_data_msg,&mission_data_msg,sizeof(mission_data_msg));
    memcpy(&hal.gcs_cmd_msg,&gcs_cmd_msg,sizeof(gcs_cmd_msg));

    hal.apm_pilot_cmd_updated = apm_pilot_cmd_updated;
    hal.apm_gcs_cmd_updated = apm_gcs_cmd_updated;
    hal.apm_mission_data_updated = apm_mission_data_updated;

    // update hal
    hal.update();

    // the Copter main loop, includes fast_loop and loops defined in ArduCopter.cpp by AP_scheduler
    copter->loop();

    // output
    memcpy(&fms_out_msg,&hal.fms_out_msg,sizeof(fms_out_msg));
    memcpy(&control_out_msg,&hal.control_out_msg,sizeof(control_out_msg));

    apm_pilot_cmd_updated = hal.apm_pilot_cmd_updated;
    apm_gcs_cmd_updated = hal.apm_gcs_cmd_updated;
    apm_mission_data_updated = hal.apm_mission_data_updated;

    apm_pilot_cmd_log = hal.apm_pilot_cmd_log;
    apm_gcs_cmd_log = hal.apm_gcs_cmd_log;
    apm_mission_data_log = hal.apm_mission_data_log;
}

void APM_Copter_Init_Para_P1(void)
{
    FMT_CHECK(param_link_variable(PARAM_GET(APM, USER_TEST_P1),       &copter->test_value_p1));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, CH7_OPT),            &copter->g.ch7_option));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, CH8_OPT),            &copter->g.ch8_option));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, CH9_OPT),            &copter->g.ch9_option));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, CH10_OPT),           &copter->g.ch10_option));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, CH11_OPT),           &copter->g.ch11_option));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, CH12_OPT),           &copter->g.ch12_option));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, DISARM_DELAY),       &copter->g.disarm_delay));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, FLTMODE1),           &copter->g.flight_mode1));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, FLTMODE2),           &copter->g.flight_mode2));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, FLTMODE3),           &copter->g.flight_mode3));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, FLTMODE4),           &copter->g.flight_mode4));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, FLTMODE5),           &copter->g.flight_mode5));
    // FMT_CHECK(param_link_variable(PARAM_GET(APM, FLTMODE6),           &copter->g.flight_mode6));
    // FMT_CHECK(param_link_variable(PARAM_GET(APM, FLOW_ENABLE),        &copter->optflow._enabled));
    // FMT_CHECK(param_link_variable(PARAM_GET(APM, FLOW_FXSCALER),      &copter->optflow._flowScalerX));
    // FMT_CHECK(param_link_variable(PARAM_GET(APM, FLOW_FYSCALER),      &copter->optflow._flowScalerY));
    // FMT_CHECK(param_link_variable(PARAM_GET(APM, FLOW_ORIENT_YAW),    &copter->optflow._yawAngle_cd));
    // FMT_CHECK(param_link_variable(PARAM_GET(APM, FLOW_POS_X),         &copter->optflow._pos_offset.x));
    // FMT_CHECK(param_link_variable(PARAM_GET(APM, FLOW_POS_Y),         &copter->optflow._pos_offset.y));
    // FMT_CHECK(param_link_variable(PARAM_GET(APM, FLOW_POS_Z),         &copter->optflow._pos_offset.z));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, FRAME_CLASS),        &copter->g2.frame_class));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, FRAME_TYPE),         &copter->g.frame_type));
    // PARAM_FLOAT(FS_BATT_ENABLE, 0),
    // PARAM_FLOAT(FS_BATT_ENABLE2, 0),
    // PARAM_FLOAT(FS_BATT_MAH, 0),
    // PARAM_FLOAT(FS_BATT_VOLT2, 10.5),
    // PARAM_FLOAT(FS_BATT_VOLTAGE, 38.31),
    FMT_CHECK(param_link_variable(PARAM_GET(APM, FS_CRASH_CHECK),     &copter->g.fs_crash_check));
    // FMT_CHECK(param_link_variable(PARAM_GET(APM, FS_GCS_ENABLE),      &copter->g.failsafe_gcs));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, FS_THR_ENABLE),      &copter->g.failsafe_throttle));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, FS_THR_VALUE),       &copter->g.failsafe_throttle_value));
    // FMT_CHECK(param_link_variable(PARAM_GET(APM, MIS_RESTART),        &copter->mission._restart));
    // FMT_CHECK(param_link_variable(PARAM_GET(APM, MIS_TOTAL),          &copter->mission._cmd_total));
    // FMT_CHECK(param_link_variable(PARAM_GET(APM, PHLD_BRAKE_ANGLE),   &copter->g.poshold_brake_angle_max));
    // FMT_CHECK(param_link_variable(PARAM_GET(APM, PHLD_BRAKE_RATE),    &copter->g.poshold_brake_rate));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, PILOT_ACCEL_Z),      &copter->g.pilot_accel_z));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, PILOT_THR_BHV),      &copter->g.throttle_behavior));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, PILOT_THR_FILT),     &copter->g.throttle_filt));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, PILOT_TKOFF_ALT),    &copter->g.pilot_takeoff_alt));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, PILOT_TKOFF_DZ),     &copter->g.takeoff_trigger_dz));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, PILOT_VELZ_MAX),     &copter->g.pilot_velocity_z_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RC_FEEL_RP),         &copter->g.rc_feel_rp));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO1_DZ),          &copter->g2.rc_channels.rc_channel(0)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO1_MAX),         &copter->g2.rc_channels.rc_channel(0)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO1_MIN),         &copter->g2.rc_channels.rc_channel(0)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO1_REVERSED),    &copter->g2.rc_channels.rc_channel(0)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO1_TRIM),        &copter->g2.rc_channels.rc_channel(0)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO2_DZ),          &copter->g2.rc_channels.rc_channel(2)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO2_MAX),         &copter->g2.rc_channels.rc_channel(2)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO2_MIN),         &copter->g2.rc_channels.rc_channel(2)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO2_REVERSED),    &copter->g2.rc_channels.rc_channel(2)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO2_TRIM),        &copter->g2.rc_channels.rc_channel(2)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO3_DZ),          &copter->g2.rc_channels.rc_channel(2)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO3_MAX),         &copter->g2.rc_channels.rc_channel(2)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO3_MIN),         &copter->g2.rc_channels.rc_channel(2)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO3_REVERSED),    &copter->g2.rc_channels.rc_channel(2)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO3_TRIM),        &copter->g2.rc_channels.rc_channel(2)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO4_DZ),          &copter->g2.rc_channels.rc_channel(3)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO4_MAX),         &copter->g2.rc_channels.rc_channel(3)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO4_MIN),         &copter->g2.rc_channels.rc_channel(3)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO4_REVERSED),    &copter->g2.rc_channels.rc_channel(3)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO4_TRIM),        &copter->g2.rc_channels.rc_channel(3)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO5_DZ),          &copter->g2.rc_channels.rc_channel(4)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO5_MAX),         &copter->g2.rc_channels.rc_channel(4)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO5_MIN),         &copter->g2.rc_channels.rc_channel(4)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO5_REVERSED),    &copter->g2.rc_channels.rc_channel(4)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO5_TRIM),        &copter->g2.rc_channels.rc_channel(4)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO6_DZ),          &copter->g2.rc_channels.rc_channel(5)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO6_MAX),         &copter->g2.rc_channels.rc_channel(5)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO6_MIN),         &copter->g2.rc_channels.rc_channel(5)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO6_REVERSED),    &copter->g2.rc_channels.rc_channel(5)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO6_TRIM),        &copter->g2.rc_channels.rc_channel(5)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO7_DZ),          &copter->g2.rc_channels.rc_channel(6)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO7_MAX),         &copter->g2.rc_channels.rc_channel(6)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO7_MIN),         &copter->g2.rc_channels.rc_channel(6)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO7_REVERSED),    &copter->g2.rc_channels.rc_channel(6)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO7_TRIM),        &copter->g2.rc_channels.rc_channel(6)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO8_DZ),          &copter->g2.rc_channels.rc_channel(7)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO8_MAX),         &copter->g2.rc_channels.rc_channel(7)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO8_MIN),         &copter->g2.rc_channels.rc_channel(7)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO8_REVERSED),    &copter->g2.rc_channels.rc_channel(7)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO8_TRIM),        &copter->g2.rc_channels.rc_channel(7)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO9_DZ),          &copter->g2.rc_channels.rc_channel(8)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO9_MAX),         &copter->g2.rc_channels.rc_channel(8)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO9_MIN),         &copter->g2.rc_channels.rc_channel(8)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO9_REVERSED),    &copter->g2.rc_channels.rc_channel(8)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO9_TRIM),        &copter->g2.rc_channels.rc_channel(8)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO10_DZ),         &copter->g2.rc_channels.rc_channel(9)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO10_MAX),        &copter->g2.rc_channels.rc_channel(9)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO10_MIN),        &copter->g2.rc_channels.rc_channel(9)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO10_REVERSED),   &copter->g2.rc_channels.rc_channel(9)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO10_TRIM),       &copter->g2.rc_channels.rc_channel(9)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO11_DZ),         &copter->g2.rc_channels.rc_channel(10)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO11_MAX),        &copter->g2.rc_channels.rc_channel(10)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO11_MIN),        &copter->g2.rc_channels.rc_channel(10)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO11_REVERSED),   &copter->g2.rc_channels.rc_channel(10)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO11_TRIM),       &copter->g2.rc_channels.rc_channel(10)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO12_DZ),         &copter->g2.rc_channels.rc_channel(11)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO12_MAX),        &copter->g2.rc_channels.rc_channel(11)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO12_MIN),        &copter->g2.rc_channels.rc_channel(11)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO12_REVERSED),   &copter->g2.rc_channels.rc_channel(11)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO12_TRIM),       &copter->g2.rc_channels.rc_channel(11)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO13_DZ),         &copter->g2.rc_channels.rc_channel(12)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO13_MAX),        &copter->g2.rc_channels.rc_channel(12)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO13_MIN),        &copter->g2.rc_channels.rc_channel(12)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO13_REVERSED),   &copter->g2.rc_channels.rc_channel(12)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO13_TRIM),       &copter->g2.rc_channels.rc_channel(12)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO14_DZ),         &copter->g2.rc_channels.rc_channel(13)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO14_MAX),        &copter->g2.rc_channels.rc_channel(13)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO14_MIN),        &copter->g2.rc_channels.rc_channel(13)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO14_REVERSED),   &copter->g2.rc_channels.rc_channel(13)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO14_TRIM),       &copter->g2.rc_channels.rc_channel(13)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO15_DZ),         &copter->g2.rc_channels.rc_channel(14)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO15_MAX),        &copter->g2.rc_channels.rc_channel(14)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO15_MIN),        &copter->g2.rc_channels.rc_channel(14)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO15_REVERSED),   &copter->g2.rc_channels.rc_channel(14)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO15_TRIM),       &copter->g2.rc_channels.rc_channel(14)->radio_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO16_DZ),         &copter->g2.rc_channels.rc_channel(15)->dead_zone));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO16_MAX),        &copter->g2.rc_channels.rc_channel(15)->radio_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO16_MIN),        &copter->g2.rc_channels.rc_channel(15)->radio_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO16_REVERSED),   &copter->g2.rc_channels.rc_channel(15)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, RADIO16_TRIM),       &copter->g2.rc_channels.rc_channel(15)->radio_trim));
    // PARAM_FLOAT(RNGFND_ADDR, 0),
    // PARAM_FLOAT(RNGFND_FUNCTION, 0),
    // PARAM_FLOAT(RNGFND_GAIN, 0.8),
    // PARAM_FLOAT(RNGFND_GNDCLEAR, 10),
    // PARAM_FLOAT(RNGFND_MAX_CM, 3000),
    // PARAM_FLOAT(RNGFND_MIN_CM, 80),
    // PARAM_FLOAT(RNGFND_OFFSET, 0),
    // PARAM_FLOAT(RNGFND_ORDER, 0),
    // PARAM_FLOAT(RNGFND_ORIENT, 25),
    // PARAM_FLOAT(RNGFND_PIN, -1),
    // PARAM_FLOAT(RNGFND_POS_X, 0),
    // PARAM_FLOAT(RNGFND_POS_Y, 0),
    // PARAM_FLOAT(RNGFND_POS_Z, 0),
    // PARAM_FLOAT(RNGFND_PWRRNG, 0),
    // PARAM_FLOAT(RNGFND_RMETRIC, 0),
    // PARAM_FLOAT(RNGFND_SCALING, 0),
    // PARAM_FLOAT(RNGFND_SETTLE, 0),
    // PARAM_FLOAT(RNGFND_STOP_PIN, -1),
    // PARAM_FLOAT(RNGFND_TYPE, 0),
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO1_FUNCTION),    &copter->g2.servo_channels.srv_channel(0)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO1_MAX),         &copter->g2.servo_channels.srv_channel(0)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO1_MIN),         &copter->g2.servo_channels.srv_channel(0)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO1_REVERSED),    &copter->g2.servo_channels.srv_channel(0)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO1_TRIM),        &copter->g2.servo_channels.srv_channel(0)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO2_FUNCTION),    &copter->g2.servo_channels.srv_channel(1)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO2_MAX),         &copter->g2.servo_channels.srv_channel(1)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO2_MIN),         &copter->g2.servo_channels.srv_channel(1)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO2_REVERSED),    &copter->g2.servo_channels.srv_channel(1)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO2_TRIM),        &copter->g2.servo_channels.srv_channel(1)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO3_FUNCTION),    &copter->g2.servo_channels.srv_channel(2)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO3_MAX),         &copter->g2.servo_channels.srv_channel(2)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO3_MIN),         &copter->g2.servo_channels.srv_channel(2)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO3_REVERSED),    &copter->g2.servo_channels.srv_channel(2)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO3_TRIM),        &copter->g2.servo_channels.srv_channel(2)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO4_FUNCTION),    &copter->g2.servo_channels.srv_channel(3)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO4_MAX),         &copter->g2.servo_channels.srv_channel(3)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO4_MIN),         &copter->g2.servo_channels.srv_channel(3)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO4_REVERSED),    &copter->g2.servo_channels.srv_channel(3)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO4_TRIM),        &copter->g2.servo_channels.srv_channel(3)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO5_FUNCTION),    &copter->g2.servo_channels.srv_channel(4)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO5_MAX),         &copter->g2.servo_channels.srv_channel(4)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO5_MIN),         &copter->g2.servo_channels.srv_channel(4)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO5_REVERSED),    &copter->g2.servo_channels.srv_channel(4)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO5_TRIM),        &copter->g2.servo_channels.srv_channel(4)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO6_FUNCTION),    &copter->g2.servo_channels.srv_channel(5)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO6_MAX),         &copter->g2.servo_channels.srv_channel(5)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO6_MIN),         &copter->g2.servo_channels.srv_channel(5)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO6_REVERSED),    &copter->g2.servo_channels.srv_channel(5)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO6_TRIM),        &copter->g2.servo_channels.srv_channel(5)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO7_FUNCTION),    &copter->g2.servo_channels.srv_channel(6)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO7_MAX),         &copter->g2.servo_channels.srv_channel(6)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO7_MIN),         &copter->g2.servo_channels.srv_channel(6)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO7_REVERSED),    &copter->g2.servo_channels.srv_channel(6)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO7_TRIM),        &copter->g2.servo_channels.srv_channel(6)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO8_FUNCTION),    &copter->g2.servo_channels.srv_channel(7)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO8_MAX),         &copter->g2.servo_channels.srv_channel(7)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO8_MIN),         &copter->g2.servo_channels.srv_channel(7)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO8_REVERSED),    &copter->g2.servo_channels.srv_channel(7)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO8_TRIM),        &copter->g2.servo_channels.srv_channel(7)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO9_FUNCTION),    &copter->g2.servo_channels.srv_channel(8)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO9_MAX),         &copter->g2.servo_channels.srv_channel(8)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO9_MIN),         &copter->g2.servo_channels.srv_channel(8)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO9_REVERSED),    &copter->g2.servo_channels.srv_channel(8)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO9_TRIM),        &copter->g2.servo_channels.srv_channel(8)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO10_FUNCTION),   &copter->g2.servo_channels.srv_channel(9)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO10_MAX),        &copter->g2.servo_channels.srv_channel(9)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO10_MIN),        &copter->g2.servo_channels.srv_channel(9)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO10_REVERSED),   &copter->g2.servo_channels.srv_channel(9)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO10_TRIM),       &copter->g2.servo_channels.srv_channel(9)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO11_FUNCTION),   &copter->g2.servo_channels.srv_channel(10)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO11_MAX),        &copter->g2.servo_channels.srv_channel(10)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO11_MIN),        &copter->g2.servo_channels.srv_channel(10)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO11_REVERSED),   &copter->g2.servo_channels.srv_channel(10)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO11_TRIM),       &copter->g2.servo_channels.srv_channel(10)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO12_FUNCTION),   &copter->g2.servo_channels.srv_channel(11)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO12_MAX),        &copter->g2.servo_channels.srv_channel(11)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO12_MIN),        &copter->g2.servo_channels.srv_channel(11)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO12_REVERSED),   &copter->g2.servo_channels.srv_channel(11)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO12_TRIM),       &copter->g2.servo_channels.srv_channel(11)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO13_FUNCTION),   &copter->g2.servo_channels.srv_channel(12)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO13_MAX),        &copter->g2.servo_channels.srv_channel(12)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO13_MIN),        &copter->g2.servo_channels.srv_channel(12)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO13_REVERSED),   &copter->g2.servo_channels.srv_channel(12)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO13_TRIM),       &copter->g2.servo_channels.srv_channel(12)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO14_FUNCTION),   &copter->g2.servo_channels.srv_channel(13)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO14_MAX),        &copter->g2.servo_channels.srv_channel(13)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO14_MIN),        &copter->g2.servo_channels.srv_channel(13)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO14_REVERSED),   &copter->g2.servo_channels.srv_channel(13)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO14_TRIM),       &copter->g2.servo_channels.srv_channel(13)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO15_FUNCTION),   &copter->g2.servo_channels.srv_channel(14)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO15_MAX),        &copter->g2.servo_channels.srv_channel(14)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO15_MIN),        &copter->g2.servo_channels.srv_channel(14)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO15_REVERSED),   &copter->g2.servo_channels.srv_channel(14)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO15_TRIM),       &copter->g2.servo_channels.srv_channel(14)->servo_trim));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO16_FUNCTION),   &copter->g2.servo_channels.srv_channel(15)->function));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO16_MAX),        &copter->g2.servo_channels.srv_channel(15)->servo_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO16_MIN),        &copter->g2.servo_channels.srv_channel(15)->servo_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO16_REVERSED),   &copter->g2.servo_channels.srv_channel(15)->reversed));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, SERVO16_TRIM),       &copter->g2.servo_channels.srv_channel(15)->servo_trim));
    // FMT_CHECK(param_link_variable(PARAM_GET(APM, SYSID_THISMAV),      &copter->g.sysid_this_mav));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, WP_NAVALT_MIN),      &copter->g2.wp_navalt_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, WP_YAW_BEHAVIOR),    &copter->g.wp_yaw_behavior));
}


void APM_Copter_Init_Para_P2(void)
{
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_ACCEL_P_MAX),    &copter->attitude_control->_accel_pitch_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_ACCEL_R_MAX),    &copter->attitude_control->_accel_roll_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_ACCEL_Y_MAX),    &copter->attitude_control->_accel_yaw_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_ANG_LIM_TC) ,    &copter->attitude_control->_angle_limit_tc));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_ANG_PIT_P),      &copter->attitude_control->_p_angle_pitch._kp));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_ANG_RLL_P),      &copter->attitude_control->_p_angle_roll._kp));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_ANG_YAW_P),      &copter->attitude_control->_p_angle_yaw._kp));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_ANGLE_BOOST),    &copter->attitude_control->_angle_boost_enabled));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_PIT_D),      &copter->attitude_control->_pid_rate_pitch._kd));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_PIT_FF),     &copter->attitude_control->_pid_rate_pitch._ff));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_PIT_FILT),   &copter->attitude_control->_pid_rate_pitch._filt_hz));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_PIT_I),      &copter->attitude_control->_pid_rate_pitch._ki));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_PIT_IMAX),   &copter->attitude_control->_pid_rate_pitch._imax));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_PIT_P),      &copter->attitude_control->_pid_rate_pitch._kp));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_RLL_D),      &copter->attitude_control->_pid_rate_roll._kd));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_RLL_FF),     &copter->attitude_control->_pid_rate_roll._ff));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_RLL_FILT),   &copter->attitude_control->_pid_rate_roll._filt_hz));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_RLL_I),      &copter->attitude_control->_pid_rate_roll._ki));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_RLL_IMAX),   &copter->attitude_control->_pid_rate_roll._imax));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_RLL_P),      &copter->attitude_control->_pid_rate_roll._kp));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_YAW_D),      &copter->attitude_control->_pid_rate_yaw._kd));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_YAW_FF),     &copter->attitude_control->_pid_rate_yaw._ff));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_YAW_FILT),   &copter->attitude_control->_pid_rate_yaw._filt_hz));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_YAW_I),      &copter->attitude_control->_pid_rate_yaw._ki));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_YAW_IMAX),   &copter->attitude_control->_pid_rate_yaw._imax));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RAT_YAW_P),      &copter->attitude_control->_pid_rate_yaw._kp));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_RATE_FF_ENAB),   &copter->attitude_control->_rate_bf_ff_enabled));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_SLEW_YAW),       &copter->attitude_control->_slew_yaw));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_THR_MIX_MAN),    &copter->attitude_control->_thr_mix_man));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_THR_MIX_MAX),    &copter->attitude_control->_thr_mix_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, ATC_THR_MIX_MIN),    &copter->attitude_control->_thr_mix_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, CIRCLE_RADIUS),      &copter->circle_nav->_radius));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, CIRCLE_RATE),        &copter->circle_nav->_rate));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, MOT_HOVER_LEARN),    &copter->motors->_throttle_hover_learn));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, MOT_PWM_MAX),        &copter->motors->_pwm_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, MOT_PWM_MIN),        &copter->motors->_pwm_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, MOT_SAFE_DISARM),    &copter->motors->_disarm_disable_pwm));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, MOT_SPIN_ARM),       &copter->motors->_spin_arm));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, MOT_SPIN_MAX),       &copter->motors->_spin_max));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, MOT_SPIN_MIN),       &copter->motors->_spin_min));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, MOT_SPOOL_TIME),     &copter->motors->_spool_up_time));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, MOT_THST_EXPO),      &copter->motors->_thrust_curve_expo));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, MOT_THST_HOVER),     &copter->motors->_throttle_hover));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, MOT_YAW_HEADROOM),   &copter->motors->_yaw_headroom));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, POS_XY_P),           &copter->pos_control->_p_pos_xy._kp));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, POS_Z_P),            &copter->pos_control->_p_pos_z._kp));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, VEL_XY_FILT_HZ),     &copter->pos_control->_pi_vel_xy._filt_hz));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, VEL_XY_I),           &copter->pos_control->_pi_vel_xy._ki));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, VEL_XY_IMAX),        &copter->pos_control->_pi_vel_xy._imax));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, VEL_XY_P),           &copter->pos_control->_pi_vel_xy._kp));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, VEL_Z_P),            &copter->pos_control->_p_vel_z._kp));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, WPNAV_ACCEL),        &copter->wp_nav->_wp_accel_cms));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, WPNAV_ACCEL_Z),      &copter->wp_nav->_wp_accel_z_cms));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, WPNAV_LOIT_JERK),    &copter->wp_nav->_loiter_jerk_max_cmsss));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, WPNAV_LOIT_MAXA),    &copter->wp_nav->_loiter_accel_cmss));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, WPNAV_LOIT_MINA),    &copter->wp_nav->_loiter_accel_min_cmss));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, WPNAV_LOIT_SPEED),   &copter->wp_nav->_loiter_speed_cms));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, WPNAV_RADIUS),       &copter->wp_nav->_wp_radius_cm));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, WPNAV_RFND_USE),     &copter->wp_nav->_rangefinder_use));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, WPNAV_SPEED),        &copter->wp_nav->_wp_speed_cms));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, WPNAV_SPEED_DN),     &copter->wp_nav->_wp_speed_down_cms));
    FMT_CHECK(param_link_variable(PARAM_GET(APM, WPNAV_SPEED_UP),     &copter->wp_nav->_wp_speed_up_cms));
}

}
