/******************************************************************************
 * Copyright 2020 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <firmament.h>
#include <board.h>

_EXT_DTCM0
static param_t __param_list_apm[] = {
//     /* Param here*/
    // PARAM_FLOAT(USER_TEST_P1, 0.15),
    PARAM_INT8(APM_INFO, 0),                    //copter->g.debug_info
    PARAM_FLOAT(ATC_ACCEL_P_MAX, 110000.0),     //copter->attitude_control->_accel_pitch_max
    PARAM_FLOAT(ATC_ACCEL_R_MAX, 110000.0),     //copter->attitude_control->_accel_roll_max
    PARAM_FLOAT(ATC_ACCEL_Y_MAX, 27000.0),      //copter->attitude_control->_accel_yaw_max
    PARAM_FLOAT(ATC_ANG_LIM_TC, 1),             //copter->attitude_control->_angle_limit_tc
    PARAM_FLOAT(ATC_ANG_PIT_P, 4.5),            //copter->attitude_control->_p_angle_pitch._kp
    PARAM_FLOAT(ATC_ANG_RLL_P, 4.5),            //copter->attitude_control->_p_angle_roll._kp
    PARAM_FLOAT(ATC_ANG_YAW_P, 4.5),            //copter->attitude_control->_p_angle_yaw._kp
    PARAM_INT8(ATC_ANGLE_BOOST, 1),             //copter->attitude_control->_angle_boost_enabled
    PARAM_FLOAT(ATC_RAT_PIT_D, 0.0036),         //copter->attitude_control->_pid_rate_pitch._kd
    PARAM_FLOAT(ATC_RAT_PIT_FF, 0),             //copter->attitude_control->_pid_rate_pitch._ff
    PARAM_FLOAT(ATC_RAT_PIT_FILT, 20),          //copter->attitude_control->_pid_rate_pitch._filt_hz
    PARAM_FLOAT(ATC_RAT_PIT_I, 0.09),           //copter->attitude_control->_pid_rate_pitch._ki
    PARAM_FLOAT(ATC_RAT_PIT_IMAX, 0.5),         //copter->attitude_control->_pid_rate_pitch._imax
    PARAM_FLOAT(ATC_RAT_PIT_P, 0.135),          //copter->attitude_control->_pid_rate_pitch._kp
    PARAM_FLOAT(ATC_RAT_RLL_D, 0.0036),         //copter->attitude_control->_pid_rate_roll._kd
    PARAM_FLOAT(ATC_RAT_RLL_FF, 0),             //copter->attitude_control->_pid_rate_roll._ff
    PARAM_FLOAT(ATC_RAT_RLL_FILT, 20),          //copter->attitude_control->_pid_rate_roll._filt_hz
    PARAM_FLOAT(ATC_RAT_RLL_I, 0.09),           //copter->attitude_control->_pid_rate_roll._ki
    PARAM_FLOAT(ATC_RAT_RLL_IMAX, 0.5),         //copter->attitude_control->_pid_rate_roll._imax
    PARAM_FLOAT(ATC_RAT_RLL_P, 0.135),          //copter->attitude_control->_pid_rate_roll._kp
    PARAM_FLOAT(ATC_RAT_YAW_D, 0.0),            //copter->attitude_control->_pid_rate_yaw._kd
    PARAM_FLOAT(ATC_RAT_YAW_FF, 0),             //copter->attitude_control->_pid_rate_yaw._ff
    PARAM_FLOAT(ATC_RAT_YAW_FILT, 2.5),         //copter->attitude_control->_pid_rate_yaw._filt_hz
    PARAM_FLOAT(ATC_RAT_YAW_I, 0.018),          //copter->attitude_control->_pid_rate_yaw._ki
    PARAM_FLOAT(ATC_RAT_YAW_IMAX, 0.5),         //copter->attitude_control->_pid_rate_yaw._imax
    PARAM_FLOAT(ATC_RAT_YAW_P, 0.18),           //copter->attitude_control->_pid_rate_yaw._kp
    PARAM_INT8(ATC_RATE_FF_ENAB, 1),            //copter->attitude_control->_rate_bf_ff_enabled
    PARAM_FLOAT(ATC_SLEW_YAW, 1500),            //copter->attitude_control->_slew_yaw
    PARAM_FLOAT(ATC_THR_MIX_MAN, 0.5),          //copter->attitude_control->_thr_mix_man
    PARAM_FLOAT(ATC_THR_MIX_MAX, 0.5),          //copter->attitude_control->_thr_mix_max
    PARAM_FLOAT(ATC_THR_MIX_MIN, 0.1),          //copter->attitude_control->_thr_mix_min
    PARAM_INT8(CH7_OPT, 0),                     //copter->g.ch7_option
    PARAM_INT8(CH8_OPT, 0),                     //copter->g.ch8_option
    PARAM_INT8(CH9_OPT, 0),                     //copter->g.ch9_option
    PARAM_INT8(CH10_OPT, 0),                    //copter->g.ch10_option
    PARAM_INT8(CH11_OPT, 0),                    //copter->g.ch11_option
    PARAM_INT8(CH12_OPT, 0),                    //copter->g.ch12_option
    PARAM_FLOAT(CIRCLE_RADIUS, 1000),           //copter->circle_nav->_radius
    PARAM_FLOAT(CIRCLE_RATE, 20),               //copter->circle_nav->_rate
    PARAM_FLOAT(DISARM_DELAY, 10),              //copter->g.disarm_delay
    PARAM_INT8(FLTMODE1, 0),                    //copter->g.flight_mode1
    PARAM_INT8(FLTMODE2, 0),                    //copter->g.flight_mode2
    PARAM_INT8(FLTMODE3, 0),                    //copter->g.flight_mode3
    PARAM_INT8(FLTMODE4, 0),                    //copter->g.flight_mode4
    PARAM_INT8(FLTMODE5, 0),                    //copter->g.flight_mode5
    PARAM_INT8(FLTMODE6, 0),                    //copter->g.flight_mode6
    PARAM_INT8(FLOW_ENABLE,0),                  //copter->optflow._enabled
    PARAM_INT16(FLOW_FXSCALER,0),               //copter->optflow._flowScalerX
    PARAM_INT16(FLOW_FYSCALER,0),               //copter->optflow._flowScalerY
    PARAM_INT16(FLOW_ORIENT_YAW,0),             //copter->optflow._yawAngle_cd
    PARAM_FLOAT(FLOW_POS_X,0),                  //copter->optflow._pos_offset.x
    PARAM_FLOAT(FLOW_POS_Y,0),                  //copter->optflow._pos_offset.y
    PARAM_FLOAT(FLOW_POS_Z,0),                  //copter->optflow._pos_offset.z
    PARAM_FLOAT(FLOW_XY_P, 0.2f),               //copter->flowhold_t.flow_pi_xy._kp
    PARAM_FLOAT(FLOW_XY_I, 0.3f),               //copter->flowhold_t.flow_pi_xy._ki
    PARAM_FLOAT(FLOW_FLOW_MAX, 0.6f),           //copter->flowhold_t.flow_max
    PARAM_FLOAT(FLOW_FILT_HZ, 20.f),            //copter->flowhold_t.flow_filter_hz
    PARAM_INT8(FLOW_QUAL_MIN, 10),              //copter->flowhold_t.flow_min_quality
    PARAM_INT8(FLOW_BRAKE_RATE, 8),             //copter->flowhold_t.brake_rate_dps
    PARAM_INT8(FRAME_CLASS, 1),                 //copter->g2.frame_class
    PARAM_INT8(FRAME_TYPE, 1),                  //copter->g.frame_type
    PARAM_FLOAT(FS_BATT_ENABLE, 0),
    PARAM_FLOAT(FS_BATT_ENABLE2, 0),
    PARAM_FLOAT(FS_BATT_MAH, 0),
    PARAM_FLOAT(FS_BATT_VOLT2, 10.5),
    PARAM_FLOAT(FS_BATT_VOLTAGE, 38.31),
    PARAM_INT8(FS_CRASH_CHECK, 0),              //copter->g.fs_crash_check
    PARAM_INT8(FS_GCS_ENABLE, 0),               //copter->g.failsafe_gcs
    PARAM_INT8(FS_THR_ENABLE, 1),               //copter->g.failsafe_throttle
    PARAM_INT16(FS_THR_VALUE, 975),             //copter->g.failsafe_throttle_value
    PARAM_INT8(MIS_RESTART, 0),                 //copter->mission._restart
    PARAM_INT16(MIS_TOTAL, 50),                 //copter->mission._cmd_total
    PARAM_FLOAT(MOT_HOVER_LEARN, 2),            //copter->motors->_throttle_hover_learn
    PARAM_INT16(MOT_PWM_MAX, 0),                //copter->motors->_pwm_max
    PARAM_INT16(MOT_PWM_MIN, 0),                //copter->motors->_pwm_min
    PARAM_INT8(MOT_SAFE_DISARM, 0),             //copter->motors->_disarm_disable_pwm
    PARAM_FLOAT(MOT_SPIN_ARM, 0.1),             //copter->motors->_spin_arm
    PARAM_FLOAT(MOT_SPIN_MAX, 0.95),            //copter->motors->_spin_max
    PARAM_FLOAT(MOT_SPIN_MIN, 0.15),            //copter->motors->_spin_min
    PARAM_FLOAT(MOT_SPOOL_TIME, 0.5),           //copter->motors->_spool_up_time
    PARAM_FLOAT(MOT_THST_EXPO, 0.0f),           //copter->motors->_thrust_curve_expo
    PARAM_FLOAT(MOT_THST_HOVER, 0.35),          //copter->motors->_throttle_hover
    PARAM_INT16(MOT_YAW_HEADROOM, 200),         //copter->motors->_yaw_headroom
    PARAM_INT16(PHLD_BRAKE_ANGLE, 3000),        //copter->g.poshold_brake_angle_max
    PARAM_INT16(PHLD_BRAKE_RATE, 8),            //copter->g.poshold_brake_rate
    PARAM_INT16(PILOT_ACCEL_Z, 150),            //copter->g.pilot_accel_z
    PARAM_INT16(PILOT_THR_BHV, 0),              //copter->g.throttle_behavior
    PARAM_FLOAT(PILOT_THR_FILT, 0),             //copter->g.throttle_filt
    PARAM_FLOAT(PILOT_TKOFF_ALT, 200),          //copter->g.pilot_takeoff_alt
    PARAM_INT16(PILOT_TKOFF_DZ, 100),           //copter->g.takeoff_trigger_dz
    PARAM_INT16(PILOT_VELZ_MAX, 200),           //copter->g.pilot_velocity_z_max
    PARAM_FLOAT(POS_XY_P, 0.85),                //copter->pos_control->_p_pos_xy._kp
    PARAM_FLOAT(POS_Z_P, 1),                    //copter->pos_control->_p_pos_z._kp
    PARAM_INT8(RC_FEEL_RP, 60),                 //copter->g.rc_feel_rp
    PARAM_INT16(RADIO1_DZ, 20),                 //copter->g2.rc_channels.rc_channel(0)->dead_zone
    PARAM_INT16(RADIO1_MAX, 1950),              //copter->g2.rc_channels.rc_channel(0)->radio_max
    PARAM_INT16(RADIO1_MIN, 1094),              //copter->g2.rc_channels.rc_channel(0)->radio_min
    PARAM_INT8(RADIO1_REVERSED, 0),             //copter->g2.rc_channels.rc_channel(0)->reversed
    PARAM_INT16(RADIO1_TRIM, 1515),             //copter->g2.rc_channels.rc_channel(0)->radio_trim
    PARAM_INT16(RADIO2_DZ, 20),                 //copter->g2.rc_channels.rc_channel(2)->dead_zone
    PARAM_INT16(RADIO2_MAX, 1950),              //copter->g2.rc_channels.rc_channel(2)->radio_max
    PARAM_INT16(RADIO2_MIN, 1094),              //copter->g2.rc_channels.rc_channel(2)->radio_min
    PARAM_INT8(RADIO2_REVERSED, 0),             //copter->g2.rc_channels.rc_channel(2)->reversed
    PARAM_INT16(RADIO2_TRIM, 1515),             //copter->g2.rc_channels.rc_channel(2)->radio_trim
    PARAM_INT16(RADIO3_DZ, 50),                 //copter->g2.rc_channels.rc_channel(2)->dead_zone
    PARAM_INT16(RADIO3_MAX, 1950),              //copter->g2.rc_channels.rc_channel(2)->radio_max
    PARAM_INT16(RADIO3_MIN, 1094),              //copter->g2.rc_channels.rc_channel(2)->radio_min
    PARAM_INT8(RADIO3_REVERSED, 0),             //copter->g2.rc_channels.rc_channel(2)->reversed
    PARAM_INT16(RADIO3_TRIM, 1051),             //copter->g2.rc_channels.rc_channel(2)->radio_trim
    PARAM_INT16(RADIO4_DZ, 20),                 //copter->g2.rc_channels.rc_channel(3)->dead_zone
    PARAM_INT16(RADIO4_MAX, 1950),              //copter->g2.rc_channels.rc_channel(3)->radio_max
    PARAM_INT16(RADIO4_MIN, 1094),              //copter->g2.rc_channels.rc_channel(3)->radio_min
    PARAM_INT8(RADIO4_REVERSED, 0),             //copter->g2.rc_channels.rc_channel(3)->reversed
    PARAM_INT16(RADIO4_TRIM, 1515),             //copter->g2.rc_channels.rc_channel(3)->radio_trim
    PARAM_INT16(RADIO5_DZ, 20),                 //copter->g2.rc_channels.rc_channel(4)->dead_zone
    PARAM_INT16(RADIO5_MAX, 1950),              //copter->g2.rc_channels.rc_channel(4)->radio_max
    PARAM_INT16(RADIO5_MIN, 1051),              //copter->g2.rc_channels.rc_channel(4)->radio_min
    PARAM_INT8(RADIO5_REVERSED, 0),             //copter->g2.rc_channels.rc_channel(4)->reversed
    PARAM_INT16(RADIO5_TRIM, 1500),             //copter->g2.rc_channels.rc_channel(4)->radio_trim
    PARAM_INT16(RADIO6_DZ, 20),                 //copter->g2.rc_channels.rc_channel(5)->dead_zone
    PARAM_INT16(RADIO6_MAX, 1950),              //copter->g2.rc_channels.rc_channel(5)->radio_max
    PARAM_INT16(RADIO6_MIN, 1051),              //copter->g2.rc_channels.rc_channel(5)->radio_min
    PARAM_INT8(RADIO6_REVERSED, 0),             //copter->g2.rc_channels.rc_channel(5)->reversed
    PARAM_INT16(RADIO6_TRIM, 1500),             //copter->g2.rc_channels.rc_channel(5)->radio_trim
    PARAM_INT16(RADIO7_DZ, 20),                 //copter->g2.rc_channels.rc_channel(6)->dead_zone
    PARAM_INT16(RADIO7_MAX, 1950),              //copter->g2.rc_channels.rc_channel(6)->radio_max
    PARAM_INT16(RADIO7_MIN, 1051),              //copter->g2.rc_channels.rc_channel(6)->radio_min
    PARAM_INT8(RADIO7_REVERSED, 0),             //copter->g2.rc_channels.rc_channel(6)->reversed
    PARAM_INT16(RADIO7_TRIM, 1500),             //copter->g2.rc_channels.rc_channel(6)->radio_trim
    PARAM_INT16(RADIO8_DZ, 20),                 //copter->g2.rc_channels.rc_channel(7)->dead_zone
    PARAM_INT16(RADIO8_MAX, 1950),              //copter->g2.rc_channels.rc_channel(7)->radio_max
    PARAM_INT16(RADIO8_MIN, 1051),              //copter->g2.rc_channels.rc_channel(7)->radio_min
    PARAM_INT8(RADIO8_REVERSED, 0),             //copter->g2.rc_channels.rc_channel(7)->reversed
    PARAM_INT16(RADIO8_TRIM, 1500),             //copter->g2.rc_channels.rc_channel(7)->radio_trim
    PARAM_INT16(RADIO9_DZ, 20),                 //copter->g2.rc_channels.rc_channel(8)->dead_zone
    PARAM_INT16(RADIO9_MAX, 1950),              //copter->g2.rc_channels.rc_channel(8)->radio_max
    PARAM_INT16(RADIO9_MIN, 1051),              //copter->g2.rc_channels.rc_channel(8)->radio_min
    PARAM_INT8(RADIO9_REVERSED, 0),             //copter->g2.rc_channels.rc_channel(8)->reversed
    PARAM_INT16(RADIO9_TRIM, 1500),             //copter->g2.rc_channels.rc_channel(8)->radio_trim
    PARAM_INT16(RADIO10_DZ, 20),                //copter->g2.rc_channels.rc_channel(9)->dead_zone
    PARAM_INT16(RADIO10_MAX, 1950),             //copter->g2.rc_channels.rc_channel(9)->radio_max
    PARAM_INT16(RADIO10_MIN, 1051),             //copter->g2.rc_channels.rc_channel(9)->radio_min
    PARAM_INT8(RADIO10_REVERSED, 0),            //copter->g2.rc_channels.rc_channel(9)->reversed
    PARAM_INT16(RADIO10_TRIM, 1500),            //copter->g2.rc_channels.rc_channel(9)->radio_trim
    PARAM_INT16(RADIO11_DZ, 20),                //copter->g2.rc_channels.rc_channel(10)->dead_zone
    PARAM_INT16(RADIO11_MAX, 1950),             //copter->g2.rc_channels.rc_channel(10)->radio_max
    PARAM_INT16(RADIO11_MIN, 1051),             //copter->g2.rc_channels.rc_channel(10)->radio_min
    PARAM_INT8(RADIO11_REVERSED, 0),            //copter->g2.rc_channels.rc_channel(10)->reversed
    PARAM_INT16(RADIO11_TRIM, 1500),            //copter->g2.rc_channels.rc_channel(10)->radio_trim
    PARAM_INT16(RADIO12_DZ, 20),                //copter->g2.rc_channels.rc_channel(11)->dead_zone
    PARAM_INT16(RADIO12_MAX, 1950),             //copter->g2.rc_channels.rc_channel(11)->radio_max
    PARAM_INT16(RADIO12_MIN, 1051),             //copter->g2.rc_channels.rc_channel(11)->radio_min
    PARAM_INT8(RADIO12_REVERSED, 0),            //copter->g2.rc_channels.rc_channel(11)->reversed
    PARAM_INT16(RADIO12_TRIM, 1500),            //copter->g2.rc_channels.rc_channel(11)->radio_trim
    PARAM_INT16(RADIO13_DZ, 20),                //copter->g2.rc_channels.rc_channel(12)->dead_zone
    PARAM_INT16(RADIO13_MAX, 1950),             //copter->g2.rc_channels.rc_channel(12)->radio_max
    PARAM_INT16(RADIO13_MIN, 1051),             //copter->g2.rc_channels.rc_channel(12)->radio_min
    PARAM_INT8(RADIO13_REVERSED, 0),            //copter->g2.rc_channels.rc_channel(12)->reversed
    PARAM_INT16(RADIO13_TRIM, 1500),            //copter->g2.rc_channels.rc_channel(12)->radio_trim
    PARAM_INT16(RADIO14_DZ, 20),                //copter->g2.rc_channels.rc_channel(13)->dead_zone
    PARAM_INT16(RADIO14_MAX, 1950),             //copter->g2.rc_channels.rc_channel(13)->radio_max
    PARAM_INT16(RADIO14_MIN, 1051),             //copter->g2.rc_channels.rc_channel(13)->radio_min
    PARAM_INT8(RADIO14_REVERSED, 0),            //copter->g2.rc_channels.rc_channel(13)->reversed
    PARAM_INT16(RADIO14_TRIM, 1500),            //copter->g2.rc_channels.rc_channel(13)->radio_trim
    PARAM_INT16(RADIO15_DZ, 20),                //copter->g2.rc_channels.rc_channel(14)->dead_zone
    PARAM_INT16(RADIO15_MAX, 1950),             //copter->g2.rc_channels.rc_channel(14)->radio_max
    PARAM_INT16(RADIO15_MIN, 1051),             //copter->g2.rc_channels.rc_channel(14)->radio_min
    PARAM_INT8(RADIO15_REVERSED, 0),            //copter->g2.rc_channels.rc_channel(14)->reversed
    PARAM_INT16(RADIO15_TRIM, 1500),            //copter->g2.rc_channels.rc_channel(14)->radio_trim
    PARAM_INT16(RADIO16_DZ, 20),                //copter->g2.rc_channels.rc_channel(15)->dead_zone
    PARAM_INT16(RADIO16_MAX, 1950),             //copter->g2.rc_channels.rc_channel(15)->radio_max
    PARAM_INT16(RADIO16_MIN, 1051),             //copter->g2.rc_channels.rc_channel(15)->radio_min
    PARAM_INT8(RADIO16_REVERSED, 0),            //copter->g2.rc_channels.rc_channel(15)->reversed
    PARAM_INT16(RADIO16_TRIM, 1500),            //copter->g2.rc_channels.rc_channel(15)->radio_trim
    PARAM_FLOAT(RNGFND_GAIN, 0.8),              //copter->g.rangefinder_gain
    PARAM_INT8(RNGFND_GNDCLEAR, 10),            //copter->rangefinder._ground_clearance_cm[0]
    PARAM_INT16(RNGFND_MAX_CM, 800),            //copter->rangefinder._max_distance_cm[0]
    PARAM_INT16(RNGFND_MIN_CM, 80),             //copter->rangefinder._min_distance_cm[0]
    PARAM_INT8(RNGFND_ORIENT, 25),              //copter->rangefinder._orientation[0]
    PARAM_INT8(RNGFND_TYPE, 0),                 //copter->rangefinder._type[0]
    PARAM_INT8(SERVO1_FUNCTION, 0),             //copter->g2.servo_channels.srv_channel(0)->function
    PARAM_INT16(SERVO1_MAX, 1900),              //copter->g2.servo_channels.srv_channel(0)->servo_max
    PARAM_INT16(SERVO1_MIN, 1100),              //copter->g2.servo_channels.srv_channel(0)->servo_min
    PARAM_INT8(SERVO1_REVERSED, 0),             //copter->g2.servo_channels.srv_channel(0)->reversed
    PARAM_INT16(SERVO1_TRIM, 1500),             //copter->g2.servo_channels.srv_channel(0)->servo_trim
    PARAM_INT8(SERVO2_FUNCTION, 0),             //copter->g2.servo_channels.srv_channel(1)->function
    PARAM_INT16(SERVO2_MAX, 1900),              //copter->g2.servo_channels.srv_channel(1)->servo_max
    PARAM_INT16(SERVO2_MIN, 1100),              //copter->g2.servo_channels.srv_channel(1)->servo_min
    PARAM_INT8(SERVO2_REVERSED, 0),             //copter->g2.servo_channels.srv_channel(1)->reversed
    PARAM_INT16(SERVO2_TRIM, 1500),             //copter->g2.servo_channels.srv_channel(1)->servo_trim
    PARAM_INT8(SERVO3_FUNCTION, 0),             //copter->g2.servo_channels.srv_channel(2)->function
    PARAM_INT16(SERVO3_MAX, 1900),              //copter->g2.servo_channels.srv_channel(2)->servo_max
    PARAM_INT16(SERVO3_MIN, 1100),              //copter->g2.servo_channels.srv_channel(2)->servo_min
    PARAM_INT8(SERVO3_REVERSED, 0),             //copter->g2.servo_channels.srv_channel(2)->reversed
    PARAM_INT16(SERVO3_TRIM, 1500),             //copter->g2.servo_channels.srv_channel(2)->servo_trim
    PARAM_INT8(SERVO4_FUNCTION, 0),             //copter->g2.servo_channels.srv_channel(3)->function
    PARAM_INT16(SERVO4_MAX, 1900),              //copter->g2.servo_channels.srv_channel(3)->servo_max
    PARAM_INT16(SERVO4_MIN, 1100),              //copter->g2.servo_channels.srv_channel(3)->servo_min
    PARAM_INT8(SERVO4_REVERSED, 0),             //copter->g2.servo_channels.srv_channel(3)->reversed
    PARAM_INT16(SERVO4_TRIM, 1500),             //copter->g2.servo_channels.srv_channel(3)->servo_trim
    PARAM_INT8(SERVO5_FUNCTION, 0),             //copter->g2.servo_channels.srv_channel(4)->function
    PARAM_INT16(SERVO5_MAX, 1900),              //copter->g2.servo_channels.srv_channel(4)->servo_max
    PARAM_INT16(SERVO5_MIN, 1100),              //copter->g2.servo_channels.srv_channel(4)->servo_min
    PARAM_INT8(SERVO5_REVERSED, 0),             //copter->g2.servo_channels.srv_channel(4)->reversed
    PARAM_INT16(SERVO5_TRIM, 1500),             //copter->g2.servo_channels.srv_channel(4)->servo_trim
    PARAM_INT8(SERVO6_FUNCTION, 0),             //copter->g2.servo_channels.srv_channel(5)->function
    PARAM_INT16(SERVO6_MAX, 1900),              //copter->g2.servo_channels.srv_channel(5)->servo_max
    PARAM_INT16(SERVO6_MIN, 1100),              //copter->g2.servo_channels.srv_channel(5)->servo_min
    PARAM_INT8(SERVO6_REVERSED, 0),             //copter->g2.servo_channels.srv_channel(5)->reversed
    PARAM_INT16(SERVO6_TRIM, 1500),             //copter->g2.servo_channels.srv_channel(5)->servo_trim
    PARAM_INT8(SERVO7_FUNCTION, 0),             //copter->g2.servo_channels.srv_channel(6)->function
    PARAM_INT16(SERVO7_MAX, 1900),              //copter->g2.servo_channels.srv_channel(6)->servo_max
    PARAM_INT16(SERVO7_MIN, 1100),              //copter->g2.servo_channels.srv_channel(6)->servo_min
    PARAM_INT8(SERVO7_REVERSED, 0),             //copter->g2.servo_channels.srv_channel(6)->reversed
    PARAM_INT16(SERVO7_TRIM, 1500),             //copter->g2.servo_channels.srv_channel(6)->servo_trim
    PARAM_INT8(SERVO8_FUNCTION, 0),             //copter->g2.servo_channels.srv_channel(7)->function
    PARAM_INT16(SERVO8_MAX, 1900),              //copter->g2.servo_channels.srv_channel(7)->servo_max
    PARAM_INT16(SERVO8_MIN, 1100),              //copter->g2.servo_channels.srv_channel(7)->servo_min
    PARAM_INT8(SERVO8_REVERSED, 0),             //copter->g2.servo_channels.srv_channel(7)->reversed
    PARAM_INT16(SERVO8_TRIM, 1500),             //copter->g2.servo_channels.srv_channel(7)->servo_trim
    PARAM_INT8(SERVO9_FUNCTION, 0),             //copter->g2.servo_channels.srv_channel(8)->function
    PARAM_INT16(SERVO9_MAX, 1900),              //copter->g2.servo_channels.srv_channel(8)->servo_max
    PARAM_INT16(SERVO9_MIN, 1100),              //copter->g2.servo_channels.srv_channel(8)->servo_min
    PARAM_INT8(SERVO9_REVERSED, 0),             //copter->g2.servo_channels.srv_channel(8)->reversed
    PARAM_INT16(SERVO9_TRIM, 1500),             //copter->g2.servo_channels.srv_channel(8)->servo_trim
    PARAM_INT8(SERVO10_FUNCTION, 0),            //copter->g2.servo_channels.srv_channel(9)->function
    PARAM_INT16(SERVO10_MAX, 1900),             //copter->g2.servo_channels.srv_channel(9)->servo_max
    PARAM_INT16(SERVO10_MIN, 1100),             //copter->g2.servo_channels.srv_channel(9)->servo_min
    PARAM_INT8(SERVO10_REVERSED, 0),            //copter->g2.servo_channels.srv_channel(9)->reversed
    PARAM_INT16(SERVO10_TRIM, 1500),            //copter->g2.servo_channels.srv_channel(9)->servo_trim
    PARAM_INT8(SERVO11_FUNCTION, 0),            //copter->g2.servo_channels.srv_channel(10)->function
    PARAM_INT16(SERVO11_MAX, 1900),             //copter->g2.servo_channels.srv_channel(10)->servo_max
    PARAM_INT16(SERVO11_MIN, 1100),             //copter->g2.servo_channels.srv_channel(10)->servo_min
    PARAM_INT8(SERVO11_REVERSED, 0),            //copter->g2.servo_channels.srv_channel(10)->reversed
    PARAM_INT16(SERVO11_TRIM, 1500),            //copter->g2.servo_channels.srv_channel(10)->servo_trim
    PARAM_INT8(SERVO12_FUNCTION, 0),            //copter->g2.servo_channels.srv_channel(11)->function
    PARAM_INT16(SERVO12_MAX, 1900),             //copter->g2.servo_channels.srv_channel(11)->servo_max
    PARAM_INT16(SERVO12_MIN, 1100),             //copter->g2.servo_channels.srv_channel(11)->servo_min
    PARAM_INT8(SERVO12_REVERSED, 0),            //copter->g2.servo_channels.srv_channel(11)->reversed
    PARAM_INT16(SERVO12_TRIM, 1500),            //copter->g2.servo_channels.srv_channel(11)->servo_trim
    PARAM_INT8(SERVO13_FUNCTION, 0),            //copter->g2.servo_channels.srv_channel(12)->function
    PARAM_INT16(SERVO13_MAX, 1900),             //copter->g2.servo_channels.srv_channel(12)->servo_max
    PARAM_INT16(SERVO13_MIN, 1100),             //copter->g2.servo_channels.srv_channel(12)->servo_min
    PARAM_INT8(SERVO13_REVERSED, 0),            //copter->g2.servo_channels.srv_channel(12)->reversed
    PARAM_INT16(SERVO13_TRIM, 1500),            //copter->g2.servo_channels.srv_channel(12)->servo_trim
    PARAM_INT8(SERVO14_FUNCTION, 0),            //copter->g2.servo_channels.srv_channel(13)->function
    PARAM_INT16(SERVO14_MAX, 1900),             //copter->g2.servo_channels.srv_channel(13)->servo_max
    PARAM_INT16(SERVO14_MIN, 1100),             //copter->g2.servo_channels.srv_channel(13)->servo_min
    PARAM_INT8(SERVO14_REVERSED, 0),            //copter->g2.servo_channels.srv_channel(13)->reversed
    PARAM_INT16(SERVO14_TRIM, 1500),            //copter->g2.servo_channels.srv_channel(13)->servo_trim
    PARAM_INT8(SERVO15_FUNCTION, 0),            //copter->g2.servo_channels.srv_channel(14)->function
    PARAM_INT16(SERVO15_MAX, 1900),             //copter->g2.servo_channels.srv_channel(14)->servo_max
    PARAM_INT16(SERVO15_MIN, 1100),             //copter->g2.servo_channels.srv_channel(14)->servo_min
    PARAM_INT8(SERVO15_REVERSED, 0),            //copter->g2.servo_channels.srv_channel(14)->reversed
    PARAM_INT16(SERVO15_TRIM, 1500),            //copter->g2.servo_channels.srv_channel(14)->servo_trim
    PARAM_INT8(SERVO16_FUNCTION, 0),            //copter->g2.servo_channels.srv_channel(15)->function
    PARAM_INT16(SERVO16_MAX, 1900),             //copter->g2.servo_channels.srv_channel(15)->servo_max
    PARAM_INT16(SERVO16_MIN, 1100),             //copter->g2.servo_channels.srv_channel(15)->servo_min
    PARAM_INT8(SERVO16_REVERSED, 0),            //copter->g2.servo_channels.srv_channel(15)->reversed
    PARAM_INT16(SERVO16_TRIM, 1500),            //copter->g2.servo_channels.srv_channel(15)->servo_trim
    PARAM_INT16(SYSID_THISMAV, 1),              //copter->g.sysid_this_mav
    PARAM_FLOAT(VEL_XY_FILT_HZ, 5),             //copter->pos_control->_pi_vel_xy._filt_hz
    PARAM_FLOAT(VEL_XY_I, 0.5),                 //copter->pos_control->_pi_vel_xy._ki
    PARAM_FLOAT(VEL_XY_IMAX, 1000),             //copter->pos_control->_pi_vel_xy._imax
    PARAM_FLOAT(VEL_XY_P, 0.95),                //copter->pos_control->_pi_vel_xy._kp
    PARAM_FLOAT(VEL_Z_P, 5),                    //copter->pos_control->_p_vel_z._kp
    PARAM_FLOAT(WP_NAVALT_MIN, 0),              //copter->g2.wp_navalt_min
    PARAM_INT8(WP_YAW_BEHAVIOR, 1),             //copter->g.wp_yaw_behavior
    PARAM_FLOAT(WPNAV_ACCEL, 200),              //copter->wp_nav->_wp_accel_cms
    PARAM_FLOAT(WPNAV_ACCEL_Z, 100),            //copter->wp_nav->_wp_accel_z_cms
    PARAM_FLOAT(WPNAV_LOIT_JERK, 1200),         //copter->wp_nav->_loiter_jerk_max_cmsss
    PARAM_FLOAT(WPNAV_LOIT_MAXA, 600),          //copter->wp_nav->_loiter_accel_cmss
    PARAM_FLOAT(WPNAV_LOIT_MINA, 150),          //copter->wp_nav->_loiter_accel_min_cmss
    PARAM_FLOAT(WPNAV_LOIT_SPEED, 1000),        //copter->wp_nav->_loiter_speed_cms
    PARAM_FLOAT(WPNAV_RADIUS, 200),             //copter->wp_nav->_wp_radius_cm
    PARAM_INT8(WPNAV_RFND_USE, 1),              //copter->wp_nav->_rangefinder_use
    PARAM_FLOAT(WPNAV_SPEED, 590),              //copter->wp_nav->_wp_speed_cms
    PARAM_FLOAT(WPNAV_SPEED_DN, 50),            //copter->wp_nav->_wp_speed_down_cms
    PARAM_FLOAT(WPNAV_SPEED_UP, 150),           //copter->wp_nav->_wp_speed_up_cms
};
PARAM_GROUP_DEFINE(APM, __param_list_apm);
