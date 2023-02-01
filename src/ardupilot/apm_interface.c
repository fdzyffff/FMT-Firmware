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

#include <APM.h>
#include <firmament.h>
#include <string.h>

#include "module/log/mlog.h"
#include "module/param/param.h"

#define FMT_READ_RADIO 1
// FMS input topic
// MCN_DECLARE(pilot_cmd);
MCN_DECLARE(gcs_cmd);
MCN_DECLARE(mission_data);
MCN_DECLARE(ins_output);
MCN_DECLARE(control_output);
MCN_DECLARE(rc_channels);

/* controller output topic */
MCN_DEFINE(control_output, sizeof(Control_Out_Bus));
MCN_DEFINE(fms_output, sizeof(FMS_Out_Bus));

// MCN_DEFINE(auto_cmd, sizeof(Auto_Cmd_Bus));
// MCN_DEFINE(fms_output, sizeof(FMS_Out_Bus));
// MCN_DEFINE(control_output, sizeof(Control_Out_Bus));

int16_t rcChannel_msg[16];
INS_Out_Bus ins_out_msg;
Mission_Data_Bus mission_data_msg;
GCS_Cmd_Bus gcs_cmd_msg;

FMS_Out_Bus fms_out_msg;
Control_Out_Bus control_out_msg;


// static int16_t rcChannel_tmp[16];
// static my_radio_in MY_RADIO_BUS;
// MCN_DEFINE(my_radio_in_topic, sizeof(MY_RADIO_BUS));

/* define parameters */
static param_t __param_list_apm[] = {
    /* Param here*/
    PARAM_FLOAT(USER_TEST_P1, 0.15),
    PARAM_FLOAT(ATC_ACCEL_P_MAX, 110000.0),     //copter->attitude_control->_accel_pitch_max
    PARAM_FLOAT(ATC_ACCEL_R_MAX, 110000.0),     //copter->attitude_control->_accel_roll_max
    PARAM_FLOAT(ATC_ACCEL_Y_MAX, 27000.0),      //copter->attitude_control->_accel_yaw_max
    PARAM_FLOAT(ATC_ANG_LIM_TC, 1),             //copter->attitude_control->_angle_limit_tc
    PARAM_FLOAT(ATC_ANG_PIT_P, 5.5),            //copter->attitude_control->_p_angle_pitch._kp
    PARAM_FLOAT(ATC_ANG_RLL_P, 5.5),            //copter->attitude_control->_p_angle_roll._kp
    PARAM_FLOAT(ATC_ANG_YAW_P, 6),              //copter->attitude_control->_p_angle_yaw._kp
    PARAM_INT8(ATC_ANGLE_BOOST, 1),             //copter->attitude_control->_angle_boost_enabled
    PARAM_FLOAT(ATC_RAT_PIT_D, 0.014),          //copter->attitude_control->_pid_rate_pitch._kd
    PARAM_FLOAT(ATC_RAT_PIT_FF, 0),             //copter->attitude_control->_pid_rate_pitch._ff
    PARAM_FLOAT(ATC_RAT_PIT_FILT, 20),          //copter->attitude_control->_pid_rate_pitch._filt_hz
    PARAM_FLOAT(ATC_RAT_PIT_I, 0.12),           //copter->attitude_control->_pid_rate_pitch._ki
    PARAM_FLOAT(ATC_RAT_PIT_IMAX, 0.5),         //copter->attitude_control->_pid_rate_pitch._imax
    PARAM_FLOAT(ATC_RAT_PIT_P, 0.16),           //copter->attitude_control->_pid_rate_pitch._kp
    PARAM_FLOAT(ATC_RAT_RLL_D, 0.013),          //copter->attitude_control->_pid_rate_roll._kd
    PARAM_FLOAT(ATC_RAT_RLL_FF, 0),             //copter->attitude_control->_pid_rate_roll._ff
    PARAM_FLOAT(ATC_RAT_RLL_FILT, 20),          //copter->attitude_control->_pid_rate_roll._filt_hz
    PARAM_FLOAT(ATC_RAT_RLL_I, 0.13),           //copter->attitude_control->_pid_rate_roll._ki
    PARAM_FLOAT(ATC_RAT_RLL_IMAX, 0.5),         //copter->attitude_control->_pid_rate_roll._imax
    PARAM_FLOAT(ATC_RAT_RLL_P, 0.18),           //copter->attitude_control->_pid_rate_roll._kp
    PARAM_FLOAT(ATC_RAT_YAW_D, 0.028),          //copter->attitude_control->_pid_rate_yaw._kd
    PARAM_FLOAT(ATC_RAT_YAW_FF, 0),             //copter->attitude_control->_pid_rate_yaw._ff
    PARAM_FLOAT(ATC_RAT_YAW_FILT, 2.5),         //copter->attitude_control->_pid_rate_yaw._filt_hz
    PARAM_FLOAT(ATC_RAT_YAW_I, 0.1),            //copter->attitude_control->_pid_rate_yaw._ki
    PARAM_FLOAT(ATC_RAT_YAW_IMAX, 0.5),         //copter->attitude_control->_pid_rate_yaw._imax
    PARAM_FLOAT(ATC_RAT_YAW_P, 0.35),           //copter->attitude_control->_pid_rate_yaw._kp
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
    PARAM_INT8(FLOW_ENABLE, 0),                 //copter->optflow._enabled
    PARAM_INT16(FLOW_FXSCALER, 0),              //copter->optflow._flowScalerX
    PARAM_INT16(FLOW_FYSCALER, 0),              //copter->optflow._flowScalerY
    PARAM_INT16(FLOW_ORIENT_YAW, 0),            //copter->optflow._yawAngle_cd
    PARAM_FLOAT(FLOW_POS_X, 0),                 //copter->optflow._pos_offset.x
    PARAM_FLOAT(FLOW_POS_Y, 0),                 //copter->optflow._pos_offset.y
    PARAM_FLOAT(FLOW_POS_Z, 0),                 //copter->optflow._pos_offset.z
    PARAM_INT8(FRAME_CLASS, 1),                 //copter->g2.frame_class
    PARAM_INT8(FRAME_TYPE, 1),                  //copter->g.frame_type
    PARAM_FLOAT(FS_BATT_ENABLE, 0),
    PARAM_FLOAT(FS_BATT_ENABLE2, 0),
    PARAM_FLOAT(FS_BATT_MAH, 0),
    PARAM_FLOAT(FS_BATT_VOLT2, 10.5),
    PARAM_FLOAT(FS_BATT_VOLTAGE, 38.31),
    PARAM_INT8(FS_CRASH_CHECK, 0),              //copter->g.fs_crash_check
    PARAM_INT8(FS_GCS_ENABLE, 0),               //copter->g.failsafe_gcs
    PARAM_INT8(FS_THR_ENABLE, 0),               //copter->g.failsafe_throttle
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
    PARAM_FLOAT(MOT_THST_EXPO, 0.6),            //copter->motors->_thrust_curve_expo
    PARAM_FLOAT(MOT_THST_HOVER, 0.3223571),     //copter->motors->_throttle_hover
    PARAM_INT16(MOT_YAW_HEADROOM, 200),         //copter->motors->_yaw_headroom
    PARAM_INT16(PHLD_BRAKE_ANGLE, 3000),        //copter->g.poshold_brake_angle_max
    PARAM_INT16(PHLD_BRAKE_RATE, 8),            //copter->g.poshold_brake_rate
    PARAM_INT16(PILOT_ACCEL_Z, 250),            //copter->g.pilot_accel_z
    PARAM_INT16(PILOT_THR_BHV, 0),              //copter->g.throttle_behavior
    PARAM_FLOAT(PILOT_THR_FILT, 0),             //copter->g.throttle_filt
    PARAM_FLOAT(PILOT_TKOFF_ALT, 200),          //copter->g.pilot_takeoff_alt
    PARAM_INT16(PILOT_TKOFF_DZ, 100),           //copter->g.takeoff_trigger_dz
    PARAM_INT16(PILOT_VELZ_MAX, 250),           //copter->g.pilot_velocity_z_max
    PARAM_FLOAT(POS_XY_P, 0.85),                //copter->pos_control->_p_pos_xy._kp
    PARAM_FLOAT(POS_Z_P, 1),                    //copter->pos_control->_p_pos_z._kp
    PARAM_INT8(RC_FEEL_RP, 60),                 //copter->g.rc_feel_rp
    PARAM_INT16(RC1_DZ, 20),                    //copter->g2.rc_channels.rc_channel(0)->dead_zone
    PARAM_INT16(RC1_MAX, 1950),                 //copter->g2.rc_channels.rc_channel(0)->radio_max
    PARAM_INT16(RC1_MIN, 1051),                 //copter->g2.rc_channels.rc_channel(0)->radio_min
    PARAM_INT8(RC1_REVERSED, 0),                //copter->g2.rc_channels.rc_channel(0)->reversed
    PARAM_INT16(RC1_TRIM, 1500),                //copter->g2.rc_channels.rc_channel(0)->radio_trim
    PARAM_INT16(RC2_DZ, 20),                    //copter->g2.rc_channels.rc_channel(2)->dead_zone
    PARAM_INT16(RC2_MAX, 1950),                 //copter->g2.rc_channels.rc_channel(2)->radio_max
    PARAM_INT16(RC2_MIN, 1051),                 //copter->g2.rc_channels.rc_channel(2)->radio_min
    PARAM_INT8(RC2_REVERSED, 0),                //copter->g2.rc_channels.rc_channel(2)->reversed
    PARAM_INT16(RC2_TRIM, 1500),                //copter->g2.rc_channels.rc_channel(2)->radio_trim
    PARAM_INT16(RC3_DZ, 20),                    //copter->g2.rc_channels.rc_channel(2)->dead_zone
    PARAM_INT16(RC3_MAX, 1950),                 //copter->g2.rc_channels.rc_channel(2)->radio_max
    PARAM_INT16(RC3_MIN, 1051),                 //copter->g2.rc_channels.rc_channel(2)->radio_min
    PARAM_INT8(RC3_REVERSED, 0),                //copter->g2.rc_channels.rc_channel(2)->reversed
    PARAM_INT16(RC3_TRIM, 1500),                //copter->g2.rc_channels.rc_channel(2)->radio_trim
    PARAM_INT16(RC4_DZ, 20),                    //copter->g2.rc_channels.rc_channel(3)->dead_zone
    PARAM_INT16(RC4_MAX, 1950),                 //copter->g2.rc_channels.rc_channel(3)->radio_max
    PARAM_INT16(RC4_MIN, 1051),                 //copter->g2.rc_channels.rc_channel(3)->radio_min
    PARAM_INT8(RC4_REVERSED, 0),                //copter->g2.rc_channels.rc_channel(3)->reversed
    PARAM_INT16(RC4_TRIM, 1500),                //copter->g2.rc_channels.rc_channel(3)->radio_trim
    PARAM_INT16(RC5_DZ, 20),                    //copter->g2.rc_channels.rc_channel(4)->dead_zone
    PARAM_INT16(RC5_MAX, 1950),                 //copter->g2.rc_channels.rc_channel(4)->radio_max
    PARAM_INT16(RC5_MIN, 1051),                 //copter->g2.rc_channels.rc_channel(4)->radio_min
    PARAM_INT8(RC5_REVERSED, 0),                //copter->g2.rc_channels.rc_channel(4)->reversed
    PARAM_INT16(RC5_TRIM, 1500),                //copter->g2.rc_channels.rc_channel(4)->radio_trim
    PARAM_INT16(RC6_DZ, 20),                    //copter->g2.rc_channels.rc_channel(5)->dead_zone
    PARAM_INT16(RC6_MAX, 1950),                 //copter->g2.rc_channels.rc_channel(5)->radio_max
    PARAM_INT16(RC6_MIN, 1051),                 //copter->g2.rc_channels.rc_channel(5)->radio_min
    PARAM_INT8(RC6_REVERSED, 0),                //copter->g2.rc_channels.rc_channel(5)->reversed
    PARAM_INT16(RC6_TRIM, 1500),                //copter->g2.rc_channels.rc_channel(5)->radio_trim
    PARAM_INT16(RC7_DZ, 20),                    //copter->g2.rc_channels.rc_channel(6)->dead_zone
    PARAM_INT16(RC7_MAX, 1950),                 //copter->g2.rc_channels.rc_channel(6)->radio_max
    PARAM_INT16(RC7_MIN, 1051),                 //copter->g2.rc_channels.rc_channel(6)->radio_min
    PARAM_INT8(RC7_REVERSED, 0),                //copter->g2.rc_channels.rc_channel(6)->reversed
    PARAM_INT16(RC7_TRIM, 1500),                //copter->g2.rc_channels.rc_channel(6)->radio_trim
    PARAM_INT16(RC8_DZ, 20),                    //copter->g2.rc_channels.rc_channel(7)->dead_zone
    PARAM_INT16(RC8_MAX, 1950),                 //copter->g2.rc_channels.rc_channel(7)->radio_max
    PARAM_INT16(RC8_MIN, 1051),                 //copter->g2.rc_channels.rc_channel(7)->radio_min
    PARAM_INT8(RC8_REVERSED, 0),                //copter->g2.rc_channels.rc_channel(7)->reversed
    PARAM_INT16(RC8_TRIM, 1500),                //copter->g2.rc_channels.rc_channel(7)->radio_trim
    PARAM_INT16(RC9_DZ, 20),                    //copter->g2.rc_channels.rc_channel(8)->dead_zone
    PARAM_INT16(RC9_MAX, 1950),                 //copter->g2.rc_channels.rc_channel(8)->radio_max
    PARAM_INT16(RC9_MIN, 1051),                 //copter->g2.rc_channels.rc_channel(8)->radio_min
    PARAM_INT8(RC9_REVERSED, 0),                //copter->g2.rc_channels.rc_channel(8)->reversed
    PARAM_INT16(RC9_TRIM, 1500),                //copter->g2.rc_channels.rc_channel(8)->radio_trim
    PARAM_INT16(RC10_DZ, 20),                   //copter->g2.rc_channels.rc_channel(9)->dead_zone
    PARAM_INT16(RC10_MAX, 1950),                //copter->g2.rc_channels.rc_channel(9)->radio_max
    PARAM_INT16(RC10_MIN, 1051),                //copter->g2.rc_channels.rc_channel(9)->radio_min
    PARAM_INT8(RC10_REVERSED, 0),               //copter->g2.rc_channels.rc_channel(9)->reversed
    PARAM_INT16(RC10_TRIM, 1500),               //copter->g2.rc_channels.rc_channel(9)->radio_trim
    PARAM_INT16(RC11_DZ, 20),                   //copter->g2.rc_channels.rc_channel(10)->dead_zone
    PARAM_INT16(RC11_MAX, 1950),                //copter->g2.rc_channels.rc_channel(10)->radio_max
    PARAM_INT16(RC11_MIN, 1051),                //copter->g2.rc_channels.rc_channel(10)->radio_min
    PARAM_INT8(RC11_REVERSED, 0),               //copter->g2.rc_channels.rc_channel(10)->reversed
    PARAM_INT16(RC11_TRIM, 1500),               //copter->g2.rc_channels.rc_channel(10)->radio_trim
    PARAM_INT16(RC12_DZ, 20),                   //copter->g2.rc_channels.rc_channel(11)->dead_zone
    PARAM_INT16(RC12_MAX, 1950),                //copter->g2.rc_channels.rc_channel(11)->radio_max
    PARAM_INT16(RC12_MIN, 1051),                //copter->g2.rc_channels.rc_channel(11)->radio_min
    PARAM_INT8(RC12_REVERSED, 0),               //copter->g2.rc_channels.rc_channel(11)->reversed
    PARAM_INT16(RC12_TRIM, 1500),               //copter->g2.rc_channels.rc_channel(11)->radio_trim
    PARAM_INT16(RC13_DZ, 20),                   //copter->g2.rc_channels.rc_channel(12)->dead_zone
    PARAM_INT16(RC13_MAX, 1950),                //copter->g2.rc_channels.rc_channel(12)->radio_max
    PARAM_INT16(RC13_MIN, 1051),                //copter->g2.rc_channels.rc_channel(12)->radio_min
    PARAM_INT8(RC13_REVERSED, 0),               //copter->g2.rc_channels.rc_channel(12)->reversed
    PARAM_INT16(RC13_TRIM, 1500),               //copter->g2.rc_channels.rc_channel(12)->radio_trim
    PARAM_INT16(RC14_DZ, 20),                   //copter->g2.rc_channels.rc_channel(13)->dead_zone
    PARAM_INT16(RC14_MAX, 1950),                //copter->g2.rc_channels.rc_channel(13)->radio_max
    PARAM_INT16(RC14_MIN, 1051),                //copter->g2.rc_channels.rc_channel(13)->radio_min
    PARAM_INT8(RC14_REVERSED, 0),               //copter->g2.rc_channels.rc_channel(13)->reversed
    PARAM_INT16(RC14_TRIM, 1500),               //copter->g2.rc_channels.rc_channel(13)->radio_trim
    PARAM_INT16(RC15_DZ, 20),                   //copter->g2.rc_channels.rc_channel(14)->dead_zone
    PARAM_INT16(RC15_MAX, 1950),                //copter->g2.rc_channels.rc_channel(14)->radio_max
    PARAM_INT16(RC15_MIN, 1051),                //copter->g2.rc_channels.rc_channel(14)->radio_min
    PARAM_INT8(RC15_REVERSED, 0),               //copter->g2.rc_channels.rc_channel(14)->reversed
    PARAM_INT16(RC15_TRIM, 1500),               //copter->g2.rc_channels.rc_channel(14)->radio_trim
    PARAM_INT16(RC16_DZ, 20),                   //copter->g2.rc_channels.rc_channel(15)->dead_zone
    PARAM_INT16(RC16_MAX, 1950),                //copter->g2.rc_channels.rc_channel(15)->radio_max
    PARAM_INT16(RC16_MIN, 1051),                //copter->g2.rc_channels.rc_channel(15)->radio_min
    PARAM_INT8(RC16_REVERSED, 0),               //copter->g2.rc_channels.rc_channel(15)->reversed
    PARAM_INT16(RC16_TRIM, 1500),               //copter->g2.rc_channels.rc_channel(15)->radio_trim
    PARAM_FLOAT(RNGFND_ADDR, 0),
    PARAM_FLOAT(RNGFND_FUNCTION, 0),
    PARAM_FLOAT(RNGFND_GAIN, 0.8),
    PARAM_FLOAT(RNGFND_GNDCLEAR, 10),
    PARAM_FLOAT(RNGFND_MAX_CM, 3000),
    PARAM_FLOAT(RNGFND_MIN_CM, 80),
    PARAM_FLOAT(RNGFND_OFFSET, 0),
    PARAM_FLOAT(RNGFND_ORDER, 0),
    PARAM_FLOAT(RNGFND_ORIENT, 25),
    PARAM_FLOAT(RNGFND_PIN, -1),
    PARAM_FLOAT(RNGFND_POS_X, 0),
    PARAM_FLOAT(RNGFND_POS_Y, 0),
    PARAM_FLOAT(RNGFND_POS_Z, 0),
    PARAM_FLOAT(RNGFND_PWRRNG, 0),
    PARAM_FLOAT(RNGFND_RMETRIC, 0),
    PARAM_FLOAT(RNGFND_SCALING, 0),
    PARAM_FLOAT(RNGFND_SETTLE, 0),
    PARAM_FLOAT(RNGFND_STOP_PIN, -1),
    PARAM_FLOAT(RNGFND_TYPE, 0),
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
    PARAM_INT8(WP_YAW_BEHAVIOR, 0),             //copter->g.wp_yaw_behavior
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

#if FMT_READ_RADIO == 1
static McnNode_t rc_channels_nod;
#endif

static McnNode_t ins_out_nod;
static McnNode_t mission_data_nod;
// static McnNode_t pilot_cmd_nod;
static McnNode_t gcs_cmd_nod;

uint8_t apm_pilot_cmd_updated = 1;
uint8_t apm_gcs_cmd_updated = 1;
uint8_t apm_mission_data_updated = 1;

uint8_t apm_pilot_cmd_log = 0;
uint8_t apm_gcs_cmd_log = 0;
uint8_t apm_mission_data_log = 0;
// static McnNode_t my_radio_in_nod;

// static int Pilot_Cmd_ID;
// static int GCS_Cmd_ID;
// static int Mission_Data_ID;
// static int FMS_Out_ID;
static char* fms_status[] = {
    "None",
    "Disarm",
    "Standby",
    "Arm"
};

static char* fms_state[] = {
    "None",
    "Disarm",
    "Standby",
    "Offboard",
    "Mission",
    "InvalidAutoMode",
    "Hold",
    "Acro",
    "Stabilize",
    "Altitude",
    "Position",
    "InvalidAssistMode",
    "Manual",
    "InValidManualMode",
    "InvalidArmMode",
    "Land",
    "Return",
    "Takeoff"
};

static char* fms_ctrl_mode[] = {
    "None",
    "Manual",
    "Acro",
    "Stabilize",
    "ALTCTL",
    "POSCTL"
};

static char* fms_mode[] = {
    "None",
    "Manual",
    "Acro",
    "Stabilize",
    "Altitude",
    "Position",
    "Mission",
    "Offboard"
};

// fmt_model_info_t fms_model_info;

static int control_out_echo(void* param)
{
    Control_Out_Bus control_out;
    if (mcn_copy_from_hub((McnHub*)param, &control_out) == FMT_EOK) {
        console_printf("timestamp:%d actuator: %d %d %d %d\n", control_out.timestamp, control_out.actuator_cmd[0], control_out.actuator_cmd[1], control_out.actuator_cmd[2], control_out.actuator_cmd[3]);
    }
    return 0;
}

static int fms_output_echo(void* param)
{
    FMS_Out_Bus fms_out;

    if (mcn_copy_from_hub((McnHub*)param, &fms_out) == FMT_EOK) {
        printf("timestamp:%u\n", fms_out.timestamp);
        printf("rate cmd: %.2f %.2f %.2f\n", fms_out.p_cmd, fms_out.q_cmd, fms_out.r_cmd);
        printf("att cmd: %.2f %.2f %.2f\n", fms_out.phi_cmd, fms_out.theta_cmd, fms_out.psi_rate_cmd);
        printf("vel cmd: %.2f %.2f %.2f\n", fms_out.u_cmd, fms_out.v_cmd, fms_out.w_cmd);
        printf("throttle cmd: %.2f\n", fms_out.throttle_cmd);
        printf("act cmd: %u %u %u %u\n", fms_out.actuator_cmd[0], fms_out.actuator_cmd[1], fms_out.actuator_cmd[2], fms_out.actuator_cmd[3]);
        printf("status:%s state:%s ctrl_mode:%s\n", fms_status[fms_out.status], fms_state[fms_out.state], fms_ctrl_mode[fms_out.ctrl_mode]);
        printf("reset:%d mode:%s\n", fms_out.reset, fms_mode[fms_out.mode]);
        printf("wp_current:%d wp_consume:%d\n", fms_out.wp_current, fms_out.wp_consume);
        printf("------------------------------------------\n");
    }

    return 0;
}


// static void mlog_start_cb(void)
// {
//     apm_pilot_cmd_updated = 1;
//     apm_gcs_cmd_updated = 1;
//     apm_mission_data_updated = 1;
// }

// static void init_parameter(void)
// {
//     FMT_CHECK(param_link_variable(PARAM_GET(APM, USER_TEST_P1), &apm_params.user_test_p1));
// }

// static int echo_my_radio_in(void* parameter)
// {
//     fmt_err_t err;
//     my_radio_in rc_chan_val;

//     err = mcn_copy_from_hub((McnHub*)parameter, &rc_chan_val);

//     if (err != FMT_EOK)
//         return -1;

//     uint8_t rc_chan_num = 16;

//     console_printf("rc channel: [");
//     for (int i = 0; i < rc_chan_num; i++) {
//         if (i == rc_chan_num - 1) {
//             console_printf("%d]\n", rc_chan_val.radio_in[i]);
//         } else {
//             console_printf("%d,", rc_chan_val.radio_in[i]);
//         }
//     }

//     return 0;
// }

void apm_interface_step(uint32_t timestamp)
{
    // if (mcn_poll(pilot_cmd_nod)) {
    //     mcn_copy(MCN_HUB(pilot_cmd), pilot_cmd_nod, &FMS_U.Pilot_Cmd);

    //     FMS_U.Pilot_Cmd.timestamp = timestamp;
    //     apm_pilot_cmd_updated = 1;
    // }

    if (mcn_poll(gcs_cmd_nod)) {
        mcn_copy(MCN_HUB(gcs_cmd), gcs_cmd_nod, &gcs_cmd_msg);

        gcs_cmd_msg.timestamp = timestamp;
        apm_gcs_cmd_updated = 1;
        apm_gcs_cmd_log = 1;
    }

    if (mcn_poll(mission_data_nod)) {
        mcn_copy(MCN_HUB(mission_data), mission_data_nod, &mission_data_msg);

        mission_data_msg.timestamp = timestamp;
        apm_mission_data_updated = 1;
        apm_mission_data_log = 1;
    }

    if (mcn_poll(rc_channels_nod)) {
        mcn_copy(MCN_HUB(rc_channels), rc_channels_nod, &rcChannel_msg);
        apm_pilot_cmd_updated = 1;
        apm_pilot_cmd_log = 1;
    }

    if (mcn_poll(ins_out_nod)) {
        mcn_copy(MCN_HUB(ins_output), ins_out_nod, &ins_out_msg);
    }

    // FMS_step();
    APM_loop();

    control_out_msg.timestamp = timestamp;
    mcn_publish(MCN_HUB(control_output), &control_out_msg);
    fms_out_msg.timestamp = timestamp;
    mcn_publish(MCN_HUB(fms_output), &fms_out_msg);

    if (apm_pilot_cmd_log) {
        apm_pilot_cmd_log = 0;
    //     /* Log pilot command */
    //     mlog_push_msg((uint8_t*)&FMS_U.Pilot_Cmd, Pilot_Cmd_ID, sizeof(Pilot_Cmd_Bus));
    }

    if (apm_gcs_cmd_log) {
        apm_gcs_cmd_log = 0;
    //     /* Log gcs command */
    //     mlog_push_msg((uint8_t*)&FMS_U.GCS_Cmd, GCS_Cmd_ID, sizeof(GCS_Cmd_Bus));
    }

    if (apm_mission_data_log) {
        apm_mission_data_log = 0;
    //     /* Log mission data */
    //     mlog_push_msg((uint8_t*)&FMS_U.Mission_Data, Mission_Data_ID, sizeof(Mission_Data_Bus));
    }

    // /* Log FMS output bus data */
    // DEFINE_TIMETAG(fms_output, 100);
    // if (check_timetag(TIMETAG(fms_output))) {
    //     /* Log FMS out data */
    //     mlog_push_msg((uint8_t*)&FMS_Y.FMS_Out, FMS_Out_ID, sizeof(FMS_Out_Bus));
    // }



    // for test purpose, will delete later
    // static int16_t count = 0;
    // for (uint8_t i = 0; i < sizeof(rcChannel_tmp)/2; i++){
    //     rcChannel_tmp[i] = count;
    //     MY_RADIO_BUS.radio_in[i] = count;
    //     // printf("  rcChannel_tmp[%d] = %d", i, rcChannel_tmp[15]);
    // }
    // mcn_publish(MCN_HUB(rc_channels), &rcChannel_tmp);
    // mcn_publish(MCN_HUB(my_radio_in_topic), &MY_RADIO_BUS);
    // count++;
    // my_radio_in rc_chan_val;
    // if (mcn_poll(my_radio_in_nod)) {
    //     mcn_copy(MCN_HUB(my_radio_in_topic), my_radio_in_nod, &rc_chan_val);
    //     // APM_update_rc();
    //     uint8_t i = 15;
    //     // printf("  rc_chan_val.radio_in[%d] = %d\n", i, rc_chan_val.radio_in[i]);
    // }
}


void apm_interface_init(void)
{
    // fms_model_info.period = FMS_EXPORT.period;
    // fms_model_info.info = (char*)FMS_EXPORT.model_info;

    // mcn_advertise(MCN_HUB(fms_output), fms_output_echo);

    // pilot_cmd_nod = mcn_subscribe(MCN_HUB(pilot_cmd), NULL, NULL);
    gcs_cmd_nod = mcn_subscribe(MCN_HUB(gcs_cmd), NULL, NULL);
    mission_data_nod = mcn_subscribe(MCN_HUB(mission_data), NULL, NULL);
    ins_out_nod = mcn_subscribe(MCN_HUB(ins_output), NULL, NULL);
    rc_channels_nod = mcn_subscribe(MCN_HUB(rc_channels), NULL, NULL);

    // Pilot_Cmd_ID = mlog_get_bus_id("Pilot_Cmd");
    // GCS_Cmd_ID = mlog_get_bus_id("GCS_Cmd");
    // Mission_Data_ID = mlog_get_bus_id("Mission_Data");
    // FMS_Out_ID = mlog_get_bus_id("FMS_Out");
    // FMT_ASSERT(Pilot_Cmd_ID >= 0);
    // FMT_ASSERT(GCS_Cmd_ID >= 0);
    // FMT_ASSERT(Mission_Data_ID >= 0);
    // FMT_ASSERT(FMS_Out_ID >= 0);

    // mlog_register_callback(MLOG_CB_START, mlog_start_cb);

    // init_parameter();
    // FMT_CHECK(mcn_advertise(MCN_HUB(rc_channels), echo_my_radio_in)); // should be advertised in pilot_cmd, put here for SIL

    mcn_advertise(MCN_HUB(control_output), control_out_echo);
    mcn_advertise(MCN_HUB(fms_output), fms_output_echo);


    APM_init();

    // for test purpose, will delete later
    // FMT_CHECK(mcn_advertise(MCN_HUB(my_radio_in_topic), echo_my_radio_in));
    // my_radio_in_nod = mcn_subscribe(MCN_HUB(my_radio_in_topic), NULL, NULL);
}