#ifndef AP_HAL_H
#define AP_HAL_H

#include "AP_Math.h"
#include <firmament.h>
#include "APM.h"

#pragma once

#define RC_OUTPUT_MIN_PULSEWIDTH 400
#define RC_OUTPUT_MAX_PULSEWIDTH 2100

#ifndef RC_INPUT_MAX_CHANNELS
#define RC_INPUT_MAX_CHANNELS 18
#endif

/* Define the CH_n names, indexed from 1, if we don't have them already */
#ifndef CH_1
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_9 8
#define CH_10 9
#define CH_11 10
#define CH_12 11
#define CH_13 12
#define CH_14 13
#define CH_15 14
#define CH_16 15
#define CH_17 16
#define CH_18 17
#define CH_NONE 255
#endif

extern uint32_t millis();
extern uint64_t micro64();


struct sitl_fdm {
    // this is the structure passed between FDM models and the main SITL code
    uint64_t timestamp_us;
    int32_t latitude, longitude; // degrees
    float altitude;  // MSL
    float heading;   // degrees
    float speedN, speedE, speedD; // m/s
    float xAccel, yAccel, zAccel;       // m/s/s in body frame
    float rollRate, pitchRate, yawRate; // degrees/s/s in body frame
    float rollDeg, pitchDeg, yawDeg;    // euler angles, degrees
    Quaternion quaternion;
    float airspeed; // m/s
    float battery_voltage; // Volts
    float battery_current; // Amps
    float rpm1;            // main prop RPM
    float rpm2;            // secondary RPM
    uint8_t rcin_chan_count;
    float  rcin[8];         // RC input 0..1
    Vector3f bodyMagField;  // Truth XYZ magnetic field vector in body-frame. Includes motor interference. Units are milli-Gauss.
    Vector3f angAccel; // Angular acceleration in degrees/s/s about the XYZ body axes
    uint8_t position_ok;
};

struct my_temp_log_t {
    float pos_x;
    float pos_y;
    float pos_z;
    float pos_x_des;
    float pos_y_des;
    float pos_z_des;
    float vel_x_des;  //x速度cm/s
    float vel_y_des;  //y速度cm/s
    float vel_xy_des;  //水平速度cm/s
    float vel_z_des;  //升降速度cm/s
	
    float ang_roll;
    float ang_roll_des;
    float ang_pitch;
    float ang_pitch_des;
    float ang_yaw;
    float ang_yaw_des;

    float rate_roll;
    float rate_roll_des;
    float rate_pitch;
    float rate_pitch_des;
    float rate_yaw;
    float rate_yaw_des;

    float rate_roll_kP;
    float rate_roll_kI;
    float rate_roll_kD;

    float rate_pitch_kP;
    float rate_pitch_kI;
    float rate_pitch_kD;

    float rate_yaw_kP;
    float rate_yaw_kI;
    float rate_yaw_kD;

    int16_t rc1_in;
    int16_t rc2_in;
    int16_t rc3_in;
    int16_t rc4_in;
    int16_t rc5_in;

    int16_t rc1_out;
    int16_t rc2_out;
    int16_t rc3_out;
    int16_t rc4_out;

    int8_t my_flt_mode;
    int16_t throttle_out;
    int16_t roll_out;
    int16_t pitch_out;
    int16_t yaw_out;      
	
    uint16_t current_cmd_id;
    uint8_t land_complete_state;
    int8_t my_home_state;
    struct Location my_home_loc;
    int8_t my_version;
};

class RCInput {
public:
    RCInput();
    void init();
    bool  new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();
    void set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) {}
    int16_t _rc_in_data[20];
    bool _new_input;
};

class RCOutput {
public:
    RCOutput() ;
    void     init();
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);
    uint16_t read_last_sent(uint8_t ch) { return read(ch); }
    void     set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) {}
    void     set_safety_pwm(uint32_t chmask, uint16_t period_us) {}
    void     set_failsafe_pwm(uint32_t chmask, uint16_t period_us) {}
    void     cork(void)  {_new_output = false;}
    void     push(void);
    uint16_t  _rc_out_data[20];
    bool _new_output;
};

class Sensors {
public:
    Sensors();
}

class AP_HAL{
public:
    RCInput rcin;
    RCOutput rcout;
    sitl_fdm sitl_state;
    Sensors sensors;

    // interface to FMT Bus, copy in apm_copter_wrapper.cpp
    int16_t rcChannel_msg[16];
    INS_Out_Bus ins_out_msg;
    Mission_Data_Bus mission_data_msg;
    FMS_Out_Bus fms_out_msg;
    Control_Out_Bus control_out_msg;
    GCS_Cmd_Bus gcs_cmd_msg;
    Rangefinder_Bus rangefinder_data_msg;

    uint8_t apm_pilot_cmd_updated;
    uint8_t apm_gcs_cmd_updated;
    uint8_t apm_mission_data_updated;
    
    uint8_t apm_pilot_cmd_log;
    uint8_t apm_gcs_cmd_log;
    uint8_t apm_mission_data_log;

    bool get_soft_armed() {return false;}
    uint32_t micros() {return (uint32_t)micro64();}
    void info();
    void update();
    void update_rc();
    void update_mission();
    void update_inertial();
    void update_sensors();
};

extern AP_HAL hal;

#endif
