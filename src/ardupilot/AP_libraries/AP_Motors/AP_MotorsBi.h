/// @file	AP_MotorsBi.h
/// @brief	Motor control class for Tricopters
#pragma once

#include "AP_Common.h"
#include "AP_Math.h"        // ArduPilot Mega Vector/Matrix math Library
#include "SRV_Channel.h"
#include "AP_MotorsMulticopter.h"

// tail servo channels
#define AP_MOTORS_CH_BI_1    CH_3
#define AP_MOTORS_CH_BI_2    CH_4

#define AP_MOTORS_BI_SERVO_RANGE_DEG_MIN   15   // minimum angle movement of tail servo in degrees
#define AP_MOTORS_BI_SERVO_RANGE_DEG_MAX   80  // maximum angle movement of tail servo in degrees

/// @class      AP_MotorsBi
class AP_MotorsBi : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsBi(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) 
        :  AP_MotorsMulticopter(loop_rate, speed_hz)
    {
    };

    // init
    void                init(motor_frame_class frame_class, motor_frame_type frame_type);

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type);

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz );

    // enable - starts allowing signals to be sent to motors
    virtual void        enable();

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        output_test(uint8_t motor_seq, int16_t pwm);

    // output_to_motors - sends minimum values out to the motors
    virtual void        output_to_motors();

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint16_t    get_motor_mask();

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing();

    // call vehicle supplied thrust compensation if set
    // void                thrust_compensation(void) override;
    
    // calc_yaw_radio_output - calculate final radio output for yaw channel
    int16_t             calc_servo_radio_output(SRV_Channel* servo_channel_in, float yaw_input, float yaw_input_max);        // calculate radio output for yaw servo, typically in range of 1100-1900

    // parameters

    SRV_Channel     *_yaw_servo_left;  // yaw output channel
    SRV_Channel     *_yaw_servo_right; // yaw output channel
    float           _thrust_right;
    float           _thrust_left;
    float           _servo_angle_right;
    float           _servo_angle_left;
};
