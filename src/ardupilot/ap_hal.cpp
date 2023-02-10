#include "ap_hal.h"

extern uint32_t millis();
extern uint64_t micro64();

extern int16_t RC_in_data[20];
extern int16_t RC_out_data[20];

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
RCInput::RCInput()
{
    memset(_rc_in_data, 0, sizeof(_rc_in_data));
    _new_input = false;
}

void RCInput::init()
{}

bool RCInput::new_input() {
    return _new_input;
}

uint8_t RCInput::num_channels() {
    return 0;
}

uint16_t RCInput::read(uint8_t ch) {
    //put sbus read-out code here.
    return (uint16_t)_rc_in_data[ch];
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    if (len > RC_INPUT_MAX_CHANNELS) {
        len = RC_INPUT_MAX_CHANNELS;
    }
    for (uint8_t i = 0; i < len; i++){
        periods[i] = read(i);
    }
    return len;
}

bool RCInput::set_overrides(int16_t *overrides, uint8_t len) {
    return false;
}

bool RCInput::set_override(uint8_t channel, int16_t override) {
    return false;
}

void RCInput::clear_overrides()
{}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
RCOutput::RCOutput()
{
    memset(_rc_out_data, 0, sizeof(_rc_out_data));
    _new_output = false;
}

void RCOutput::init() {}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {}

uint16_t RCOutput::get_freq(uint8_t ch) {
    return 50;
}

void RCOutput::enable_ch(uint8_t ch)
{}

void RCOutput::disable_ch(uint8_t ch)
{}

void RCOutput::write(uint8_t ch, uint16_t period_us)
{
    _rc_out_data[ch] = (int16_t)period_us;
}

void RCOutput::push()
{
    for (uint8_t i = 0; i < 16; i++){
        hal.control_out_msg.actuator_cmd[i] = (uint16_t)_rc_out_data[i];
        _new_output = true;
    }
}

uint16_t RCOutput::read(uint8_t ch) {
    //send the value last sent
    return (uint16_t)_rc_out_data[ch];
}

void AP_HAL::info() {
    printf("This APM hal\n");
}

void AP_HAL::update() {
    update_inertial();
    update_rc();
    update_mission();
    update_sensors();
}

void AP_HAL::update_inertial() {
    sitl_state.latitude = (int32_t)(degrees(ins_out_msg.lat)*1e7f);  // unit:0.0000001d,inertial_nav
    sitl_state.longitude = (int32_t)(degrees(ins_out_msg.lon)*1e7f); //unit:0.0000001d,inertial_nav
    sitl_state.altitude = ins_out_msg.alt;         // double MSL

    sitl_state.speedN = ins_out_msg.vn; // float m/s, positive value for north
    sitl_state.speedE = ins_out_msg.ve; // float m/s, positive value for east
    sitl_state.speedD = ins_out_msg.vd; // float m/s, positive value for down
       
    sitl_state.xAccel = ins_out_msg.ax; // float m/s/s in body
    sitl_state.yAccel = ins_out_msg.ay; // float m/s/s in body
    sitl_state.zAccel = ins_out_msg.az; // float m/s/s in body   

    sitl_state.rollRate  = degrees(ins_out_msg.p); // float degrees/s in body frame, positive value for roll
    sitl_state.pitchRate = degrees(ins_out_msg.q); // float degrees/s in body frame, positive value for pitch
    sitl_state.yawRate   = degrees(ins_out_msg.r); // float degrees/s in body frame, positive value for turn

    sitl_state.rollDeg   = degrees(ins_out_msg.phi);   // float euler angles, degrees, positive value for roll
    sitl_state.pitchDeg  = degrees(ins_out_msg.theta); // float euler angles, degrees, positive value for pitch
    sitl_state.yawDeg    = wrap_360(degrees(ins_out_msg.psi));   // float euler angles, degrees, positive value for turn
    // euler angle to quanternion
    sitl_state.quaternion.from_euler(radians(sitl_state.rollDeg), radians(sitl_state.pitchDeg), radians(sitl_state.yawDeg));
    sitl_state.position_ok = ins_out_msg.flag&(1<<5);
}

void AP_HAL::update_rc(void)
{
    // trans rc value to copter instance
    // console_printf("hal.apm_pilot_cmd_updated : %d\n", hal.apm_pilot_cmd_updated);
    for (uint8_t i = 0; i < sizeof(rcChannel_msg)/2; i++){
        rcin._rc_in_data[i] = rcChannel_msg[i];
    }
    rcin._new_input = hal.apm_pilot_cmd_updated;
    hal.apm_pilot_cmd_updated = 0;
}

void AP_HAL::update_mission(void)
{
    // reset consume
    fms_out_msg.wp_consume = 0;
}

void AP_HAL::update_sensors(void)
{
    ;
}
