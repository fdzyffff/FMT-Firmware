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
    for (uint8_t i = 0; i < 18; i++){
        RC_out_data[i] = (int16_t)_rc_out_data[i];
        _new_output = true;
    }
}

uint16_t RCOutput::read(uint8_t ch) {
    //send the value last sent
    return (uint16_t)_rc_out_data[ch];
}