#include "Copter.h"

void Copter::user_ten_hz_loop()
{
	int16_t tmp_in = RC_Channels::rc_channel(CH_8)->get_control_in();
	g2.servo_channels.set_output_scaled(SRV_Channel::k_servo_biarm, tmp_in);

	if (hal.rcin.read(6) > 1500) {
		set_mode(control_mode_t::ALT_HOLD, MODE_REASON_GCS_COMMAND);
	} 
	if (hal.rcin.read(7) > 1500) {
		set_mode(control_mode_t::LOITER, MODE_REASON_GCS_COMMAND);
	}
	if (hal.rcin.read(10) > 1500) {
		set_mode(control_mode_t::STABILIZE, MODE_REASON_GCS_COMMAND);
	}
	if (hal.rcin.read(11) > 1500) {
		set_mode(control_mode_t::FLOW_HOLD, MODE_REASON_GCS_COMMAND);
	}
}
