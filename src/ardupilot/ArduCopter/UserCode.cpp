#include "Copter.h"

void Copter::user_ten_hz_loop()
{
	int16_t tmp_in = RC_Channels::rc_channel(CH_8)->get_control_in();
	g2.servo_channels.set_output_scaled(SRV_Channel::k_servo_biarm, tmp_in);
}