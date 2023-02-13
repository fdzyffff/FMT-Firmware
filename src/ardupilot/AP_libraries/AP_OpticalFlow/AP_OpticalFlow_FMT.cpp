/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * AP_OpticalFlow_FMT.cpp - FMT optical flow sensor.
 */

#include "ap_hal.h"

#include "AP_OpticalFlow_FMT.h"

AP_OpticalFlow_FMT::AP_OpticalFlow_FMT(OpticalFlow &_frontend) : 
    OpticalFlow_backend(_frontend) 
{
    ;
}

void AP_OpticalFlow_FMT::init(void)
{
}

void AP_OpticalFlow_FMT::update(void)
{
    if (hal.optflow_out_msg_update) {
        OpticalFlow::OpticalFlow_state state;

        state.device_id = 1;
        state.surface_quality = hal.optflow_out_msg.quality;

        // poll to provide a delta angle across the interface
        state.flowRate.x =   hal.optflow_out_msg.vx_mPs;
        state.flowRate.y =   hal.optflow_out_msg.vy_mPs;

        // The flow sensors body rates are assumed to be the same as the vehicle body rates (ie no misalignment)
        // Note - these are instantaneous values. The sensor sums these values across the interval from the last
        // poll to provide a delta angle across the interface.
        state.bodyRate = Vector2f(hal.ins_out_msg.p, ins_out_msg.q); // in radians

        _applyYaw(state.flowRate);
        
        // copy results to front end
        _update_frontend(state);
    }
}
