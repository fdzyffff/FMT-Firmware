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


#include "AP_RangeFinder_FMT.h"
#include "ap_hal.h"

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_RangeFinder_FMT::AP_RangeFinder_FMT(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state):
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    set_status(RangeFinder::RangeFinder_NoData);    
}

/*
  update distance_cm 
 */
void AP_RangeFinder_FMT::update(void)
{
    uint32_t t_last = hal.rangefinder_data_msg.timestamp_ms;
    uint32_t tnow = millis();

    // console_printf("TT hal.rangefinder_data_msg.distance_m %f\n", hal.rangefinder_data_msg.distance_m);

    if (tnow - t_last < 1000) {
        state.distance_cm = (hal.rangefinder_data_msg.distance_m*90.f);
    } else {
        state.distance_cm = 0;
    }

    update_status();
}
