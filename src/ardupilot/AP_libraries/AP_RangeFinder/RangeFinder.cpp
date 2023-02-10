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

#include "RangeFinder.h"
#include "AP_RangeFinder_FMT.h"

#include "ap_hal.h"

RangeFinder::RangeFinder(enum Rotation orientation_default) :
    num_instances(0),
    estimated_terrain_height(0)
{
    // set orientation defaults
    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        _orientation[i].set_default(orientation_default);
    }

    // init state and drivers
    memset(state,0,sizeof(state));
    memset(drivers,0,sizeof(drivers));
}

/*
  initialise the RangeFinder class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  rangefinders.
 */
void RangeFinder::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        detect_instance(i);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }
        // initialise pre-arm check variables
        state[i].pre_arm_check = false;
        state[i].pre_arm_distance_min = 9999;  // initialise to an arbitrary large value
        state[i].pre_arm_distance_max = 0;

        // initialise status
        state[i].status = RangeFinder_NotConnected;
        state[i].range_valid_count = 0;
    }
}

/*
  update RangeFinder state for all instances. This should be called at
  around 10Hz by main loop
 */
void RangeFinder::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (_type[i] == RangeFinder_TYPE_NONE) {
                // allow user to disable a rangefinder at runtime
                state[i].status = RangeFinder_NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
            drivers[i]->update();
            update_pre_arm_check(i);
        }
    }
}

bool RangeFinder::_add_backend(AP_RangeFinder_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_instances == RANGEFINDER_MAX_INSTANCES) {
        AP_HAL::panic("Too many RANGERS backends");
    }

    drivers[num_instances++] = backend;
    return true;
}

/*
  detect if an instance of a rangefinder is connected. 
 */
void RangeFinder::detect_instance(uint8_t instance)
{
    enum RangeFinder_Type type = (enum RangeFinder_Type)_type[instance].get();
    switch (type) {
    case RangeFinder_TYPE_FMT:
        if (AP_RangeFinder_PX4_PWM::detect(*this, instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_RangeFinder_PX4_PWM(*this, instance, state[instance]);
        }
        break;
    default:
        break;
    }
}

// query status
RangeFinder::RangeFinder_Status RangeFinder::status(uint8_t instance) const
{
    // sanity check instance
    if (instance >= RANGEFINDER_MAX_INSTANCES) {
        return RangeFinder_NotConnected;
    }

    if (drivers[instance] == nullptr || _type[instance] == RangeFinder_TYPE_NONE) {
        return RangeFinder_NotConnected;
    }

    return state[instance].status;
}

RangeFinder::RangeFinder_Status RangeFinder::status_orient(enum Rotation orientation) const
{
    uint8_t i;
    if (find_instance(orientation, i)) {
        return status(i);
    }
    return RangeFinder_NotConnected;
}

void RangeFinder::handle_msg(mavlink_message_t *msg)
{
    uint8_t i;
    for (i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && (_type[i] != RangeFinder_TYPE_NONE)) {
          drivers[i]->handle_msg(msg);
        }
    }
}

// return true if we have a range finder with the specified orientation
bool RangeFinder::has_orientation(enum Rotation orientation) const
{
    uint8_t i;
    return find_instance(orientation, i);
}

// find first range finder instance with the specified orientation
bool RangeFinder::find_instance(enum Rotation orientation, uint8_t &instance) const
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (_orientation[i] == orientation) {
            instance = i;
            return true;
        }
    }
    return false;
}

uint16_t RangeFinder::distance_cm_orient(enum Rotation orientation) const
{
    uint8_t i;
    if (find_instance(orientation, i)) {
        return distance_cm(i);
    }
    return 0;
}

uint16_t RangeFinder::voltage_mv_orient(enum Rotation orientation) const
{
    uint8_t i;
    if (find_instance(orientation, i)) {
        return voltage_mv(i);
    }
    return 0;
}

int16_t RangeFinder::max_distance_cm_orient(enum Rotation orientation) const
{
    uint8_t i;
    if (find_instance(orientation, i)) {
        return max_distance_cm(i);
    }
    return 0;
}

int16_t RangeFinder::min_distance_cm_orient(enum Rotation orientation) const
{
    uint8_t i;
    if (find_instance(orientation, i)) {
        return min_distance_cm(i);
    }
    return 0;
}

int16_t RangeFinder::ground_clearance_cm_orient(enum Rotation orientation) const
{
    uint8_t i;
    if (find_instance(orientation, i)) {
        return ground_clearance_cm(i);
    }
    return 0;
}

// true if sensor is returning data
bool RangeFinder::has_data(uint8_t instance) const
{
    // sanity check instance
    if (instance >= RANGEFINDER_MAX_INSTANCES) {
        return RangeFinder_NotConnected;
    }
    return ((state[instance].status != RangeFinder_NotConnected) && (state[instance].status != RangeFinder_NoData));
}

bool RangeFinder::has_data_orient(enum Rotation orientation) const
{
    uint8_t i;
    if (find_instance(orientation, i)) {
        return has_data(i);
    }
    return false;
}

uint8_t RangeFinder::range_valid_count_orient(enum Rotation orientation) const
{
    uint8_t i;
    if (find_instance(orientation, i)) {
        return range_valid_count(i);
    }
    return 0;
}

/*
  returns true if pre-arm checks have passed for all range finders
  these checks involve the user lifting or rotating the vehicle so that sensor readings between
  the min and 2m can be captured
 */
bool RangeFinder::pre_arm_check() const
{
    for (uint8_t i=0; i<num_instances; i++) {
        // if driver is valid but pre_arm_check is false, return false
        if ((drivers[i] != nullptr) && (_type[i] != RangeFinder_TYPE_NONE) && !state[i].pre_arm_check) {
            return false;
        }
    }
    return true;
}

/*
  set pre-arm checks to passed if the range finder has been exercised through a reasonable range of movement
      max distance sensed is at least 50cm > min distance sensed
      max distance < 200cm
      min distance sensed is within 10cm of ground clearance or sensor's minimum distance
 */
void RangeFinder::update_pre_arm_check(uint8_t instance)
{
    // return immediately if already passed or no sensor data
    if (state[instance].pre_arm_check || state[instance].status == RangeFinder_NotConnected || state[instance].status == RangeFinder_NoData) {
        return;
    }

    // update min, max captured distances
    state[instance].pre_arm_distance_min = MIN(state[instance].distance_cm, state[instance].pre_arm_distance_min);
    state[instance].pre_arm_distance_max = MAX(state[instance].distance_cm, state[instance].pre_arm_distance_max);

    // Check that the range finder has been exercised through a realistic range of movement
    if (((state[instance].pre_arm_distance_max - state[instance].pre_arm_distance_min) > RANGEFINDER_PREARM_REQUIRED_CHANGE_CM) &&
         (state[instance].pre_arm_distance_max < RANGEFINDER_PREARM_ALT_MAX_CM) &&
         ((int16_t)state[instance].pre_arm_distance_min < (MAX(_ground_clearance_cm[instance],min_distance_cm(instance)) + 10)) &&
         ((int16_t)state[instance].pre_arm_distance_min > (MIN(_ground_clearance_cm[instance],min_distance_cm(instance)) - 10))) {
        state[instance].pre_arm_check = true;
    }
}

const Vector3f &RangeFinder::get_pos_offset_orient(enum Rotation orientation) const
{
    uint8_t i=0;
    if (find_instance(orientation, i)) {
        return get_pos_offset(i);
    }
    return pos_offset_zero;
}
