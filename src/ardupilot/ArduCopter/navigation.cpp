#include "Copter.h"

// #include <firmament.h>

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void Copter::run_nav_updates(void)
{
    // calculate distance and bearing for reporting and autopilot decisions
    calc_distance_and_bearing();

    navigation_update();
}

// calc_distance_and_bearing - calculate distance and bearing to next waypoint and home
void Copter::calc_distance_and_bearing()
{
    calc_wp_distance();
    calc_wp_bearing();
    calc_home_distance_and_bearing();
}

// calc_wp_distance - calculate distance to next waypoint for reporting and autopilot decisions
void Copter::calc_wp_distance()
{
    // get target from loiter or wpinav controller
    switch (control_mode) {
    case LOITER:
    case CIRCLE:
        wp_distance = wp_nav->get_loiter_distance_to_target();
        break;

    case AUTO:
    case RTL:
        wp_distance = wp_nav->get_wp_distance_to_destination();
        break;

    case GUIDED:
        if (guided_mode == Guided_WP) {
            wp_distance = wp_nav->get_wp_distance_to_destination();
            break;
        }
        // no break
    default:
        wp_distance = 0;
        break;
    }
}

// calc_wp_bearing - calculate bearing to next waypoint for reporting and autopilot decisions
void Copter::calc_wp_bearing()
{
    // get target from loiter or wpinav controller
    switch (control_mode) {
    case LOITER:
    case CIRCLE:
        wp_bearing = wp_nav->get_loiter_bearing_to_target();
        break;

    case AUTO:
    case RTL:
        wp_bearing = wp_nav->get_wp_bearing_to_destination();
        break;

    case GUIDED:
        if (guided_mode == Guided_WP) {
            wp_bearing = wp_nav->get_wp_bearing_to_destination();
            break;
        }
        // no break
    default:
        wp_bearing = 0;
        break;
    }
}

// calc_home_distance_and_bearing - calculate distance and bearing to home for reporting and autopilot decisions
void Copter::calc_home_distance_and_bearing()
{
    // calculate home distance and bearing
    if (position_ok()) {
        Vector3f home = pv_location_to_vector(ahrs.get_home());
        Vector3f curr = inertial_nav.get_position();
        home_distance = pv_get_horizontal_distance_cm(curr, home);
        home_bearing = pv_get_bearing_cd(curr,home);
    }
}

void Copter::navigation_init() {
    FMT_mission.current_mission_verified = true;
}

void Copter::navigation_update()
{
    if (control_mode != AUTO)
    {
        return;
    }

    if (!motors->armed())
    {
        return;
    }

    // enter the first item
    if (verify_command_callback(copter_current_cmd)) {
        FMT_mission.current_mission_verified = true;
    }

    if (FMT_mission.current_mission_verified && hal.mission_data_updated) {
        navigation_next();
        FMT_mission.current_mission_verified = false;

        hal.fms_out_msg.wp_consume = 1;
        hal.mission_data_log = 1;
        hal.mission_data_updated = 0;
    }
}

void Copter::navigation_next() {
    if (hal.mission_data_msg.valid_items >=1) {
        // do next command
        printf ("Do cmd #%d [%d]\n", hal.mission_data_msg.current[0], hal.mission_data_msg.command[0]);
        AP_Mission::Mission_Command cmd;
        Vector3f tmp_neu;
        Location_Class tmp_loc;
        bool valid_cmd = false;
        switch (hal.mission_data_msg.command[0]) {
            default :
            case NAV_Cmd_None:
                break;
            case NAV_Cmd_Takeoff:
                cmd.id = MAV_CMD_NAV_TAKEOFF;
                cmd.content.location.lat = hal.mission_data_msg.x[0];
                cmd.content.location.lng = hal.mission_data_msg.y[0];
                cmd.content.location.alt = hal.mission_data_msg.z[0]*100.f; //cm
                cmd.content.location.options = 0;
                //if want to set alt from home point add following line
                cmd.content.location.flags.relative_alt = 1; //alt from home point
                cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                valid_cmd = true;
                break;
            case NAV_Cmd_Waypoint:
                cmd.id = MAV_CMD_NAV_WAYPOINT;
                cmd.content.location.lat = hal.mission_data_msg.x[0];
                cmd.content.location.lng = hal.mission_data_msg.y[0];
                cmd.content.location.alt = hal.mission_data_msg.z[0]*100.f; //cm
                cmd.content.location.options = 0; //alt from sea level
                //if want to set alt from home point add following line
                cmd.content.location.flags.relative_alt = 1; //alt from home point
                cmd.p1 = 2;//uint16_t, 悬停时间，单位s，设为0则飞到目标点后会立即判定完成此任务
                valid_cmd = true;
                break;
            case NAV_Cmd_Land:
                cmd.id = MAV_CMD_NAV_LAND;
                //如果经纬度设0，则原地降落，如果不为0，则按照设置的高度飞过去再降落
                cmd.content.location.lat = hal.mission_data_msg.x[0];
                cmd.content.location.lng = hal.mission_data_msg.y[0];
                cmd.content.location.alt = hal.mission_data_msg.z[0]*100.f; //cm
                cmd.content.location.options = 0;
                //if want to set alt from home point add following line
                cmd.content.location.flags.relative_alt = 1; //alt from home point
                valid_cmd = true;
                break;
            case NAV_Cmd_Return:
                cmd.id = MAV_CMD_NAV_RETURN_TO_LAUNCH;
                valid_cmd = true;
                break;

        }
        if (valid_cmd) {
            if (!start_command(cmd)) {
                printf ("Fail to process cmd id[%d]", hal.mission_data_msg.command[0]);
            } 
        } else {
            printf ("Invalid cmd id[%d]", hal.mission_data_msg.command[0]);
        }
    }
}

Vector3f Copter::get_destination() {
    Vector3f dest_pos = Vector3f(0.0f, 0.0f, 0.0f);
    switch (control_mode) {
    case AUTO:
    case RTL:
        dest_pos = wp_nav->get_wp_destination();
        break;

    case GUIDED:
        if (guided_mode == Guided_WP) {
            dest_pos = wp_nav->get_wp_destination();
        } else {
            dest_pos = pos_control->get_pos_target();
        }
        break;
        // no break

    case LOITER:
    case CIRCLE:
        dest_pos = pos_control->get_pos_target();
        break;

    default:
        break;
    }
    return dest_pos;
}