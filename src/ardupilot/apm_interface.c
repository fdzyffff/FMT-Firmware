/******************************************************************************
 * Copyright 2020 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <APM.h>
#include <firmament.h>
#include <string.h>

#include "module/log/mlog.h"
#include "module/param/param.h"
#include "apm_interface_param.h"

// FMS input topic
// MCN_DECLARE(pilot_cmd);
MCN_DECLARE(gcs_cmd);
MCN_DECLARE(mission_data);
MCN_DECLARE(ins_output);
MCN_DECLARE(control_output);
MCN_DECLARE(rc_channels);
MCN_DECLARE(sensor_rangefinder);
MCN_DECLARE(sensor_optflow);


/* controller output topic */
MCN_DEFINE(control_output, sizeof(Control_Out_Bus));
MCN_DEFINE(fms_output, sizeof(FMS_Out_Bus));

// MCN_DEFINE(auto_cmd, sizeof(Auto_Cmd_Bus));
// MCN_DEFINE(fms_output, sizeof(FMS_Out_Bus));
// MCN_DEFINE(control_output, sizeof(Control_Out_Bus));


// rf_data_t rf_report;


// static int16_t rcChannel_tmp[16];
// static my_radio_in MY_RADIO_BUS;
// MCN_DEFINE(my_radio_in_topic, sizeof(MY_RADIO_BUS));
APM_Handler apm_handler;

// static McnNode_t my_radio_in_nod;

// static int Pilot_Cmd_ID;
// static int GCS_Cmd_ID;
// static int Mission_Data_ID;
// static int FMS_Out_ID;

// fmt_model_info_t fms_model_info;


static char* fms_status[] = {
    "None",
    "Disarm",
    "Standby",
    "Arm"
};

static char* fms_state[] = {
    "None",
    "Disarm",
    "Standby",
    "Offboard",
    "Mission",
    "InvalidAutoMode",
    "Hold",
    "Acro",
    "Stabilize",
    "Altitude",
    "Position",
    "InvalidAssistMode",
    "Manual",
    "InValidManualMode",
    "InvalidArmMode",
    "Land",
    "Return",
    "Takeoff"
};

static char* fms_ctrl_mode[] = {
    "None",
    "Manual",
    "Acro",
    "Stabilize",
    "ALTCTL",
    "POSCTL"
};

static char* fms_mode[] = {
    "None",
    "Manual",
    "Acro",
    "Stabilize",
    "Altitude",
    "Position",
    "Mission",
    "Offboard"
};

static int control_out_echo(void* param)
{
    Control_Out_Bus control_out;
    if (mcn_copy_from_hub((McnHub*)param, &control_out) == FMT_EOK) {
        console_printf("timestamp:%d actuator: %d %d %d %d\n", control_out.timestamp, control_out.actuator_cmd[0], control_out.actuator_cmd[1], control_out.actuator_cmd[2], control_out.actuator_cmd[3]);
    }
    return 0;
}

static int fms_output_echo(void* param)
{
    FMS_Out_Bus fms_out;

    if (mcn_copy_from_hub((McnHub*)param, &fms_out) == FMT_EOK) {
        printf("timestamp:%u\n", fms_out.timestamp);
        printf("rate cmd: %.2f %.2f %.2f\n", fms_out.p_cmd, fms_out.q_cmd, fms_out.r_cmd);
        printf("att cmd: %.2f %.2f %.2f\n", fms_out.phi_cmd, fms_out.theta_cmd, fms_out.psi_rate_cmd);
        printf("vel cmd: %.2f %.2f %.2f\n", fms_out.u_cmd, fms_out.v_cmd, fms_out.w_cmd);
        printf("throttle cmd: %.2f\n", fms_out.throttle_cmd);
        printf("act cmd: %u %u %u %u\n", fms_out.actuator_cmd[0], fms_out.actuator_cmd[1], fms_out.actuator_cmd[2], fms_out.actuator_cmd[3]);
        printf("status:%s state:%s ctrl_mode:%s\n", fms_status[fms_out.status], fms_state[fms_out.state], fms_ctrl_mode[fms_out.ctrl_mode]);
        printf("reset:%d mode:%s\n", fms_out.reset, fms_mode[fms_out.mode]);
        printf("wp_current:%d wp_consume:%d\n", fms_out.wp_current, fms_out.wp_consume);
        printf("------------------------------------------\n");
    }

    return 0;
}


// static void mlog_start_cb(void)
// {
//     pilot_cmd_updated = 1;
//     gcs_cmd_updated = 1;
//     mission_data_updated = 1;
// }

// static void init_parameter(void)
// {
//     FMT_CHECK(param_link_variable(PARAM_GET(APM, USER_TEST_P1), &apm_params.user_test_p1));
// }

// static int echo_my_radio_in(void* parameter)
// {
//     fmt_err_t err;
//     my_radio_in rc_chan_val;

//     err = mcn_copy_from_hub((McnHub*)parameter, &rc_chan_val);

//     if (err != FMT_EOK)
//         return -1;

//     uint8_t rc_chan_num = 16;

//     console_printf("rc channel: [");
//     for (int i = 0; i < rc_chan_num; i++) {
//         if (i == rc_chan_num - 1) {
//             console_printf("%d]\n", rc_chan_val.radio_in[i]);
//         } else {
//             console_printf("%d,", rc_chan_val.radio_in[i]);
//         }
//     }

//     return 0;
// }

void apm_interface_step(uint32_t timestamp)
{
    // if (mcn_poll(pilot_cmd_nod)) {
    //     mcn_copy(MCN_HUB(pilot_cmd), pilot_cmd_nod, &FMS_U.Pilot_Cmd);

    //     FMS_U.Pilot_Cmd.timestamp = timestamp;
    //     pilot_cmd_updated = 1;
    // }

    if (mcn_poll(apm_handler.gcs_cmd_nod)) {
        mcn_copy(MCN_HUB(gcs_cmd), apm_handler.gcs_cmd_nod, &apm_handler.gcs_cmd_msg);

        apm_handler.gcs_cmd_msg.timestamp = timestamp;
        apm_handler.gcs_cmd_updated = 1;
        apm_handler.gcs_cmd_log = 1;
    }

    if (mcn_poll(apm_handler.mission_data_nod)) {
        mcn_copy(MCN_HUB(mission_data), apm_handler.mission_data_nod, &apm_handler.mission_data_msg);

        apm_handler.mission_data_msg.timestamp = timestamp;
        apm_handler.mission_data_updated = 1;
        apm_handler.mission_data_log = 1;
    }

    if (mcn_poll(apm_handler.rc_channels_nod)) {
        mcn_copy(MCN_HUB(rc_channels), apm_handler.rc_channels_nod, &apm_handler.rcChannel_msg);
        apm_handler.pilot_cmd_updated = 1;
        apm_handler.pilot_cmd_log = 1;
    }

    if (mcn_poll(apm_handler.ins_out_nod)) {
        mcn_copy(MCN_HUB(ins_output), apm_handler.ins_out_nod, &apm_handler.ins_out_msg);
    }

    if (mcn_poll(apm_handler.rangefinder_data_nod)) {
        mcn_copy(MCN_HUB(sensor_rangefinder), apm_handler.rangefinder_data_nod, &apm_handler.rangefinder_data_msg);
        apm_handler.rangefinder_data_updated = 1;
        apm_handler.rangefinder_data_log = 1;
    }

    if (mcn_poll(apm_handler.optflow_data_nod)) {
        mcn_copy(MCN_HUB(sensor_optflow), apm_handler.optflow_data_nod, &apm_handler.optflow_data_msg);
        apm_handler.optflow_data_updated = 1;
        apm_handler.optflow_data_log = 1;
    }

    // FMS_step();
    APM_loop();

    apm_handler.control_out_msg.timestamp = timestamp;
    mcn_publish(MCN_HUB(control_output), &apm_handler.control_out_msg);
    apm_handler.fms_out_msg.timestamp = timestamp;
    mcn_publish(MCN_HUB(fms_output), &apm_handler.fms_out_msg);

    if (apm_handler.pilot_cmd_log) {
        apm_handler.pilot_cmd_log = 0;
    //     /* Log pilot command */
    //     mlog_push_msg((uint8_t*)&FMS_U.Pilot_Cmd, Pilot_Cmd_ID, sizeof(Pilot_Cmd_Bus));
    }

    if (apm_handler.gcs_cmd_log) {
        apm_handler.gcs_cmd_log = 0;
    //     /* Log gcs command */
    //     mlog_push_msg((uint8_t*)&FMS_U.GCS_Cmd, GCS_Cmd_ID, sizeof(GCS_Cmd_Bus));
    }

    if (apm_handler.mission_data_log) {
        apm_handler.mission_data_log = 0;
    //     /* Log mission data */
    //     mlog_push_msg((uint8_t*)&FMS_U.Mission_Data, Mission_Data_ID, sizeof(Mission_Data_Bus));
    }

    if (apm_handler.rangefinder_data_log) {
        apm_handler.rangefinder_data_log = 0;
    //     /* Log mission data */
    //     mlog_push_msg((uint8_t*)&FMS_U.Mission_Data, Mission_Data_ID, sizeof(Mission_Data_Bus));
    }

    if (apm_handler.optflow_data_log) {
        apm_handler.optflow_data_log = 0;
    //     /* Log mission data */
    //     mlog_push_msg((uint8_t*)&FMS_U.Mission_Data, Mission_Data_ID, sizeof(Mission_Data_Bus));
    }

    // /* Log FMS output bus data */
    // DEFINE_TIMETAG(fms_output, 100);
    // if (check_timetag(TIMETAG(fms_output))) {
    //     /* Log FMS out data */
    //     mlog_push_msg((uint8_t*)&FMS_Y.FMS_Out, FMS_Out_ID, sizeof(FMS_Out_Bus));
    // }



    // for test purpose, will delete later
    // static int16_t count = 0;
    // for (uint8_t i = 0; i < sizeof(rcChannel_tmp)/2; i++){
    //     rcChannel_tmp[i] = count;
    //     MY_RADIO_BUS.radio_in[i] = count;
    //     // printf("  rcChannel_tmp[%d] = %d", i, rcChannel_tmp[15]);
    // }
    // mcn_publish(MCN_HUB(rc_channels), &rcChannel_tmp);
    // mcn_publish(MCN_HUB(my_radio_in_topic), &MY_RADIO_BUS);
    // count++;
    // my_radio_in rc_chan_val;
    // if (mcn_poll(my_radio_in_nod)) {
    //     mcn_copy(MCN_HUB(my_radio_in_topic), my_radio_in_nod, &rc_chan_val);
    //     // APM_update_rc();
    //     uint8_t i = 15;
    //     // printf("  rc_chan_val.radio_in[%d] = %d\n", i, rc_chan_val.radio_in[i]);
    // }
}


void apm_interface_init(void)
{
    // fms_model_info.period = FMS_EXPORT.period;
    // fms_model_info.info = (char*)FMS_EXPORT.model_info;

    // mcn_advertise(MCN_HUB(fms_output), fms_output_echo);

    // pilot_cmd_nod = mcn_subscribe(MCN_HUB(pilot_cmd), NULL, NULL);
    apm_handler.gcs_cmd_nod          = mcn_subscribe(MCN_HUB(gcs_cmd), NULL, NULL);
    apm_handler.mission_data_nod     = mcn_subscribe(MCN_HUB(mission_data), NULL, NULL);
    apm_handler.ins_out_nod          = mcn_subscribe(MCN_HUB(ins_output), NULL, NULL);
    apm_handler.rc_channels_nod      = mcn_subscribe(MCN_HUB(rc_channels), NULL, NULL);
    apm_handler.rangefinder_data_nod = mcn_subscribe(MCN_HUB(sensor_rangefinder), NULL, NULL);
    apm_handler.optflow_data_nod     = mcn_subscribe(MCN_HUB(sensor_optflow), NULL, NULL);

    // Pilot_Cmd_ID = mlog_get_bus_id("Pilot_Cmd");
    // GCS_Cmd_ID = mlog_get_bus_id("GCS_Cmd");
    // Mission_Data_ID = mlog_get_bus_id("Mission_Data");
    // FMS_Out_ID = mlog_get_bus_id("FMS_Out");
    // FMT_ASSERT(Pilot_Cmd_ID >= 0);
    // FMT_ASSERT(GCS_Cmd_ID >= 0);
    // FMT_ASSERT(Mission_Data_ID >= 0);
    // FMT_ASSERT(FMS_Out_ID >= 0);

    // mlog_register_callback(MLOG_CB_START, mlog_start_cb);

    // init_parameter();
    // FMT_CHECK(mcn_advertise(MCN_HUB(rc_channels), echo_my_radio_in)); // should be advertised in pilot_cmd, put here for SIL

    mcn_advertise(MCN_HUB(control_output), control_out_echo);
    mcn_advertise(MCN_HUB(fms_output),     fms_output_echo);


    APM_init();

    // for test purpose, will delete later
    // FMT_CHECK(mcn_advertise(MCN_HUB(my_radio_in_topic), echo_my_radio_in));
    // my_radio_in_nod = mcn_subscribe(MCN_HUB(my_radio_in_topic), NULL, NULL);
}