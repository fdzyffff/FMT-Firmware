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

#define FMT_READ_RADIO 0
// FMS input topic
// MCN_DECLARE(pilot_cmd);
// MCN_DECLARE(gcs_cmd);
// MCN_DECLARE(mission_data);
MCN_DECLARE(ins_output);
// MCN_DECLARE(control_output);
MCN_DECLARE(rc_channels);

// MCN_DEFINE(auto_cmd, sizeof(Auto_Cmd_Bus));
// MCN_DEFINE(fms_output, sizeof(FMS_Out_Bus));
// MCN_DEFINE(control_output, sizeof(Control_Out_Bus));

int16_t rcChannel[16];
INS_Out_Bus ins_out_msg;

// static int16_t rcChannel_tmp[16];
// static my_radio_in MY_RADIO_BUS;
// MCN_DEFINE(my_radio_in_topic, sizeof(MY_RADIO_BUS));

/* define parameters */
static param_t __param_list_apm[] = {
    /* Param here*/
    PARAM_FLOAT(USER_TEST_P1, 0.15),
};
PARAM_GROUP_DEFINE(APM, __param_list_apm);

#if FMT_READ_RADIO == 1
static McnNode_t rc_channels_nod;
#endif

static McnNode_t ins_out_nod;
// static McnNode_t pilot_cmd_nod;
// static McnNode_t gcs_cmd_nod;
// static McnNode_t mission_data_nod;
// static McnNode_t control_out_nod;
// static uint8_t pilot_cmd_updated = 1;
// static uint8_t gcs_cmd_updated = 1;
// static uint8_t mission_data_updated = 1;
// static McnNode_t my_radio_in_nod;

// static int Pilot_Cmd_ID;
// static int GCS_Cmd_ID;
// static int Mission_Data_ID;
// static int FMS_Out_ID;
// static char* fms_status[] = {
//     "None",
//     "Disarm",
//     "Standby",
//     "Arm"
// };

// static char* fms_state[] = {
//     "None",
//     "Disarm",
//     "Standby",
//     "Offboard",
//     "Mission",
//     "InvalidAutoMode",
//     "Hold",
//     "Acro",
//     "Stabilize",
//     "Altitude",
//     "Position",
//     "InvalidAssistMode",
//     "Manual",
//     "InValidManualMode",
//     "InvalidArmMode",
//     "Land",
//     "Return",
//     "Takeoff"
// };

// static char* fms_ctrl_mode[] = {
//     "None",
//     "Manual",
//     "Acro",
//     "Stabilize",
//     "ALTCTL",
//     "POSCTL"
// };

// static char* fms_mode[] = {
//     "None",
//     "Manual",
//     "Acro",
//     "Stabilize",
//     "Altitude",
//     "Position",
//     "Mission",
//     "Offboard"
// };

// fmt_model_info_t fms_model_info;

// static int apm_output_echo(void* param)
// {
//     FMS_Out_Bus fms_out;

//     mcn_copy_from_hub((McnHub*)param, &fms_out);

//     printf("timestamp:%u\n", fms_out.timestamp);
//     printf("rate cmd: %.2f %.2f %.2f\n", fms_out.p_cmd, fms_out.q_cmd, fms_out.r_cmd);
//     printf("att cmd: %.2f %.2f %.2f\n", fms_out.phi_cmd, fms_out.theta_cmd, fms_out.psi_rate_cmd);
//     printf("vel cmd: %.2f %.2f %.2f\n", fms_out.u_cmd, fms_out.v_cmd, fms_out.w_cmd);
//     printf("throttle cmd: %.2f\n", fms_out.throttle_cmd);
//     printf("act cmd: %u %u %u %u\n", fms_out.actuator_cmd[0], fms_out.actuator_cmd[1], fms_out.actuator_cmd[2], fms_out.actuator_cmd[3]);
//     printf("status:%s state:%s ctrl_mode:%s\n", fms_status[fms_out.status], fms_state[fms_out.state], fms_ctrl_mode[fms_out.ctrl_mode]);
//     printf("reset:%d mode:%s\n", fms_out.reset, fms_mode[fms_out.mode]);
//     printf("wp_current:%d wp_consume:%d\n", fms_out.wp_current, fms_out.wp_consume);
//     printf("------------------------------------------\n");

//     return 0;
// }

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

    // if (mcn_poll(gcs_cmd_nod)) {
    //     mcn_copy(MCN_HUB(gcs_cmd), gcs_cmd_nod, &FMS_U.GCS_Cmd);

    //     FMS_U.GCS_Cmd.timestamp = timestamp;
    //     gcs_cmd_updated = 1;
    // }

    // if (mcn_poll(mission_data_nod)) {
    //     mcn_copy(MCN_HUB(mission_data), mission_data_nod, &FMS_U.Mission_Data);

    //     FMS_U.Mission_Data.timestamp = timestamp;
    //     mission_data_updated = 1;
    // }
    // if (mcn_poll(control_out_nod)) {
    //     mcn_copy(MCN_HUB(control_output), control_out_nod, &FMS_U.Control_Out);
    // }

    // FMS_step();

    // mcn_publish(MCN_HUB(fms_output), &FMS_Y.FMS_Out);

    // if (pilot_cmd_updated) {
    //     pilot_cmd_updated = 0;
    //     /* Log pilot command */
    //     mlog_push_msg((uint8_t*)&FMS_U.Pilot_Cmd, Pilot_Cmd_ID, sizeof(Pilot_Cmd_Bus));
    // }

    // if (gcs_cmd_updated) {
    //     gcs_cmd_updated = 0;
    //     /* Log gcs command */
    //     mlog_push_msg((uint8_t*)&FMS_U.GCS_Cmd, GCS_Cmd_ID, sizeof(GCS_Cmd_Bus));
    // }

    // if (mission_data_updated) {
    //     mission_data_updated = 0;
    //     /* Log mission data */
    //     mlog_push_msg((uint8_t*)&FMS_U.Mission_Data, Mission_Data_ID, sizeof(Mission_Data_Bus));
    // }

    // /* Log FMS output bus data */
    // DEFINE_TIMETAG(fms_output, 100);
    // if (check_timetag(TIMETAG(fms_output))) {
    //     /* Log FMS out data */
    //     mlog_push_msg((uint8_t*)&FMS_Y.FMS_Out, FMS_Out_ID, sizeof(FMS_Out_Bus));
    // }

#if FMT_READ_RADIO == 1
    if (mcn_poll(rc_channels_nod)) {
        mcn_copy(MCN_HUB(rc_channels), rc_channels_nod, &rcChannel);
        APM_update_rc();
    }
#endif

    if (mcn_poll(ins_out_nod)) {
        mcn_copy(MCN_HUB(ins_output), ins_out_nod, &ins_out_msg);
        APM_update_inertial();
        
        // static uint32_t tnow;
        // if (systime_now_ms() - tnow > 1000) {
        //     printf("--------------ins_out_msg.timestamp = %ld--------------\n", ins_out_msg.timestamp);
        //     printf("  ins_out_msg.phi = %f\n", ins_out_msg.phi);
        //     printf("  ins_out_msg.theta = %f\n", ins_out_msg.theta);
        //     printf("  ins_out_msg.psi = %f\n", ins_out_msg.psi);
        //     printf("  ins_out_msg.quat = [%f,%f,%f,%f]\n", ins_out_msg.quat[0], ins_out_msg.quat[1], ins_out_msg.quat[2], ins_out_msg.quat[3]);
        //     printf("  ins_out_msg.p = %f\n", ins_out_msg.p);
        //     printf("  ins_out_msg.q = %f\n", ins_out_msg.q);
        //     printf("  ins_out_msg.r = %f\n", ins_out_msg.r);
        //     printf("  ins_out_msg.ax = %f\n", ins_out_msg.ax);
        //     printf("  ins_out_msg.ay = %f\n", ins_out_msg.ay);
        //     printf("  ins_out_msg.az = %f\n", ins_out_msg.az);
        //     printf("  ins_out_msg.vn = %f\n", ins_out_msg.vn);
        //     printf("  ins_out_msg.ve = %f\n", ins_out_msg.ve);
        //     printf("  ins_out_msg.vd = %f\n", ins_out_msg.vd);
        //     printf("  ins_out_msg.reserved = %f\n", ins_out_msg.reserved);
        //     printf("  ins_out_msg.lat = %f\n", ins_out_msg.lat);
        //     printf("  ins_out_msg.lon = %f\n", ins_out_msg.lon);
        //     printf("  ins_out_msg.alt = %f\n", ins_out_msg.alt);
        //     printf("  ins_out_msg.lat_0 = %f\n", ins_out_msg.lat_0);
        //     printf("  ins_out_msg.lon_0 = %f\n", ins_out_msg.lon_0);
        //     printf("  ins_out_msg.alt_0 = %f\n", ins_out_msg.alt_0);
        //     printf("  ins_out_msg.x_R = %f\n", ins_out_msg.x_R);
        //     printf("  ins_out_msg.y_R = %f\n", ins_out_msg.y_R);
        //     printf("  ins_out_msg.h_R = %f\n", ins_out_msg.h_R);
        //     printf("  ins_out_msg.h_AGL = %f\n", ins_out_msg.h_AGL);
        //     printf("  ins_out_msg.flag = %ld\n", ins_out_msg.flag);
        //     printf("  ins_out_msg.status = %ld\n", ins_out_msg.status);
        //     tnow = systime_now_ms();
        // }
    }

    APM_loop();

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
    // gcs_cmd_nod = mcn_subscribe(MCN_HUB(gcs_cmd), NULL, NULL);
    // mission_data_nod = mcn_subscribe(MCN_HUB(mission_data), NULL, NULL);
    // ins_out_nod = mcn_subscribe(MCN_HUB(ins_output), NULL, NULL);
    // control_out_nod = mcn_subscribe(MCN_HUB(control_output), NULL, NULL);

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

#if FMT_READ_RADIO == 1
    rc_channels_nod = mcn_subscribe(MCN_HUB(rc_channels), NULL, NULL);
#endif
    ins_out_nod = mcn_subscribe(MCN_HUB(ins_output), NULL, NULL);

    APM_init();

    // for test purpose, will delete later
    // FMT_CHECK(mcn_advertise(MCN_HUB(my_radio_in_topic), echo_my_radio_in));
    // my_radio_in_nod = mcn_subscribe(MCN_HUB(my_radio_in_topic), NULL, NULL);
}