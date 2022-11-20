#include <APM.h>
#include <firmament.h>

#include "apm_copter_wrapper.h"

APM_Params_t apm_params = { 0 };

void APM_init(void)                //上电初始化
{
    APM_init_para();
    APM_Copter_Init();
}

void APM_init_para(void)           //参数初始化
{
    APM_Copter_init_para();
    // copter.p1 = apm_params.p1;
}

void APM_400Hz(void)               //400hz飞控循环
{
    APM_update_rc();
    APM_update_para();
    APM_update_inertial();
    APM_Copter_400Hz();
}

void APM_10Hz(void)                //10hz飞控循环
{
    ;
}

void APM_5Hz(void)                 //5hz飞控循环
{
    ;
}

void APM_3Hz(void)                 //3hz飞控循环
{
    ;
}

void APM_1Hz(void)                 //1hz飞控循环
{
    ;
}

void APM_update_para(void){
    APM_init_para();
}

void APM_update_rc(void)
{
    // trans rc value to copter instance
    // for (uint8_t i = 0; i < 18; i++){
    //     hal.rcin._rc_in_data[i] = RC_in_data[i];
    // }
    // hal.rcin._new_input = true;
}

void APM_update_inertial(void)
{
    ;
}

// uint32_t APM_millis(void)
// {
//     return systime_now_ms();
// }

// uint64_t APM_micros(void)
// {
//     return systime_now_us();
// }


