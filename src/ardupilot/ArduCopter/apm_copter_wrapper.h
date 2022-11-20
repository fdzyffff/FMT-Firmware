#ifndef APM_COPTER_WRAPPER_H
#define APM_COPTER_WRAPPER_H

void APM_Copter_Init(void);    //上电初始化
void APM_Copter_400Hz(void);   //400hz飞控循环
void APM_Copter_10Hz(void);    //10hz飞控循环
void APM_Copter_init_para(void);
void APM_Copter_update_para(void);
void APM_Copter_update_rc(void);
void APM_Copter_update_inertial(void);

// uint32_t millis();
// uint64_t micro64();

// extern APM_Params_t apm_params;
// extern uint32_t APM_millis(void);
// extern Copter copter;
// extern AP_HAL hal;
// extern my_temp_log_t my_temp_log;

// extern int16_t RC_in_data[20];
// extern int16_t RC_out_data[20];

#endif