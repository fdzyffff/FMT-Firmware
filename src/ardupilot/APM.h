#ifndef APM_H
#define APM_H

#include <firmament.h>

// modify for apm interface
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float p1;         ///test parameter p1
} APM_Params_t;

extern APM_Params_t apm_params;

void APM_init(void);                //上电初始化
void APM_init_para(void);           //参数初始化
void APM_400Hz(void);               //400hz飞控循环
void APM_10Hz(void);                //10hz飞控循环
void APM_5Hz(void);                 //5hz飞控循环
void APM_3Hz(void);                 //3hz飞控循环
void APM_1Hz(void);                 //1hz飞控循环
// void APM_update_para(void);         //参数更新

void APM_update_para(void);
void APM_update_rc(void);
void APM_update_inertial(void);

// extern uint32_t APM_millis(void);
// extern uint64_t APM_micros(void);

#ifdef __cplusplus
}
#endif

#endif