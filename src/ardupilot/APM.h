#ifndef APM_H
#define APM_H

#include <firmament.h>

// modify for apm interface
#ifdef __cplusplus
extern "C" {
#endif



// #ifndef DEFINED_TYPEDEF_FOR_IMU_Bus_
//     #define DEFINED_TYPEDEF_FOR_IMU_Bus_

// typedef struct {
//     uint32_t timestamp;
//     float gyr_x;
//     float gyr_y;
//     float gyr_z;
//     float acc_x;
//     float acc_y;
//     float acc_z;
// } IMU_Bus;

// #endif

// #ifndef DEFINED_TYPEDEF_FOR_MAG_Bus_
//     #define DEFINED_TYPEDEF_FOR_MAG_Bus_

// typedef struct {
//     uint32_t timestamp;
//     float mag_x;
//     float mag_y;
//     float mag_z;
// } MAG_Bus;

// #endif

// #ifndef DEFINED_TYPEDEF_FOR_Barometer_Bus_
//     #define DEFINED_TYPEDEF_FOR_Barometer_Bus_

// typedef struct {
//     uint32_t timestamp;
//     float pressure;
//     float temperature;
// } Barometer_Bus;

// #endif

// #ifndef DEFINED_TYPEDEF_FOR_GPS_uBlox_Bus_
//     #define DEFINED_TYPEDEF_FOR_GPS_uBlox_Bus_

// typedef struct {
//     uint32_t timestamp;
//     uint32_t iTOW;
//     uint16_t year;
//     uint8_t month;
//     uint8_t day;
//     uint8_t hour;
//     uint8_t min;
//     uint8_t sec;
//     uint8_t valid;
//     uint32_t tAcc;
//     int32_t nano;
//     uint8_t fixType;
//     uint8_t flags;
//     uint8_t reserved1;
//     uint8_t numSV;
//     int32_t lon;
//     int32_t lat;
//     int32_t height;
//     int32_t hMSL;
//     uint32_t hAcc;
//     uint32_t vAcc;
//     int32_t velN;
//     int32_t velE;
//     int32_t velD;
//     int32_t gSpeed;
//     int32_t heading;
//     uint32_t sAcc;
//     uint32_t headingAcc;
//     uint16_t pDOP;
//     uint16_t reserved2;
// } GPS_uBlox_Bus;

// #endif

// #ifndef DEFINED_TYPEDEF_FOR_Rangefinder_Bus_
//     #define DEFINED_TYPEDEF_FOR_Rangefinder_Bus_

// typedef struct {
//     uint32_t timestamp;
//     float distance_m;
// } Rangefinder_Bus;

// #endif

// #ifndef DEFINED_TYPEDEF_FOR_Optical_Flow_Bus_
//     #define DEFINED_TYPEDEF_FOR_Optical_Flow_Bus_

// typedef struct {
//     uint32_t timestamp;
//     float vx;
//     float vy;
//     uint32_t valid;
// } Optical_Flow_Bus;

// #endif

#ifndef DEFINED_TYPEDEF_FOR_INS_Out_Bus_
    #define DEFINED_TYPEDEF_FOR_INS_Out_Bus_

typedef struct {
    uint32_t timestamp;
    float phi;
    float theta;
    float psi;
    float quat[4];
    float p;
    float q;
    float r;
    float ax;
    float ay;
    float az;
    float vn;
    float ve;
    float vd;
    float reserved;
    double lat;
    double lon;
    double alt;
    double lat_0;
    double lon_0;
    double alt_0;
    float x_R;
    float y_R;
    float h_R;
    float h_AGL;
    uint32_t flag;
    uint32_t status;
} INS_Out_Bus;

#endif

// #ifndef DEFINED_TYPEDEF_FOR_Pilot_cmd_Bus_
//     #define DEFINED_TYPEDEF_FOR_Pilot_cmd_Bus_

// typedef struct {
//     uint32_t timestamp;
//     float phi;
//     float theta;
//     float psi;
//     float quat[4];
//     float p;
//     float q;
//     float r;
//     float ax;
//     float ay;
//     float az;
//     float vn;
//     float ve;
//     float vd;
//     float reserved;
//     double lat;
//     double lon;
//     double alt;
//     double lat_0;
//     double lon_0;
//     double alt_0;
//     float x_R;
//     float y_R;
//     float h_R;
//     float h_AGL;
//     uint32_t flag;
//     uint32_t status;
// } Pilot_Cmd_Bus;

// #endif

// parameters
// typedef struct {
//     float user_test_p1;             //test parameter p1
// } APM_Params_t;

// typedef struct {
//     int16_t radio_in[16];
// } my_radio_in;

extern int16_t rcChannel[16];
extern INS_Out_Bus ins_out_msg;

void APM_init(void);                //上电初始化
void APM_init_para(void);           //参数初始化
void APM_loop(void);                //ardupilot主循环
// void APM_update_para(void);      //参数更新

void APM_update_para(void);
void APM_update_rc(void);
void APM_update_inertial(void);

// extern APM_Params_t apm_params;

#ifdef __cplusplus
}
#endif

#endif