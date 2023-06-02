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

#ifndef MAG_H__
#define MAG_H__

#include <firmament.h>

#ifdef __cplusplus
extern "C" {
#endif


/* mag device bus type */
#define BMS_SPI_BUS_TYPE 1
#define BMS_I2C_BUS_TYPE 2



 struct bms_configure {
    //  rt_uint32_t sample_rate_hz;
    //  rt_uint16_t dlpf_freq_hz;
    //  rt_uint32_t bms_range_ga;
    rt_uint32_t test;
 };

struct bms_device {
    struct rt_device parent;
    const struct bms_ops* ops;
    struct bms_configure config;
    rt_uint8_t bus_type;
};
typedef struct bms_device* bms_dev_t;

/* bms driver opeations */
struct bms_ops {
    rt_err_t (*bms_config)(bms_dev_t bms, const struct bms_configure* cfg);
    rt_err_t (*bms_control)(bms_dev_t bms, int cmd, void* arg);
    rt_size_t (*bms_read)(bms_dev_t bms, rt_off_t pos, void* data, rt_size_t size);
};

rt_err_t hal_bms_register(bms_dev_t bms, const char* name, rt_uint32_t flag, void* data);

#ifdef __cplusplus
}
#endif

#endif
