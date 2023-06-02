/******************************************************************************
 * Copyright 2020-2021 The Firmament Authors. All Rights Reserved.
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
#include "module/sensor/sensor_bms.h"
#include "hal/bms/bms.h"


/**
 * @brief Measure scaled bms data (gauss)
 * 
 * @param bms_dev bms sensor device
 * @param buffer Data buffer
 * @return fmt_err_t FMT_EOK for success
 */

fmt_err_t sensor_bms_measure(sensor_bms_t bms_dev, bms_data_t* bms_report)
{
    rt_size_t r_byte;
    rt_uint16_t report[4];
    r_byte = rt_device_read(bms_dev->dev, 0, (void*)report, 8);
    bms_report->Valtage_Value=report[0];
    bms_report->Current_Value=report[1];
    bms_report->Relative_State_OfCharge_Value=(uint8_t)report[2];
    bms_report->Max_Error_Value=(uint8_t)report[3];
    return r_byte == 8 ? FMT_EOK : FMT_ERROR;
}

/**
 * @brief Initialize bms sensor device
 * 
 * @param bms_dev_name bms device name
 * @return sensor_bms_t bms sensor device
 */
sensor_bms_t sensor_bms_init(const char* bms_dev_name)
{
    sensor_bms_t bms_dev = (sensor_bms_t)rt_malloc(sizeof(struct sensor_bms));
    RT_ASSERT(bms_dev != NULL);

    bms_dev->dev = rt_device_find(bms_dev_name);
    RT_ASSERT(bms_dev->dev);

    RT_CHECK(rt_device_open(bms_dev->dev, RT_DEVICE_OFLAG_RDWR));

    return bms_dev;
}
