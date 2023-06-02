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

#include "hal/bms/bms.h"
#include <firmament.h>
#include "board.h"

_EXT_DTCM1
static rt_err_t hal_bms_init(struct rt_device* dev)
{
    rt_err_t ret = RT_EOK;
    bms_dev_t bms;

    RT_ASSERT(dev != RT_NULL);
    bms = (bms_dev_t)dev;

    /* apply configuration */
    if (bms->ops->bms_config) {
        ret = bms->ops->bms_config(bms, &bms->config);
    }

    return ret;
}

_EXT_DTCM1
static rt_size_t hal_bms_read(struct rt_device* dev,
                              rt_off_t pos,
                              void* buffer,
                              rt_size_t size)
{
    rt_size_t rb = 0;
    bms_dev_t bms;

    RT_ASSERT(dev != RT_NULL);

    bms = (bms_dev_t)dev;

    if (bms->ops->bms_read && size) {
        rb = bms->ops->bms_read(bms, pos, buffer, size);
    }

    return rb;
}


_EXT_DTCM1
rt_err_t hal_bms_register(bms_dev_t bms, const char* name, rt_uint32_t flag, void* data)
{
    rt_err_t ret;
    struct rt_device* device;

    RT_ASSERT(bms != RT_NULL);

    device = &(bms->parent);

    device->type = (bms->bus_type == BMS_SPI_BUS_TYPE) ? RT_Device_Class_SPIDevice : RT_Device_Class_I2CBUS;
    device->ref_count = 0;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

    device->init = hal_bms_init;
    device->open = RT_NULL;
    device->close = RT_NULL;
    device->read = hal_bms_read;
    device->write = RT_NULL;
    device->control = RT_NULL;
    device->user_data = data;

    /* register a character device */
    ret = rt_device_register(device, name, flag);

    return ret;
}
