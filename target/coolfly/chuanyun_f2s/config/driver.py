# Modify this file to decide which drivers are compiled

DRIVERS = [
    # 'imu/icm20600.c',
    # 'imu/icm20689.c',
    'imu/bmi055.c',
    # 'imu/bmi088.c',
    'mag/ist8310.c',
    # 'mag/mmc5983ma.c',
    # 'barometer/ms5611.c',
    'barometer/spl06.c',
    'gps/gps_m8n.c',
    # 'rgb_led/ncp5623c.c',
    'mtd/ramtron.c',
    'mtd/spi_tfcard.c',
    # 'vision_flow/pmw3901_l0x.c',
    # 'vision_flow/pmw3901_xx.c',
    # 'range_finder/tf_luna.c',
    # 'airspeed/ms4525.c',
    'vision_flow/mtf_01.c',
]

DRIVERS_CPPPATH = []
#include "driver/vision_flow/pmw3901_l0x.h"