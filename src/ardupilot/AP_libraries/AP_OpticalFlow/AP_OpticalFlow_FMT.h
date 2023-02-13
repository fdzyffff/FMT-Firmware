#pragma once

#include "OpticalFlow.h"

class AP_OpticalFlow_FMT : public OpticalFlow_backend
{
public:
    /// constructor
    AP_OpticalFlow_FMT(OpticalFlow &_frontend);

    // init - initialise the sensor
    void init();

    // update - read latest values from sensor and fill in x,y and totals.
    void update(void);

private:
    uint32_t last_flow_ms;

    uint8_t next_optflow_index;
    uint8_t optflow_delay;
};
