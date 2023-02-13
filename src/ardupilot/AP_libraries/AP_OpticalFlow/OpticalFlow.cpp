#include "OpticalFlow.h"
#include "AP_OpticalFlow_FMT.h"

// default constructor
OpticalFlow::OpticalFlow(AP_AHRS_NavEKF &ahrs)
    : _ahrs(ahrs),
      _last_update_ms(0)
{
    memset(&_state, 0, sizeof(_state));

    // healthy flag will be overwritten on update
    _flags.healthy = false;
}

void OpticalFlow::init(void)
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    if (!backend) {
        backend = new AP_OpticalFlow_FMT(*this);
    }

    if (backend != nullptr) {
        backend->init();
    }
}

void OpticalFlow::update(void)
{
    if (backend != nullptr) {
        backend->update();
    }
    // only healthy if the data is less than 0.5s old
    _flags.healthy = (millis() - _last_update_ms < 500);
}

