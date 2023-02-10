#ifndef AP_LOG_H
#define AP_LOG_H

#pragma once

class AP_Log
{
public:
    AP_Log() {};
    float climb_rate_cms_thr = 0.0f;
    float climb_rate_cms_after_surface = 0.0f;
};

extern AP_Log apm_log;
#endif
