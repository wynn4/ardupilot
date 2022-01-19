#pragma once

#include <AP_Param/AP_Param.h>

# define AP_MOT_TEMPDIVIDER_DEFAULT       1.8f

class AP_MotorTempMonitor_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_MotorTempMonitor_Params(void);

    /* Do not allow copies */
    AP_MotorTempMonitor_Params(const AP_MotorTempMonitor_Params &other) = delete;
    AP_MotorTempMonitor_Params &operator=(const AP_MotorTempMonitor_Params&) = delete;

    // Battery monitor driver types
    enum MotorTempMonitor_Type {
        MotorTempMonitor_TYPE_NONE          = 0,
        MotorTempMonitor_TYPE_INDAGO        = 1,
    };

    // low voltage sources (used for BATT_LOW_TYPE parameter)
    enum MotorTempMonitor_HighTemp_Source {
        MotorTempMonitor_HighTempSource_None          = 0,
        MotorTempMonitor_HighTempSource_c = 1
    };

    MotorTempMonitor_Type type(void) const { return (enum MotorTempMonitor_Type)_type.get(); }
    MotorTempMonitor_HighTemp_Source failsafe_temp_source(void) { return (enum MotorTempMonitor_HighTemp_Source)_failsafe_temp_source.get(); }

    AP_Int8  _type;                     /// 0=disabled, 3=voltage only, 4=voltage and current
    AP_Float _deg_c_divisor;           /// temp in deg C calculated as DEG_C = ((value received) / _deg_c_divisor) - _deg_c_offset
    AP_Float _deg_c_offset;            /// temp in deg C calculated as DEG_C = ((value received) / _deg_c_divisor) - _deg_c_offset
    AP_Int8  _high_temp_timeout;      /// timeout in seconds before a high temperature event will be triggered
    AP_Int8  _failsafe_temp_source;  /// voltage type used for detection of low voltage event
    AP_Float _min_temp_c;              /// voltage level used to trigger a low battery failsafe
    AP_Int8  _failsafe_high_action;      /// action to preform on a low battery failsafe
    AP_Int8  _failsafe_critical_action; /// action to preform on a critical battery failsafe
    AP_Float _high_temp_c;  /// tempeterature (deg C) used to trigger a high temp warning
    AP_Float _critical_temp_c;   /// tempeterature (deg C) used to trigger a high temp failsafe
};
