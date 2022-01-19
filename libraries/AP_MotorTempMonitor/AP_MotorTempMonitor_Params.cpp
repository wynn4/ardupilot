#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "AP_MotorTempMonitor_Params.h"

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
  #define DEFAULT_LOW_BATTERY_VOLTAGE 10.5f
#else
  #define DEFAULT_LOW_BATTERY_VOLTAGE 0.0f
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)

const AP_Param::GroupInfo AP_MotorTempMonitor_Params::var_info[] = {
    // @Param: MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the motor temperature
    // @Values: 0:Disabled,1:Indago deg C via I2C
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("MONITOR", 1, AP_MotorTempMonitor_Params, _type, MotorTempMonitor_TYPE_NONE, AP_PARAM_FLAG_ENABLE),

    // @Param: TEMP_DIV
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the raw temperature register reading of the motor to deg C. For Indago, this is (raw_value/ 1.8 - TEMP_DIV).
    // @User: Advanced
    AP_GROUPINFO("TEMP_DIV", 2, AP_MotorTempMonitor_Params, _deg_c_divisor, AP_MOT_TEMPDIVIDER_DEFAULT),

    // @Param: TEMP_OFFSET
    // @DisplayName: Temperature offset
    // @Description: Temperature offset at zero value raw input from sensor
    // @Units: C
    // @User: Standard
    AP_GROUPINFO("TEMP_OFFSET", 3, AP_MotorTempMonitor_Params, _deg_c_offset, 0),

    // @Param: HIGH_TIMER
    // @DisplayName: High temperature timeout
    // @Description: This is the timeout in seconds before a high temperature event will be triggered. A value of zero disables high temperature errors.
    // @Units: C
    // @Increment: 1
    // @Range: 0 120
    // @User: Advanced
    AP_GROUPINFO("HIGH_TIMER", 4, AP_MotorTempMonitor_Params, _high_temp_timeout, 10),

    // @Param: FS_TEMPSRC
    // @DisplayName: Failsafe temperature source
    // @Description: Temperature type used for detection of high temperature event
    // @Values: 0:None, 1:Indago calculation
    // @User: Advanced
    AP_GROUPINFO("FS_TEMPSRC", 5, AP_MotorTempMonitor_Params, _failsafe_temp_source, MotorTempMonitor_HighTempSource_c),

    // @Param: HIGH_TEMP
    // @DisplayName: High motor temperature threshold
    // @Description: Motor temperature that triggers a high temperature failsafe. Set to 0 to disable. If the motor temperature exceeds this value continuously for more then the period specified by the @PREFIX@HIGH_TIMER parameter then the vehicle will perform the failsafe specified by the @PREFIX@FS_HIGH_ACT parameter.
    // @Units: DEG C
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("HIGH_TEMP", 6, AP_MotorTempMonitor_Params, _high_temp_c, 0),

    // @Param: CRT_VOLT
    // @DisplayName: Critical battery voltage
    // @Description: Motor temperature that triggers a critical temperature failsafe. Set to 0 to disable. If the motor temperature exceeds this value continuously for more then the period specified by the @PREFIX@HIGH_TIMER parameter then the vehicle will perform the failsafe specified by the @PREFIX@FS_CRT_ACT parameter.
    // @Units: C
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("CRT_TEMP", 7, AP_MotorTempMonitor_Params, _critical_temp_c, 0),

    // @Param: FS_HIGH_ACT
    // @DisplayName: High motor temperature failsafe action
    // @Description: What action the vehicle should perform if a motor hits a high temperature failsafe
    // @Values{Plane}: 0:None,1:RTL,2:Land,3:Terminate,4:QLand
    // @Values{Copter}: 0:None,1:Land,2:RTL,3:SmartRTL or RTL,4:SmartRTL or Land,5:Terminate
    // @Values{Sub}: 0:None,2:Disarm,3:Enter surface mode
    // @Values{Rover}: 0:None,1:RTL,2:Hold,3:SmartRTL,4:SmartRTL or Hold,5:Terminate
    // @Values{Tracker}: 0:None
    // @User: Standard
    AP_GROUPINFO("FS_HIGH_ACT", 8, AP_MotorTempMonitor_Params, _failsafe_high_action, 0),

    // @Param: FS_CRT_ACT
    // @DisplayName: Critical motor temperature failsafe action
    // @Description: What action the vehicle should perform if a motor hits a critical motor temperature failsafe
    // @Values{Plane}: 0:None,1:RTL,2:Land,3:Terminate,4:QLand,5:Parachute
    // @Values{Copter}: 0:None,1:Land,2:RTL,3:SmartRTL or RTL,4:SmartRTL or Land,5:Terminate
    // @Values{Sub}: 0:None,2:Disarm,3:Enter surface mode
    // @Values{Rover}: 0:None,1:RTL,2:Hold,3:SmartRTL,4:SmartRTL or Hold,5:Terminate
    // @Values{Tracker}: 0:None
    // @User: Standard
    AP_GROUPINFO("FS_CRT_ACT", 9, AP_MotorTempMonitor_Params, _failsafe_critical_action, 0),

    AP_GROUPEND

};

AP_MotorTempMonitor_Params::AP_MotorTempMonitor_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
