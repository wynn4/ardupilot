#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "AP_MotorTempMonitor_Params.h"
#include "AP_MotorTempMonitor_Analog.h"

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
  #define DEFAULT_LOW_BATTERY_VOLTAGE 10.5f
#else
  #define DEFAULT_LOW_BATTERY_VOLTAGE 0.0f
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)

const AP_Param::GroupInfo AP_MotorTempMonitor_Params::var_info[] = {
    // @Param: MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Analog Voltage Only,4:Analog Voltage and Current,5:Solo,6:Bebop,7:SMBus-Maxell,8:UAVCAN-BatteryInfo,9:BLHeli ESC,10:SumOfFollowing,11:FuelFlow,12:FuelLevelPWM
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("MONITOR", 1, AP_MotorTempMonitor_Params, _type, MotorTempMonitor_TYPE_NONE, AP_PARAM_FLAG_ENABLE),

    // @Param: VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Sets the analog input pin that should be used for voltage monitoring.
    // @Values: -1:Disabled, 2:Pixhawk/Pixracer/Navio2/Pixhawk2_PM1, 13:Pixhawk2_PM2, 14:CubeOrange, 13:CubeOrange_PM2, 16:Durandal, 100:PX4-v1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("VOLT_PIN", 2, AP_MotorTempMonitor_Params, _volt_pin, AP_BATT_VOLT_PIN),

    // @Param: CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Sets the analog input pin that should be used for current monitoring.
    // @Values: -1:Disabled, 3:Pixhawk/Pixracer/Navio2/Pixhawk2_PM1, 14:Pixhawk2_PM2, 15:CubeOrange, 4:CubeOrange_PM2, 17:Durandal, 101:PX4-v1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("CURR_PIN", 3, AP_MotorTempMonitor_Params, _curr_pin, AP_BATT_CURR_PIN),

    // @Param: VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin (@PREFIX@VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.
    // @User: Advanced
    AP_GROUPINFO("TEMP_DIV", 4, AP_MotorTempMonitor_Params, _deg_c_divisor, AP_MOT_TEMPDIVIDER_DEFAULT),

    // @Param: AMP_PERVLT
    // @DisplayName: Amps per volt
    // @Description: Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.
    // @Units: A/V
    // @User: Standard
    AP_GROUPINFO("AMP_PERVLT", 5, AP_MotorTempMonitor_Params, _curr_amp_per_volt, AP_BATT_CURR_AMP_PERVOLT_DEFAULT),

    // @Param: AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor
    // @Units: V
    // @User: Standard
    AP_GROUPINFO("TEMP_OFFSET", 6, AP_MotorTempMonitor_Params, _deg_c_offset, 0),

    // @Param: CAPACITY
    // @DisplayName: Battery capacity
    // @Description: Capacity of the battery in mAh when full
    // @Units: mAh
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("CAPACITY", 7, AP_MotorTempMonitor_Params, _pack_capacity, 3300),

    // @Param: WATT_MAX
    // @DisplayName: Maximum allowed power (Watts)
    // @Description: If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.
    // @Units: W
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO_FRAME("WATT_MAX", 8, AP_MotorTempMonitor_Params, _watt_max, 0, AP_PARAM_FRAME_PLANE),

    // @Param: SERIAL_NUM
    // @DisplayName: Battery serial number
    // @Description: Battery serial number, automatically filled in for SMBus batteries, otherwise will be -1. With UAVCAN it is the battery_id.
    // @User: Advanced
    AP_GROUPINFO("SERIAL_NUM", 9, AP_MotorTempMonitor_Params, _serial_number, AP_BATT_SERIAL_NUMBER_DEFAULT),

    // @Param: HIGH_TIMER
    // @DisplayName: Low voltage timeout
    // @Description: This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.
    // @Units: s
    // @Increment: 1
    // @Range: 0 120
    // @User: Advanced
    AP_GROUPINFO("HIGH_TIMER", 10, AP_MotorTempMonitor_Params, _high_temp_timeout, 10),

    // @Param: FS_VOLTSRC
    // @DisplayName: Failsafe voltage source
    // @Description: Voltage type used for detection of low voltage event
    // @Values: 0:Raw Voltage, 1:Sag Compensated Voltage
    // @User: Advanced
    AP_GROUPINFO("FS_VOLTSRC", 11, AP_MotorTempMonitor_Params, _failsafe_temp_source, MotorTempMonitor_HighTempSource_deg_C),

    // @Param: HIGH_TEMP
    // @DisplayName: Low battery voltage
    // @Description: Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the @PREFIX@LOW_TIMER parameter then the vehicle will perform the failsafe specified by the @PREFIX@FS_LOW_ACT parameter.
    // @Units: DEG C
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("HIGH_TEMP", 13, AP_MotorTempMonitor_Params, _high_temp_deg_c, 0),

    // @Param: CRT_VOLT
    // @DisplayName: Critical battery voltage
    // @Description: Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the @PREFIX@LOW_TIMER parameter then the vehicle will perform the failsafe specified by the @PREFIX@FS_CRT_ACT parameter.
    // @Units: V
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("CRT_TEMP", 14, AP_MotorTempMonitor_Params, _critical_temp_deg_c, 0),

    // @Param: FS_HIGH_ACT
    // @DisplayName: Low battery failsafe action
    // @Description: What action the vehicle should perform if it hits a low battery failsafe
    // @Values{Plane}: 0:None,1:RTL,2:Land,3:Terminate,4:QLand
    // @Values{Copter}: 0:None,1:Land,2:RTL,3:SmartRTL or RTL,4:SmartRTL or Land,5:Terminate
    // @Values{Sub}: 0:None,2:Disarm,3:Enter surface mode
    // @Values{Rover}: 0:None,1:RTL,2:Hold,3:SmartRTL,4:SmartRTL or Hold,5:Terminate
    // @Values{Tracker}: 0:None
    // @User: Standard
    AP_GROUPINFO("FS_HIGH_ACT", 16, AP_MotorTempMonitor_Params, _failsafe_high_action, 0),

    // @Param: FS_CRT_ACT
    // @DisplayName: Critical battery failsafe action
    // @Description: What action the vehicle should perform if it hits a critical battery failsafe
    // @Values{Plane}: 0:None,1:RTL,2:Land,3:Terminate,4:QLand,5:Parachute
    // @Values{Copter}: 0:None,1:Land,2:RTL,3:SmartRTL or RTL,4:SmartRTL or Land,5:Terminate
    // @Values{Sub}: 0:None,2:Disarm,3:Enter surface mode
    // @Values{Rover}: 0:None,1:RTL,2:Hold,3:SmartRTL,4:SmartRTL or Hold,5:Terminate
    // @Values{Tracker}: 0:None
    // @User: Standard
    AP_GROUPINFO("FS_CRT_ACT", 17, AP_MotorTempMonitor_Params, _failsafe_critical_action, 0),

    AP_GROUPEND

};

AP_MotorTempMonitor_Params::AP_MotorTempMonitor_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
