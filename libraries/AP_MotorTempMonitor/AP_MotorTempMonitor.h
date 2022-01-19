#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_MotorTempMonitor_Params.h"

// maximum number of battery monitors
#define AP_MOT_TEMP_MONITOR_MAX_INSTANCES       4

// first monitor is always the primary monitor
#define AP_MOT_TEMP_PRIMARY_INSTANCE            0

#define AP_MOT_TEMP_SERIAL_NUMBER_DEFAULT       -1

#define AP_MOT_TEMP_MONITOR_TIMEOUT             5000

#define AP_MOT_TEMP_MONITOR_RES_EST_TC_1        0.5f
#define AP_MOT_TEMP_MONITOR_RES_EST_TC_2        0.1f

// declare backend class
class AP_MotorTempMonitor_Backend;
class AP_MotorTempMonitor_I2CBus;
class AP_MotorTempMonitor_I2CBus_INDAGO;

class AP_MotorTempMonitor
{
    friend class AP_MotorTempMonitor_Backend;
//    friend class AP_MotorTempMonitor_Analog;
//    friend class AP_MotorTempMonitor_Analog_GPIO;
    friend class AP_MotorTempMonitor_I2CBus;
    friend class AP_MotorTempMonitor_I2CBus_INDAGO;
//    friend class AP_MotorTempMonitor_SMBus_Maxell;
//    friend class AP_MotorTempMonitor_UAVCAN;
//    friend class AP_MotorTempMonitor_Sum;
//    friend class AP_MotorTempMonitor_FuelFlow;
//    friend class AP_AP_MotorTempMonitorMonitor_FuelLevel_PWM;

public:

    // battery failsafes must be defined in levels of severity so that vehicles wont fall backwards
    enum MotorTempFailsafe {
        MotorTempFailsafe_None = 0,
        MotorTempFailsafe_High,
        MotorTempFailsafe_Critical
    };

    FUNCTOR_TYPEDEF(motor_temp_failsafe_handler_fn_t, void, const char *, const int8_t);

    AP_MotorTempMonitor(uint32_t log_motor_temp_bit, motor_temp_failsafe_handler_fn_t motor_temp_failsafe_handler_fn, const int8_t *failsafe_priorities);

    /* Do not allow copies */
    AP_MotorTempMonitor(const AP_MotorTempMonitor &other) = delete;
    AP_MotorTempMonitor &operator=(const AP_MotorTempMonitor&) = delete;

    static AP_MotorTempMonitor *get_singleton() {
        return _singleton;
    }


    // The BattMonitor_State structure is filled in by the backend driver
    struct MotorTempMonitor_State {
        uint32_t    last_time_micros;          // time when voltage and current was last read in microseconds
        uint32_t    high_temp_start_ms;      // time when voltage dropped below the minimum in milliseconds
        uint32_t    critical_temp_start_ms; // critical voltage failsafe start timer in milliseconds
        float       temp_c;               // motor temperature in degrees Celsius
        uint32_t    temperature_time;          // timestamp of the last received temperature message
        MotorTempFailsafe failsafe;              // stage failsafe the battery is in
        bool        healthy;                   // battery monitor is communicating correctly
    };

    // Return the number of battery monitor instances
    uint8_t num_instances(void) const { return _num_instances; }

    // detect and initialise any available battery monitors
    void init();

    /// Read the battery voltage and current for all batteries.  Should be called at 10hz
    void read();

    // healthy - returns true if monitor is functioning
    bool healthy(uint8_t instance) const;
    bool healthy() const { return healthy(AP_MOT_TEMP_PRIMARY_INSTANCE); }

    /// returns true if a battery failsafe has ever been triggered
    bool has_failsafed(void) const { return _has_triggered_failsafe; };

    /// returns the highest failsafe action that has been triggered
    int8_t get_highest_failsafe_priority(void) const { return _highest_failsafe_priority; };

    /// get_type - returns battery monitor type
    enum AP_MotorTempMonitor_Params::MotorTempMonitor_Type get_type() const { return get_type(AP_MOT_TEMP_PRIMARY_INSTANCE); }
    enum AP_MotorTempMonitor_Params::MotorTempMonitor_Type get_type(uint8_t instance) const { return _params[instance].type(); }

    // temperature
    bool get_temperature(float &temperature) const { return get_temperature(temperature, AP_MOT_TEMP_PRIMARY_INSTANCE); };
    bool get_temperature(float &temperature, const uint8_t instance) const;
    float temperature_c(uint8_t instance) const;

      // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    bool arming_checks(size_t buflen, char *buffer) const;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    /// parameters
    AP_MotorTempMonitor_Params _params[AP_MOT_TEMP_MONITOR_MAX_INSTANCES];

private:
    static AP_MotorTempMonitor *_singleton;

    MotorTempMonitor_State state[AP_MOT_TEMP_MONITOR_MAX_INSTANCES];
    AP_MotorTempMonitor_Backend *drivers[AP_MOT_TEMP_MONITOR_MAX_INSTANCES];
    uint32_t    _log_motor_temp_bit;
    uint8_t     _num_instances;                                     /// number of monitors


    /// returns the failsafe state of the battery
    MotorTempFailsafe check_failsafe(const uint8_t instance);
    void check_failsafes(void); // checks all batteries failsafes

    motor_temp_failsafe_handler_fn_t _motor_temp_failsafe_handler_fn;
    const int8_t *_failsafe_priorities; // array of failsafe priorities, sorted highest to lowest priority, -1 indicates no more entries

    int8_t      _highest_failsafe_priority; // highest selected failsafe action level (used to restrict what actions we move into)
    bool        _has_triggered_failsafe;  // true after a battery failsafe has been triggered for the first time

};

namespace AP {
    AP_MotorTempMonitor &motor_temp_mon();
};
