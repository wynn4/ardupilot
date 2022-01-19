#include "AP_MotorTempMonitor.h"
#include "AP_MotorTempMonitor_I2CBus.h"
//#include "AP_MotorTempMonitor_Analog.h"
//#include "AP_MotorTempMonitor_SMBus.h"
//#include "AP_MotorTempMonitor_Bebop.h"
//#include "AP_MotorTempMonitor_BLHeliESC.h"
//#include "AP_MotorTempMonitor_Sum.h"
//#include "AP_MotorTempMonitor_FuelFlow.h"
//#include "AP_MotorTempMonitor_FuelLevel_PWM.h"
//#include "AP_MotorTempMonitor_Analog_GPIO.h"

#include <AP_HAL/AP_HAL.h>

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Notify/AP_Notify.h>

extern const AP_HAL::HAL& hal;

AP_MotorTempMonitor *AP_MotorTempMonitor::_singleton;

const AP_Param::GroupInfo AP_MotorTempMonitor::var_info[] = {

    // @Group: 1_
    // @Path: AP_MotorTempMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1_", 0, AP_MotorTempMonitor, AP_MotorTempMonitor_Params),

    // @Group: 2_
    // @Path: AP_MotorTempMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 1, AP_MotorTempMonitor, AP_MotorTempMonitor_Params),

    // @Group: 3_
    // @Path: AP_MotorTempMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[2], "3_", 2, AP_MotorTempMonitor, AP_MotorTempMonitor_Params),

    // @Group: 4_
    // @Path: AP_MotorTempMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[3], "4_", 3, AP_MotorTempMonitor, AP_MotorTempMonitor_Params),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AP_MotorTempMonitor::AP_MotorTempMonitor(uint32_t log_motor_temp_bit, motor_temp_failsafe_handler_fn_t motor_temp_failsafe_handler_fn, const int8_t *failsafe_priorities) :
    _log_motor_temp_bit(log_motor_temp_bit),
    _num_instances(0),
    _motor_temp_failsafe_handler_fn(motor_temp_failsafe_handler_fn),
    _failsafe_priorities(failsafe_priorities)
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_MotorTempMonitor must be singleton");
    }
    _singleton = this;
}

// init - instantiate the battery monitors
void
AP_MotorTempMonitor::init()
{
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }

    _highest_failsafe_priority = INT8_MAX;

#ifdef HAL_MOT_TEMP_MONITOR_DEFAULT
    if (_params[0]._type == 0) {
        // we can't use set_default() as the type is used as a flag for parameter conversion
        _params[0]._type.set((AP_MotorTempMonitor_Params::MotorTempMonitor_Type)HAL_MOT_TEMP_MONITOR_DEFAULT);
    }
#endif

    // create each instance
    for (uint8_t instance=0; instance<AP_MOT_TEMP_MONITOR_MAX_INSTANCES; instance++) {

        switch (get_type(instance)) {

            case AP_MotorTempMonitor_Params::MotorTempMonitor_TYPE_INDAGO:
                drivers[instance] = new AP_MotorTempMonitor_I2CBus_INDAGO(*this, state[instance], _params[instance],
                                                                  hal.i2c_mgr->get_device(AP_MOTORTEMPMONITOR_I2C_BUS_INTERNAL, AP_MOTORTEMPMONITOR_I2C_ADDR+instance,
                                                                                          100000, true, 20));
                break;

            case AP_MotorTempMonitor_Params::MotorTempMonitor_TYPE_NONE:
            default:
                break;
        }

        // call init function for each backend
        if (drivers[instance] != nullptr) {
            drivers[instance]->init();
            // _num_instances is the index for looping over instances
            // as we always check for drivers[instances] being nullptr
            // this is safe
            _num_instances = instance + 1;
        }
    }
}

// read - read the voltage and current for all instances
void
AP_MotorTempMonitor::read()
{
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger->should_log(_log_motor_temp_bit)) {
        logger->Write_MotorTemp();
    }

    check_failsafes();
}

// healthy - returns true if monitor is functioning
bool AP_MotorTempMonitor::healthy(uint8_t instance) const {
    return instance < _num_instances && state[instance].healthy;
}

/// temperature_c - returns motor temperature in deg C
float AP_MotorTempMonitor::temperature_c(uint8_t instance) const
{
    if (instance < _num_instances) {
        return state[instance].temp_c;
    } else {
        return 0.0f;
    }
}

void AP_MotorTempMonitor::check_failsafes(void)
{
    if (hal.util->get_soft_armed()) {
        for (uint8_t i = 0; i < _num_instances; i++) {
            if (drivers[i] == nullptr) {
                continue;
            }

            const MotorTempFailsafe type = drivers[i]->update_failsafes();
            if (type <= state[i].failsafe) {
                continue;
            }

            int8_t action = 0;
            const char *type_str = nullptr;
            switch (type) {
                case AP_MotorTempMonitor::MotorTempFailsafe_None:
                    continue; // should not have been called in this case
                case AP_MotorTempMonitor::MotorTempFailsafe_High:
                    action = _params[i]._failsafe_high_action;
                    type_str = "high";
                    break;
                case AP_MotorTempMonitor::MotorTempFailsafe_Critical:
                    action = _params[i]._failsafe_critical_action;
                    type_str = "critical";
                    break;
            }

            gcs().send_text(MAV_SEVERITY_WARNING, "Motor %d is %s %.0f deg C", i + 1, type_str,
                            (double)temperature_c(i));
            _has_triggered_failsafe = true;
            AP_Notify::flags.failsafe_battery = true;
            state[i].failsafe = type;

            // map the desired failsafe action to a prioritiy level
            int8_t priority = 0;
            if (_failsafe_priorities != nullptr) {
                while (_failsafe_priorities[priority] != -1) {
                    if (_failsafe_priorities[priority] == action) {
                        break;
                    }
                    priority++;
                }

            }

            // trigger failsafe if the action was equal or higher priority
            // It's valid to retrigger the same action if a different battery provoked the event
            if (priority <= _highest_failsafe_priority) {
                _motor_temp_failsafe_handler_fn(type_str, action);
                _highest_failsafe_priority = priority;
            }
        }
    }
}


// returns true if there is a temperature reading
bool AP_MotorTempMonitor::get_temperature(float &temperature, const uint8_t instance) const
{
    if (instance >= AP_MOT_TEMP_MONITOR_MAX_INSTANCES) {
        return false;
    } else {
        temperature = state[instance].temp_c;
        return (AP_HAL::millis() - state[instance].temperature_time) <= AP_MOT_TEMP_MONITOR_TIMEOUT;
    }
}

bool AP_MotorTempMonitor::arming_checks(size_t buflen, char *buffer) const
{
    char temp_buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1] {};

    for (uint8_t i = 0; i < AP_MOT_TEMP_MONITOR_MAX_INSTANCES; i++) {
        if (drivers[i] != nullptr && !(drivers[i]->arming_checks(temp_buffer, sizeof(temp_buffer)))) {
            hal.util->snprintf(buffer, buflen, "Motor %d %s", i + 1, temp_buffer);
            return false;
        }
    }

    return true;
}

namespace AP {

AP_MotorTempMonitor &motor_temp_mon()
{
    return *AP_MotorTempMonitor::get_singleton();
}

};

