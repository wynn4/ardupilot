/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorTempMonitor.h"
#include "AP_MotorTempMonitor_Backend.h"

/*
  base class constructor.
  This incorporates initialisation as well.
*/
AP_MotorTempMonitor_Backend::AP_MotorTempMonitor_Backend(AP_MotorTempMonitor &mon, AP_MotorTempMonitor::MotorTempMonitor_State &mon_state,
                                               AP_MotorTempMonitor_Params &params) :
        _mon(mon),
        _state(mon_state),
        _params(params)
{
}


AP_MotorTempMonitor::MotorTempFailsafe AP_MotorTempMonitor_Backend::update_failsafes(void)
{
    const uint32_t now = AP_HAL::millis();

    bool high_temp, critical_temp;
    check_failsafe_types(high_temp, critical_temp);

    if (critical_temp) {
        // this is the first time our voltage has dropped below minimum so start timer
        if (_state.critical_temp_start_ms == 0) {
            _state.critical_temp_start_ms = now;
        } else if (_params._high_temp_timeout > 0 &&
                   now - _state.critical_temp_start_ms > uint32_t(_params._high_temp_timeout)*1000U) {
            return AP_MotorTempMonitor::MotorTempFailsafe_Critical;
        }
    } else {
        // not critical temp so reset timer
        _state.critical_temp_start_ms = 0;
    }


    if (high_temp) {
        // this is the first time our voltage has dropped below minimum so start timer
        if (_state.high_temp_start_ms == 0) {
            _state.high_temp_start_ms = now;
        } else if (_params._high_temp_timeout > 0 &&
                   now - _state.high_temp_start_ms > uint32_t(_params._high_temp_timeout)*1000U) {
            return AP_MotorTempMonitor::MotorTempFailsafe_High;
        }
    } else {
        // acceptable voltage so reset timer
        _state.high_temp_start_ms = 0;
    }

    // if we've gotten this far then battery is ok
    return AP_MotorTempMonitor::MotorTempFailsafe_None;
}

static bool update_check(size_t buflen, char *buffer, bool failed, const char *message)
{
    if (failed) {
        strncpy(buffer, message, buflen);
        return false;
    }
    return true;
}


bool AP_MotorTempMonitor_Backend::arming_checks(char * buffer, size_t buflen) const
{
    bool high_temp, critical_temp;
    check_failsafe_types(high_temp, critical_temp);


    bool fs_temperature_inversion = is_positive(_params._high_temp_c) &&
                                 is_positive(_params._critical_temp_c) &&
                                 (_params._critical_temp_c < _params._high_temp_c);


    bool result = update_check(buflen, buffer, high_temp,  "high temperature failsafe");
    result = result && update_check(buflen, buffer, critical_temp, "critical temperature failsafe");
    result = result && update_check(buflen, buffer, fs_temperature_inversion, "high failsafe critical < high");

    return result;
}

void AP_MotorTempMonitor_Backend::check_failsafe_types(bool &high_temp, bool &critical_temp) const
{
    // use voltage or sag compensated voltage
    float motor_temp;
    switch (_params.failsafe_temp_source()) {
        case AP_MotorTempMonitor_Params::MotorTempMonitor_HighTempSource_None:
        default:
            motor_temp = _params._min_temp_c;
            break;
        case AP_MotorTempMonitor_Params::MotorTempMonitor_HighTempSource_c:
            motor_temp = _state.temp_c;
            break;
    }

    // check critical battery levels
    if ((motor_temp > _params._min_temp_c) && (_params._critical_temp_c > _params._min_temp_c) && (motor_temp < _params._critical_temp_c)) {
        critical_temp = true;
    } else {
        critical_temp = false;
    }


    if ((motor_temp > _params._min_temp_c) && (_params._high_temp_c > _params._min_temp_c) && (motor_temp < _params._high_temp_c)) {
        high_temp = true;
    } else {
        high_temp = false;
    }
}
