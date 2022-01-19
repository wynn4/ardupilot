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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorTempMonitor.h"

class AP_MotorTempMonitor_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_MotorTempMonitor_Backend(AP_MotorTempMonitor &mon, AP_MotorTempMonitor::MotorTempMonitor_State &mon_state, AP_MotorTempMonitor_Params &params);

    // we declare a virtual destructor so that BattMonitor driver can
    // override with a custom destructor if need be
    virtual ~AP_MotorTempMonitor_Backend(void) {}

    // initialise
    virtual void init() = 0;

//    // read the latest battery voltage
//    virtual void read() = 0;

    // updates failsafe timers, and returns what failsafes are active
    AP_MotorTempMonitor::MotorTempFailsafe update_failsafes(void);

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    bool arming_checks(char * buffer, size_t buflen) const;

protected:
    AP_MotorTempMonitor                      &_mon;      // reference to front-end
    AP_MotorTempMonitor::MotorTempMonitor_State   &_state;    // reference to this instances state (held in the front-end)
    AP_MotorTempMonitor_Params               &_params;   // reference to this instances parameters (held in the front-end)

    // checks what failsafes could be triggered
    void check_failsafe_types(bool &high_temp, bool &critical_temp) const;
};
