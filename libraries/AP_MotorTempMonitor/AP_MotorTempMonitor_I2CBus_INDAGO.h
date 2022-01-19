#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorTempMonitor_I2CBus.h"
#include <AP_HAL/I2CDevice.h>

#define AP_MOTORTEMPMONITOR_I2C_BUS_INTERNAL           0
#define AP_MOTORTEMPMONITOR_SMBUS_BUS_EXTERNAL           1
#define AP_MOTORTEMPMONITOR_I2C_ADDR               0x51
#define AP_MOTORTEMPMONITOR_I2C_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds

class AP_MotorTempMonitor_I2CBus_INDAGO : public AP_MotorTempMonitor_I2CBus
{
public:

    // Constructor
    AP_MotorTempMonitor_I2CBus_INDAGO(AP_MotorTempMonitor &mon,
                             AP_MotorTempMonitor::MotorTempMonitor_State &mon_state,
                             AP_MotorTempMonitor_Params &params,
                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

private:

    void timer(void) override;
    bool read_temp(void);


};
