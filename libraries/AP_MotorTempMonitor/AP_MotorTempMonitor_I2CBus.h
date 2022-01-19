#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorTempMonitor_Backend.h"
#include <utility>

#define AP_BATTMONITOR_SMBUS_BUS_INTERNAL           0
#define AP_BATTMONITOR_SMBUS_BUS_EXTERNAL           1
#define AP_BATTMONITOR_SMBUS_I2C_ADDR               0x0B
#define AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds

class AP_MotorTempMonitor_I2CBus : public AP_MotorTempMonitor_Backend
{
public:


    /// Constructor
    AP_MotorTempMonitor_I2CBus(AP_MotorTempMonitor &mon,
                    AP_MotorTempMonitor::MotorTempMonitor_State &mon_state,
                    AP_MotorTempMonitor_Params &params,
                    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // virtual destructor to reduce compiler warnings
    virtual ~AP_MotorTempMonitor_I2CBus() {}

    void init(void) override;

protected:


//    // reads the temperature from the motor
//    // returns true if the read was successful
//    bool read_temp(void);

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    bool _has_cell_voltages;        // smbus backends flag this as true once they have recieved a valid cell voltage report

    virtual void timer(void) = 0;   // timer function to read from the battery

};

// include specific implementations
#include "AP_MotorTempMonitor_I2CBus_INDAGO.h"
