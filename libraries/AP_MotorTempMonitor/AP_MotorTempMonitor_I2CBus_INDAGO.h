#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorTempMonitor_I2CBus.h"
#include <AP_HAL/I2CDevice.h>

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

    // read_block - returns number of characters read if successful, zero if unsuccessful
    uint8_t read_block(uint8_t reg, uint8_t* data, uint8_t max_len, bool append_zero) const;

    uint8_t _button_press_count;
};
