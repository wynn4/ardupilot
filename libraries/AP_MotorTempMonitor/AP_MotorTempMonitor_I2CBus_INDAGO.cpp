#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorTempMonitor.h"
#include "AP_MotorTempMonitor_I2CBus_INDAGO.h"
#include <utility>

AP_MotorTempMonitor_I2CBus_INDAGO::AP_MotorTempMonitor_I2CBus_INDAGO(AP_MotorTempMonitor &mon,
                                                                     AP_MotorTempMonitor::MotorTempMonitor_State &mon_state,
                                                                     AP_MotorTempMonitor_Params &params,
                                                                     AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev) : AP_MotorTempMonitor_I2CBus(mon, mon_state, params, std::move(dev))
{
  _has_cell_voltages = false;
}

void AP_MotorTempMonitor_I2CBus_INDAGO::timer()
{

    read_temp();

}

bool AP_MotorTempMonitor_I2CBus_INDAGO::read_temp(void)
{
    uint8_t buf;

    if(!_dev->read_registers(0x2, &buf, sizeof(buf))) {
      return false;
    }

        _state.temperature_time = AP_HAL::millis();
        _state.temp_c = (float)(buf)/_params._deg_c_divisor +_params._deg_c_offset;
        return true;
}
