#include "AP_MotorTempMonitor_I2CBus.h"

AP_MotorTempMonitor_I2CBus::AP_MotorTempMonitor_I2CBus(AP_MotorTempMonitor &mon,
                                                       AP_MotorTempMonitor::MotorTempMonitor_State &mon_state,
                                                       AP_MotorTempMonitor_Params &params,
                                                       AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
  : AP_MotorTempMonitor_Backend(mon, mon_state, params),
  _dev(std::move(dev))
{

}

void AP_MotorTempMonitor_I2CBus::init(void) {
    if (_dev) {
        _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_MotorTempMonitor_I2CBus::timer, void));
    }
}
