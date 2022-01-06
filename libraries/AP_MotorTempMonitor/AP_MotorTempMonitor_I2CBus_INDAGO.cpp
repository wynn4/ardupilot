#include "AP_MotorTempMonitor_I2CBus_INDAGO.h"

AP_MotorTempMonitor_I2CBus_INDAGO::AP_MotorTempMonitor_I2CBus_INDAGO()
{

}

void AP_MotorTempMonitor_I2CBus_INDAGO::init(void) {
    if (_dev) {
        _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_MotorTempMonitor_I2CBus_INDAGO::timer, void));
    }
}

bool AP_MotorTempMonitor_I2CBus_INDAGO::read_word(uint8_t reg, uint16_t& data) const
{
    // buffer to hold results (1 extra byte returned holding PEC)
    const uint8_t read_size = 2 + (_pec_supported ? 1 : 0);
    uint8_t buff[read_size];    // buffer to hold results

    // read the appropriate register from the device
    if (!_dev->read_registers(reg, buff, sizeof(buff))) {
        return false;
    }

    // check PEC
    if (_pec_supported) {
        const uint8_t pec = get_PEC(AP_BATTMONITOR_SMBUS_I2C_ADDR, reg, true, buff, 2);
        if (pec != buff[2]) {
            return false;
        }
    }

    // convert buffer to word
    data = (uint16_t)buff[1]<<8 | (uint16_t)buff[0];

    // return success
    return true;
}
