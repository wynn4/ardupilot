#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_DJIUART:public AP_BattMonitor_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_BattMonitor_DJIUART(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params):
        AP_BattMonitor_Backend(mon, mon_state, params)
    {};

    virtual ~AP_BattMonitor_DJIUART(void) {};
    void init() override;
    void read() override;
    bool has_current() const override { return true; };
    bool has_cell_voltages() const override { return true; };

private:
    AP_HAL::UARTDriver *_port;

    uint64_t _next_request_t_us = 0;

    void _parse(uint8_t b);
    void _decode();

    enum {
        PARSE_STATE_IDLE,
        PARSE_STATE_GOT_START,
        PARSE_STATE_GOT_LEN,
    };
    uint8_t _parse_state = PARSE_STATE_IDLE;

    const uint8_t RX_BUFFER_LEN=128;
    uint8_t _rx_buffer[RX_BUFFER_LEN];
    uint8_t _rx_buffer_idx = 0;
    uint8_t _this_msg_len = 0;
};
