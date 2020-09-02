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


#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_BattMonitor_DJIUART.h"

extern const AP_HAL::HAL &hal;

void AP_BattMonitor_DJIUART::init(void)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_BatteryDJIUART, 0);

    _state.healthy = false;
}

void AP_BattMonitor_DJIUART::read(void)
{
    //Request message
    const uint8_t request_status_msg[] = {0xab, 0x0e, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd4};
    if(_port == nullptr) {
        return; //Nothing to do here
    }

    //Send a request for new data 2x second
    uint64_t now = AP_HAL::micros64();
    if(now >= _next_request_t_us) {
        //Send a request on the port
        _port->write(request_status_msg,sizeof(request_status_msg));
        _next_request_t_us = now + 5E5;
    }

    //Read some bytes from the serial
    uint32_t available = _port->available();
    for(uint32_t i=0; i<available; ++i) {
      uint8_t b = _port->read();
      _parse(b);
    }

    //Look at the last update, determine the health based on a timeout
    _state.healthy = (now - _state.last_time_micros) < 250000U;
}

void AP_BattMonitor_DJIUART::_parse(uint8_t b) {
    if(_rx_buffer_idx >= (RX_BUFFER_LEN - 1)) {
        _rx_buffer_idx = 0; //Wrap the buffer index
    }

    //Push a byte into the buffer
    _rx_buffer[_rx_buffer_idx++] = b;

    switch(_parse_state) {
        case PARSE_STATE_IDLE:
          if(b == 0xAB) {
              _parse_state = PARSE_STATE_GOT_START;
              _this_msg_received_len = 1;
          }
          break;
        case PARSE_STATE_GOT_START:
          _this_msg_len = b;
          _parse_state = PARSE_STATE_GOT_LEN;
          _this_msg_received_len++;
          break;
        case PARSE_STATE_GOT_LEN:
          if(++_this_msg_received_len >= _this_msg_len) {
              //Received a full message, decode it
              _decode();

              _parse_state = PARSE_STATE_IDLE;
              _this_msg_len = 0;
              _rx_buffer_idx = 0;
          }
          break;
        default:
          break;
    }
}

void AP_BattMonitor_DJIUART::_decode() {
    //If this is a response (0x22) message, check its validity then pull out votlage/soc/current/etc data
    if(_rx_buffer[3] == 0x22 && _this_msg_len == 37) {
        uint32_t now = AP_HAL::micros();

        uint16_t cells[4];
        //First 4 bytes are header data
        uint16_t temperature = _rx_buffer[5] | (_rx_buffer[6] << 8);
        uint16_t voltage =     _rx_buffer[7] | (_rx_buffer[8] << 8);
        //int32_t current =      _rx_buffer[9] | (_rx_buffer[10] << 8) | (_rx_buffer[11] << 16) | (_rx_buffer[12] << 24);
        int32_t curr_filt =    _rx_buffer[13] | (_rx_buffer[14] << 8) | (_rx_buffer[15] << 16) | (_rx_buffer[16] << 24);
        uint16_t soc =         _rx_buffer[17] | (_rx_buffer[18]<<8);
        cells[0] =             _rx_buffer[19] | (_rx_buffer[20]<<8);
        cells[1] =             _rx_buffer[21] | (_rx_buffer[22]<<8);
        cells[2] =             _rx_buffer[23] | (_rx_buffer[24]<<8);
        cells[3] =             _rx_buffer[25] | (_rx_buffer[26]<<8);
        uint8_t power_stat =   _rx_buffer[31];

        _state.voltage = voltage/1.E3;
        _state.current_amps = curr_filt/1.E3;
        _state.cell_voltages.cells[0] = cells[0];
        _state.cell_voltages.cells[1] = cells[1];
        _state.cell_voltages.cells[2] = cells[2];
        _state.cell_voltages.cells[3] = cells[3];
        _state.temperature = temperature/1.E2;
        _state.is_powering_off = (power_stat == 0xFF);
        if(!_state.is_powering_off) {
            _state.powerOffNotified = false; //Reset the flag
        }

        _pct_remaining = (uint8_t)soc;
        _state.last_time_micros = now;
        _state.temperature_time = AP_HAL::millis();

        //hal.console->printf("SOC: %i (%fV/%fA) [ %f,%f,%f,%f ] temp: %f powering_off: %02X\n",soc,_state.voltage,_state.current_amps,cells[0]/1.e3,cells[1]/1.e3,cells[2]/1.e3,cells[3]/1.e3,_state.temperature,power_stat);
    }
}
