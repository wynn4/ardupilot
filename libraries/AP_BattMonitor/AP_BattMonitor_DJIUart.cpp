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
#include "AP_BattMonitor_DJIUART.h"

extern const AP_HAL::HAL &hal;

void AP_BattMonitor_DJIUART::init(void)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_BatteryDJIUART, 0));

    if (_port != nullptr) {
    }
}

void AP_BattMonitor_DJIUART::read(void)
{
    if(_port == nullptr) {
        return; //Nothing to do here
    }

    //Send a request for new data every second
    uint64_t now = AP_HAL::micros64();
    if(now >= _next_request_t_us) {
        //Send a request on the port
        _port->send();
        _next_request_t_us = now + 1E6;
    }

    //Read some bytes from the serial
    uint32_t available = port->available();
    for(uint32_t i=0; i<available; ++i) {
      uint8_t b = _port->read();
      _parse(b);
    }
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
    if(_rx_buffer[3] == 0x22) {
        int soc = rxbuf[17] | (rxbuf[18]<<8);
        int current = rxbuf[9] | (rxbuf[10]<<8);
        int filtcurrent = rxbuf[13] | (rxbuf[14]<<8);
        int cells[4];
        cells[0] = rxbuf[19] | (rxbuf[20]<<8);
        cells[1] = rxbuf[21] | (rxbuf[22]<<8);
        cells[2] = rxbuf[23] | (rxbuf[24]<<8);
        cells[3] = rxbuf[25] | (rxbuf[26]<<8);
    }

}
