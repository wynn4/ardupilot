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
    _got_type_response = false;
}

void AP_BattMonitor_DJIUART::read(void)
{
    //Request message
    const uint8_t request_status_msg[] = {0xab, 0x0e, 0x00, STATUS_MSG_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd4};
    const uint8_t request_type_msg[] =   {0xab, 0x0e, 0x00, TYPE_MSG_ID,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6c};

    if(_port == nullptr) {
        return; //Nothing to do here
    }

    //Send a request for new data 2x second
    uint64_t now = AP_HAL::micros64();
    if(now >= _next_request_t_us) {
        //Send a request on the port
        _port->write(request_status_msg,sizeof(request_status_msg));
        _next_request_t_us = now + 5E5;
        if(!_got_type_response) {
          _port->write(request_type_msg,sizeof(request_type_msg));
        }
    }

    //Read some bytes from the serial
    uint32_t available = _port->available();
    for(uint32_t i=0; i<available; ++i) {
      uint8_t b = _port->read();
      _parse(b);
    }

    //Look at the last update, determine the health based on a timeout
    _state.healthy = (AP_HAL::micros() - _state.last_time_micros) < 2000000U;
    if(!_state.healthy) {
        _got_type_response = false; //Be sure to re-request the type data if the battery times out
    }
}

void AP_BattMonitor_DJIUART::_parse(uint8_t b) {
    if(_rx_buffer_idx >= (RX_BUFFER_LEN - 1)) {
        _rx_buffer_idx = 0; //Wrap the buffer index
        memset(_rx_buffer,0,RX_BUFFER_LEN);
    }

    //Push a byte into the buffer
    _rx_buffer[_rx_buffer_idx++] = b;

    switch(_parse_state) {
        case PARSE_STATE_IDLE:
          if(b == 0xAB) {
              _parse_state = PARSE_STATE_GOT_START;
          } else {
              _rx_buffer_idx = 0;
          }
          break;

        case PARSE_STATE_GOT_START:
          //Sanity check, len must be > 6 bytes, can't be more than, lets say 100 bytes
          if(b <= 6 || b>=100) {
              _parse_state = PARSE_STATE_IDLE;
              _rx_buffer_idx = 0;
          }
          _parse_state = PARSE_STATE_GOT_LEN;
          break;

        case PARSE_STATE_GOT_LEN:
          if(_rx_buffer_idx >= _rx_buffer[1]) { //_rx_buffer[1] is the length
              //TODO: verify the CRC (once we figure out the CRC algo...)

              //Received a full message, decode it
              switch(_rx_buffer[3]) {
                  case TYPE_MSG_ID:
                    _handle_type_message();
                    break;
                  case STATUS_MSG_ID:
                    _handle_status_message();
                    break;
                  default:
                    break;
              }
              _parse_state = PARSE_STATE_IDLE;
              _rx_buffer_idx = 0;
          }
          break;
        default:
          break;
    }
}

void AP_BattMonitor_DJIUART::_handle_type_message() {
    if(_rx_buffer[3] != TYPE_MSG_ID || _rx_buffer[1] != TYPE_MSG_LEN) {
        return;
    }

   _nom_capacity_amps =  (float)(_rx_buffer[9]  | (_rx_buffer[10] << 8))/1.E3;
   _nom_voltage =        (float)(_rx_buffer[11] | (_rx_buffer[12] << 8))/1.E3;

   _params._pack_capacity = (int32_t)(_nom_capacity_amps*1.E3);
   _params._serial_number = (int32_t)(_rx_buffer[17] | (_rx_buffer[18] << 8));
   _got_type_response = true;
}

void AP_BattMonitor_DJIUART::_handle_status_message() {
    if(_rx_buffer[3] != STATUS_MSG_ID || _rx_buffer[1] != STATUS_MSG_LEN) {
        return;
    }

    //First 4 bytes are header data
    uint16_t temperature =          _rx_buffer[5] | (_rx_buffer[6] << 8);
    uint16_t voltage =              _rx_buffer[7] | (_rx_buffer[8] << 8);
    //int32_t current =             _rx_buffer[9] | (_rx_buffer[10] << 8) | (_rx_buffer[11] << 16) | (_rx_buffer[12] << 24);
    int32_t curr_filt =             _rx_buffer[13] | (_rx_buffer[14] << 8) | (_rx_buffer[15] << 16) | (_rx_buffer[16] << 24);
    uint16_t soc =                  _rx_buffer[17] | (_rx_buffer[18]<<8);
    _state.cell_voltages.cells[0] = _rx_buffer[19] | (_rx_buffer[20]<<8);
    _state.cell_voltages.cells[1] = _rx_buffer[21] | (_rx_buffer[22]<<8);
    _state.cell_voltages.cells[2] = _rx_buffer[23] | (_rx_buffer[24]<<8);
    _state.cell_voltages.cells[3] = _rx_buffer[25] | (_rx_buffer[26]<<8);
    uint8_t power_stat =            _rx_buffer[31];

    _state.voltage = voltage/1.E3;
    _state.current_amps = -1. * (curr_filt/1.E3); //Reports a negative current.
    _state.temperature = temperature/1.E2;
    _state.is_powering_off = (power_stat == 0xFF);
    if(!_state.is_powering_off) {
        _state.powerOffNotified = false; //Reset the flag
    }

    _pct_remaining = (uint8_t)soc;
    _state.last_time_micros = AP_HAL::micros();
    _state.temperature_time = AP_HAL::millis();

    //Update the consumed mah/wh based on the SoC %, capacity and voltage
    if(_got_type_response) {
        float soc_pct = (float)soc/100.;
        float nom_wattage = (float)_nom_capacity_amps * _nom_voltage;
        _state.consumed_mah = (float)_nom_capacity_amps * 1.E3 * (1. - soc_pct);
        _state.consumed_wh = nom_wattage * (1. - soc_pct);
    }
}
