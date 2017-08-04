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
/*
  multicopter simulator class
*/

#include "SIM_Multicopter.h"
#include <AP_Motors/AP_Motors.h>

#include <stdio.h>

using namespace SITL;

extern const AP_HAL::HAL& hal;

MultiCopter::MultiCopter(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    frame(NULL),
    /*
     * Simu Planck
     */
    _sock(true)
{

    frame = Frame::find_frame(frame_str);
    if (frame == NULL) {
        printf("Frame '%s' not found", frame_str);
        exit(1);
    }

    if (strstr(frame_str, "-fast")) {
        frame->init(1.5, 0.5, 85, 4*radians(360));
    } else {
        frame->init(1.5, 0.51, 15, 4*radians(360));
    }
    frame_height = 0.1;
    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;

    /*
     * Simu Planck
     */
    _sock.bind("0.0.0.0", planck_recv_port);
    _sock.reuseaddress();
    _sock.set_blocking(false);

}

// calculate rotational and linear accelerations
void MultiCopter::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    frame->calculate_forces(*this, input, rot_accel, body_accel);
}

/*
  update the multicopter simulation by one time step
 */
void MultiCopter::update(const struct sitl_input &input)
{
    //hal.console->printf("MultiCopter::update: %lu \n", (unsigned) AP_HAL::millis());

    // get wind vector setup
    update_wind(input);

    Vector3f rot_accel;

    calculate_forces(input, rot_accel, accel_body);

    update_dynamics(rot_accel);

    // update lat/lon/altitude
    update_position();

    // update magnetic field
    update_mag_field_bf();

    update_planck();
}


/*
 * Simu Planck
 */
void MultiCopter::update_planck()
{

    // Create simulation packet
    simu_planck_t pkt;
    pkt.time_simu_us = time_now_us;                         // Time is us of the simulation clock - defined by Ardupilot
    pkt.origin_lat = home.lat * 1.0e-7;                     // Lat - Origin of the Local NED frame used by the simulation back end (SIM_Aircraft.cpp)
    pkt.origin_lon = home.lng * 1.0e-7;                     // Lon - Origin of the Local NED frame used by the simulation back end (SIM_Aircraft.cpp)
    pkt.origin_alt_amsl = ((double)home.alt) * 1.0e-2;      // Alt (amsl) - Origin of the Local NED frame used by the simulation back end (SIM_Aircraft.cpp)
    pkt.alt_amsl = ((double)location.alt) * 1.0e-2;         // Alt (amsl) of the aircraft
    pkt.pos_n = position.x;                                 // X Position of the aircraft - In Local NED frame defined just above
    pkt.pos_e = position.y;                                 // Y Position of the aircraft - In Local NED frame defined just above
    pkt.pos_d = position.z;                                 // Z Position of the aircraft - In Local NED frame defined just above
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    pkt.roll = r;                                           // Roll (rad) - Attitude of the Aircraft
    pkt.pitch = p;                                          // Pitch (rad) - Attitude of the Aircraft
    pkt.yaw = y;                                            // Yaw (rad) - Attitude of the Aircraft

    //Send simulation packet
    _sock.sendto(&pkt, sizeof(pkt), planck_simu_ip, planck_send_port);

    //Check if there is a ack coming back:
    simu_planck_t pkt_recv;
    if(!_planck_lock){
      if(_sock.recv(&pkt_recv, sizeof(pkt_recv), 0) == sizeof(pkt_recv)){

        //If there is a ack: Enable the time locking mechanism
        hal.console->printf("Planck: Start time lock\n");
        printf("Planck: Start time lock\n");
        _planck_lock = true;
      }
    }

    //If Planck is out of sync: Still listen with low timeout
    else if(_planck_out_sync){

      //Receive planck ctrl ack: Enable the in sync mode
      if(_sock.recv(&pkt_recv, sizeof(pkt_recv), 10) == sizeof(pkt_recv)){
        printf("Planck: Out of Sync but response %i \n", _count_timout);
        _count_timout = 0;
        _planck_out_sync = false;

      }
      else{
        _count_timout +=1;
        printf("Planck: Sync no response count: %i \n", _count_timout);
      }

      //If no answer of a long time: Disable Planck time lock
      if(_count_timout >= 200){
        hal.console->printf("Planck: Stop time lock\n");
        printf("Planck: Stop time lock\n");
        _planck_out_sync = false;
        _planck_lock = false;
      }

    }

    //In sync mode:
    else {

      //Receive ack msg with planck_ctrl clock time until Ardupilot time and Planck_ctrl time matches
      bool planck_in_sync = false;
      while(!planck_in_sync && _planck_lock && !_planck_out_sync){

        //Receive planck ctrl ack
        if(_sock.recv(&pkt_recv, sizeof(pkt_recv), 200) == sizeof(pkt_recv)){
          //If not sync: wait for an other ack msg
          if(pkt_recv.time_simu_us != pkt.time_simu_us){
            printf("Planck: Out of sync %9.4f ms ", ((double)pkt.time_simu_us - pkt_recv.time_simu_us)/1e3);
            printf("AP: %u us ", (unsigned) pkt.time_simu_us);
            printf("PL: %u us \n", (unsigned) pkt_recv.time_simu_us);
          }
          //In sync: move forward with the simulation
          else{
            //printf("Planck: In of sync %9.9f ms \n", ((double)pkt.time_simu_us - pkt_recv.time_simu_us)/1e3);
            planck_in_sync = true;
          }
        }

        //If no answer from Planck for timeout (200ms), move to planck out of sync mode
        else{
          _count_timout = 0;
          _planck_out_sync = true;
          printf("Planck: Sync no response \n");
        }
      }
    }
}


