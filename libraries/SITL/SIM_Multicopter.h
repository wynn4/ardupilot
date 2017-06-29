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

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_Motor.h"
#include "SIM_Frame.h"

#include <AP_HAL/utility/Socket.h>


namespace SITL {

/*
  a multicopter simulator
 */
class MultiCopter : public Aircraft {
public:
    MultiCopter(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new MultiCopter(home_str, frame_str);
    }

    void update_planck();


protected:
    // calculate rotational and linear accelerations
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
    Frame *frame;


private:
    /*
     * Simu Planck
     */
    struct simu_planck_t {
      uint64_t time_simu_us;
      double latitude;
      double longitude;
      double altitude;
      double pos_n;
      double pos_e;
      double pos_d;
      double roll;
      double pitch;
      double yaw;
    };
//    simu_planck_t _simu_planck;

    SocketAPM _sock;
    bool _planck_lock = false;

};

}
