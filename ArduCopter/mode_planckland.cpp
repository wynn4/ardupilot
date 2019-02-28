#include "Copter.h"

bool Copter::ModePlanckLand::init(bool ignore_checks){

    if(!copter.planck_interface.ready_for_land())
      return false;

    if(Copter::ModeGuided::init(ignore_checks)) {
        float land_velocity = abs((copter.g.land_speed > 0 ?
            copter.g.land_speed : copter.pos_control->get_speed_down()))/100.;
      copter.planck_interface.request_land(land_velocity);
    }

    return false;
}

void Copter::ModePlanckLand::run(){
    copter.mode_planckland.run();
}
