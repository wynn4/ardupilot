#include "Copter.h"

bool ModePlanckRTB::init(bool ignore_checks){

    //If we are already landed this makes no sense
    if(copter.ap.land_complete)
      return false;

    gcs().send_text(MAV_SEVERITY_INFO, "Entering Planck RTB: storing yaw mode from %d to %d", (unsigned)_stored_yaw_mode,(unsigned)auto_yaw.mode());

    _stored_yaw_mode = auto_yaw.mode();

    //If we're ready to land, jump right to it
    if(copter.mode_planckland.init(ignore_checks)) {
      _is_landing = true;
      return true;
    }

    //Otherwise, run tracking
    if(copter.mode_plancktracking.init(ignore_checks)) {
      _is_landing = false;
      return true;
    }

    return false;
}

void ModePlanckRTB::run(){
    if(!_is_landing)
    {
        //This checks if planck is ready to land and requests a landing
        if(copter.mode_planckland.init(true))
        {
            printf("PlanckRTB run: landing ready\n");
            _is_landing = true;
        }
    }
    copter.mode_plancktracking.run();
}

void ModePlanckRTB::exit()
{
  gcs().send_text(MAV_SEVERITY_INFO, "Exiting RTB: setting yaw mode to %d from %d", (unsigned)_stored_yaw_mode,(unsigned)auto_yaw.mode());

  auto_yaw.set_mode(_stored_yaw_mode);
}
