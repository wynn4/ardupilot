#include "Copter.h"

bool ModePlanckRTB::init(bool ignore_checks){

    //If we are already landed this makes no sense
    if(copter.ap.land_complete)
      return false;

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
    if(_is_landing && copter.planck_interface.was_last_request_rejected() && (copter.planck_interface.get_last_cmd_req_id() == PLANCK_CMD_REQ_LAND)) {
        _is_landing = false;
    }

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
