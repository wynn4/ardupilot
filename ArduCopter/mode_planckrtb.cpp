#include "Copter.h"

bool Copter::ModePlanckRTB::init(bool ignore_checks){

    //If we are already landed this makes no sense
    if(copter.ap.land_complete)
      return false;
      
    //If we're ready to land, jump right to it
    if(copter.mode_planckland.init(ignore_checks)) {
      return true;
    }
    
    //Otherwise, run tracking
    if(copter.mode_plancktracking.init(ignore_checks)) {
      return true;
    }

    return false;
}

void Copter::ModePlanckRTB::run(){
    copter.mode_plancktracking.run();
}
