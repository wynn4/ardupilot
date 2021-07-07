#include "Copter.h"

bool ModePlanckLand::init(bool ignore_checks){

    //If we are already landed this makes no sense
    if(copter.ap.land_complete)
      return false;
    gcs().send_text(MAV_SEVERITY_INFO, "EnPL: symode from %d to %d", (unsigned)_stored_yaw_mode,(unsigned)auto_yaw.mode());

    _stored_yaw_mode = auto_yaw.mode();

    if(!is_equal(copter.pos_control->get_pos_z_p().kP().get(),_kpz_nom)){
      _kpz_nom = copter.pos_control->get_pos_z_p().kP().get();
    }

    if(!copter.planck_interface.ready_for_land()) {
      return false;
    }

    ModeGuided::set_angle(Quaternion(),0,true,0);
    if(ModeGuidedNoGPS::init(ignore_checks)) {
        float land_velocity = abs((copter.g.land_speed > 0 ?
            copter.g.land_speed : copter.pos_control->get_max_speed_down()))/100.;
      copter.planck_interface.request_land(land_velocity);
      copter.pos_control->get_pos_z_p().kP(g.planck_land_kp_z);
      return true;
    }
    return false;
}

// perform cleanup required when leaving planck land mode
void ModePlanckLand::exit()
{
    copter.pos_control->get_pos_z_p().kP(_kpz_nom);
    gcs().send_text(MAV_SEVERITY_INFO, "EPL: ymode to %d from %d", (unsigned)_stored_yaw_mode,(unsigned)auto_yaw.mode());

    auto_yaw.set_mode(_stored_yaw_mode);
}

void ModePlanckLand::run(){
    copter.mode_plancktracking.run();
}
