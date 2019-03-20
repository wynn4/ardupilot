#include "Copter.h"

bool Copter::ModePlanckWingman::init(bool ignore_checks){

    //If we are already landed this makes no sense
    if(copter.ap.land_complete)
      return false;
      
    //Otherwise, run tracking
    if(copter.mode_plancktracking.init(ignore_checks)) {

      //And command a zero-rate, telling planck to maintain current relative position 
      copter.planck_interface.request_move_target(Vector3f(0,0,0),0,true);

      //Make sure we don't immediately send a different command
      _next_req_send_t_ms = millis() + _send_rate_ms;
      return true;
    }

    return false;
}

void Copter::ModePlanckWingman::run() {

  //Update the command if the user is providing input from the sticks
  if(millis() > _next_req_send_t_ms) {
    // throttle failsafe check
    if( !copter.failsafe.radio ) {

      //Get position/yaw offsets from user as necessary
      float x_rate = channel_roll->norm_input_dz() * copter.wp_nav->get_speed_xy()/100.;
      float y_rate = channel_pitch->norm_input_dz() * copter.wp_nav->get_speed_xy()/100.;
      
      //Turn x/y rates into N/E rates
      float N_rate = (x_rate*copter.ahrs.cos_yaw() - y_rate*copter.ahrs.sin_yaw());
      float E_rate = (x_rate*copter.ahrs.sin_yaw() + y_rate*copter.ahrs.cos_yaw());
      
      float yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
      float z_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
      
      Vector3f rate_NED(N_rate, E_rate, z_rate); //NED
      
      //Send a command request if any of our rates are non-zero
      if(N_rate != 0 || E_rate != 0 || yaw_rate != 0 || z_rate !=0) {
        copter.planck_interface.request_move_target(rate_NED, yaw_rate, true);
      }
    }

    _next_req_send_t_ms = millis() + _send_rate_ms;
  }
  
  copter.mode_plancktracking.run();
}
