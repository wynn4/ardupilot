#include "Copter.h"

bool Copter::ModePlanckTracking::init(bool ignore_checks){
    return Copter::ModeGuided::init(ignore_checks);
}

void Copter::ModePlanckTracking::run(){
    //Set guided mode attitude/velocity commands
    if(copter.planck_interface.is_sending_accel_cmds()) {
        float accel_n, accel_e, yaw_cd, vz_cms;
        bool is_yaw_rate;
        
        bool good_cmd = copter.planck_interface.get_accel_cmd(
            accel_n, accel_e, yaw_cd, vz_cms, is_yaw_rate
        );
        
        if(!good_cmd) {
            accel_n = accel_e = yaw_cd = vz_cms = 0;
            is_yaw_rate = true;
        }
        
        //Turn accel into lean angles
        float roll_cd, pitch_cd;
        copter.pos_control->accel_to_lean_angles(
          accel_n * 100.,
          accel_e * 100.,
          roll_cd,
          pitch_cd);

        //Convert this to quaternions, yaw rates
        Quaternion q;
        q.from_euler(ToRad(roll_cd/100.), ToRad(pitch_cd/100.), ToRad(yaw_cd/100.));
        float yaw_rate_rads = ToRad(yaw_cd / 100.);
    
        //Update the GUIDED mode controller
        Copter::ModeGuided::set_angle(q,vz_cms,is_yaw_rate,yaw_rate_rads);
    }
    else if(copter.planck_interface.is_sending_attitude_cmds()) {
        float roll_cd, pitch_cd, yaw_cd, vz_cms;
        bool is_yaw_rate;
        
        bool good_cmd = copter.planck_interface.get_attitude_z_rate_cmd(
            roll_cd, pitch_cd, yaw_cd, vz_cms, is_yaw_rate
        );
        
        if(!good_cmd) {
            roll_cd = pitch_cd = yaw_cd = vz_cms = 0;
            is_yaw_rate = true;
        }

        //Convert this to quaternions, yaw rates
        Quaternion q;
        q.from_euler(ToRad(roll_cd/100.), ToRad(pitch_cd/100.), ToRad(yaw_cd/100.));
        float yaw_rate_rads = ToRad(yaw_cd / 100.);
    
        //Update the GUIDED mode controller
        Copter::ModeGuided::set_angle(q,vz_cms,is_yaw_rate,yaw_rate_rads);
    }
    else if(copter.planck_interface.is_sending_velocity_cmds()) {
        Vector3f vel_cmd;
        float yaw_cmd_cd;
        bool yaw_rate = true;
        
        bool good_cmd = copter.planck_interface.get_velocity_cmd_cms(vel_cmd, yaw_cmd_cd);
        
        if(!good_cmd) {
            vel_cmd.x = vel_cmd.y = vel_cmd.z = 0;
            yaw_cmd_cd = 0;
            yaw_rate = false;
        }
        
        Copter::ModeGuided::set_velocity(vel_cmd, yaw_rate, yaw_cmd_cd);
    }
    else if(copter.planck_interface.is_sending_position_cmds()){
        Location loc_cmd;
        copter.planck_interface.get_position_cmd(loc_cmd);
        Copter::ModeGuided::set_destination(loc_cmd);
    }
    
    //Run the guided mode controller
    Copter::ModeGuided::run();
}

bool Copter::ModePlanckTracking::do_user_takeoff_start(float final_alt_above_home)
{
    // Check if planck is ready
    if(!copter.planck_interface.ready_for_takeoff())
      return false;

    // Tell planck to start commanding
    copter.planck_interface.request_takeoff(final_alt_above_home/100.);

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    return true;
}
