#include <AC_Planck/AC_Planck.h>
#include <AP_HAL/AP_HAL.h>
#include "../ArduCopter/defines.h"

 void AC_Planck::handle_planck_mavlink_msg(const mavlink_channel_t &chan, const mavlink_message_t *mav_msg,
    AP_AHRS &ahrs)
  {

   switch(mav_msg->msgid)
  {

     case MAVLINK_MSG_ID_PLANCK_STATUS:
      _chan = chan; //Set the channel based on the incoming status message
      _status.timestamp_ms = AP_HAL::millis();
      _status.takeoff_ready = (bool)mavlink_msg_planck_status_get_takeoff_ready(mav_msg);
      _status.land_ready = (bool)mavlink_msg_planck_status_get_land_ready(mav_msg);
      _status.commbox_ok = (bool)(mavlink_msg_planck_status_get_failsafe(mav_msg) & 0x01);
      _status.commbox_gps_ok = (bool)(mavlink_msg_planck_status_get_failsafe(mav_msg) & 0x02);
      _status.tracking_tag = (bool)(mavlink_msg_planck_status_get_status(mav_msg) & 0x01);
      _status.tracking_commbox_gps = (bool)(mavlink_msg_planck_status_get_status(mav_msg) & 0x02);
      _status.takeoff_complete = (bool)mavlink_msg_planck_status_get_takeoff_complete(mav_msg);
      break;

     case MAVLINK_MSG_ID_PLANCK_ATT_CMD_MSG:
      _attitude_cmd.timestamp_ms = AP_HAL::millis();
      _attitude_cmd.roll_cd = ToDeg(mavlink_msg_planck_att_cmd_msg_get_roll(mav_msg)) * 100.;
      _attitude_cmd.pitch_cd = ToDeg(mavlink_msg_planck_att_cmd_msg_get_pitch(mav_msg)) * 100.;
      _attitude_cmd.yaw_cd = ToDeg(mavlink_msg_planck_att_cmd_msg_get_yaw(mav_msg)) * 100.;
      _attitude_cmd.yaw_rate_cds = ToDeg(mavlink_msg_planck_att_cmd_msg_get_yaw_rate(mav_msg)) * 100.;
      _attitude_cmd.use_yaw_rate = (bool)mavlink_msg_planck_att_cmd_msg_get_use_yaw_rate(mav_msg);
      _attitude_cmd.vz_cms = mavlink_msg_planck_att_cmd_msg_get_vz(mav_msg) * 100.;
      _last_cmd_type = ATTITUDE;
      break;

     case MAVLINK_MSG_ID_PLANCK_VEL_CMD_MSG:
      _velocity_timestamp_ms = AP_HAL::millis();
      _velocity_cmd_cms.x = mavlink_msg_planck_vel_cmd_msg_get_vx(mav_msg) * 100.;
      _velocity_cmd_cms.y = mavlink_msg_planck_vel_cmd_msg_get_vy(mav_msg) * 100.;
      _velocity_cmd_cms.z = -mavlink_msg_planck_vel_cmd_msg_get_vz(mav_msg) * 100.; //Ardu uses positive up
      _velocity_yaw_cmd_cd = ToDeg(mavlink_msg_planck_vel_cmd_msg_get_yaw(mav_msg)) * 100.;
      _last_cmd_type = VELOCITY;
      break;

     case MAVLINK_MSG_ID_PLANCK_POS_CMD_MSG:
    {
      _position_cmd.lat = mavlink_msg_planck_pos_cmd_msg_get_latitude(mav_msg);
      _position_cmd.lng = mavlink_msg_planck_pos_cmd_msg_get_longitude(mav_msg);
      _position_cmd.alt = mavlink_msg_planck_pos_cmd_msg_get_altitude(mav_msg)/10; //mm->cm

       switch(mavlink_msg_planck_pos_cmd_msg_get_frame(mav_msg)) {
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
          _position_cmd.flags.relative_alt = true;
          _position_cmd.flags.terrain_alt = false;
          break;
        case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
          _position_cmd.flags.relative_alt = true;
          _position_cmd.flags.terrain_alt = true;
          break;
        case MAV_FRAME_GLOBAL_INT:
        default:
          // Copter does not support navigation to absolute altitudes. This convert the WGS84 altitude
          // to a home-relative altitude before passing it to the navigation controller
          _position_cmd.alt -= ahrs.get_home().alt;
          _position_cmd.flags.relative_alt = true;
          _position_cmd.flags.terrain_alt = false;
          break;
      }

       _position_cmd_stale = false;
      _last_cmd_type = POSITION;
      break;
    }

     default:
      break;
  }
}

 void AC_Planck::send_stateinfo(const mavlink_channel_t &chan,
    uint8_t control_mode,
    bool armed,
    bool in_flight,
    bool failsafe,
    AP_AHRS_NavEKF &ahrs,
    AP_InertialNav &inertial_nav,
    Location &current_loc,
    AP_GPS &gps)
  {

     if(ahrs.get_NavEKF2().activeCores() > 0)
    {

       Vector3f accel;
      ahrs.get_NavEKF2().getAccelNED(accel);

       uint8_t status = 0x00;
      if(armed)
        status |= 0x01;

       if(in_flight)
        status |= 0x02;

       if(failsafe)
        status |= 0x04;

       const Vector3f &vel = inertial_nav.get_velocity()/100;

       mavlink_msg_planck_stateinfo_send(
          chan,
          PLANCK_SYS_ID,
          PLANCK_CTRL_COMP_ID,
          AP_HAL::micros64(),
          gps.time_epoch_usec(),
          control_mode,
          status,
          ahrs.roll,
          ahrs.pitch,
          ahrs.yaw,
          accel.x,
          accel.y,
          accel.z,
          current_loc.lat,                // in 1E7 degrees
          current_loc.lng,                // in 1E7 degrees
          (ahrs.get_home().alt + current_loc.alt) * 10UL,      // millimeters above sea level
          current_loc.alt * 10,           // millimeters above ground
          vel.x,                          // X speed m/s (+ve North)
          vel.y,                          // Y speed m/s (+ve East)
          vel.z);                         // Z speed m/s (+ve up)
    }
}

 void AC_Planck::request_takeoff(const float alt)
{
  //Send a takeoff command message to planck
  mavlink_msg_planck_start_takeoff_send(
      _chan,
      PLANCK_SYS_ID,         //uint8_t target_system
      PLANCK_CTRL_COMP_ID,   //uint8_t target_component,
      alt);                  //float altitude
}

 void AC_Planck::request_rtb(const float alt, const float rate_up, const float rate_down)
{
  //Send an RTL command message to planck
  mavlink_msg_planck_start_return_to_landing_platform_send(
      _chan,
      PLANCK_SYS_ID,         //uint8_t target_system
      PLANCK_CTRL_COMP_ID,   //uint8_t target_component,
      alt,                   //float altitude
      rate_up,               //float rate_up
      rate_down);            //float rate_down
}

 void AC_Planck::request_land(const float descent_rate)
{
  //Send a land command message to planck
  mavlink_msg_planck_start_land_send(
      _chan,
      PLANCK_SYS_ID,         //uint8_t target_system
      PLANCK_CTRL_COMP_ID,   //uint8_t target_component,
      descent_rate);         //float descent_rate
}

 void AC_Planck::stop_commanding(void)
{
    mavlink_msg_planck_stop_controlling_send(
        _chan,
        PLANCK_SYS_ID,         //uint8_t target_system
        PLANCK_CTRL_COMP_ID,   //uint8_t target_component,
        1);                    //uint8_t stop commanding,
}

 //Returns true if the command is good, false if its too old
bool AC_Planck::get_attitude_cmd(float &roll_cd,
  float &pitch_cd,
  float &yaw_cd,
  bool &is_yaw_rate)
{
    if((AP_HAL::millis() - _attitude_cmd.timestamp_ms) >= _cmd_timeout_att_ms)
        return false;

     roll_cd = _attitude_cmd.roll_cd;
    pitch_cd = _attitude_cmd.pitch_cd;
    yaw_cd = _attitude_cmd.yaw_cd;
    is_yaw_rate = _attitude_cmd.use_yaw_rate;

     return true;
}

 //Returns true if the command is good, false if its too old
bool AC_Planck::get_attitude_z_rate_cmd(float &roll_cd,
  float &pitch_cd,
  float &yaw_cd,
  float &vz_cms,
  bool &is_yaw_rate)
{
    if((AP_HAL::millis() - _attitude_cmd.timestamp_ms) >= _cmd_timeout_att_ms)
        return false;

     get_attitude_cmd(roll_cd, pitch_cd, yaw_cd, is_yaw_rate);

     vz_cms = _attitude_cmd.vz_cms;

     return true;
}

 //Returns true if the command is good, false if its too old
bool AC_Planck::get_velocity_cmd_cms(Vector3f &vel_cmd, float &yaw_cmd_cd)
{
    if((AP_HAL::millis() - _velocity_timestamp_ms) >= _cmd_timeout_vel_ms)
        return false;

     vel_cmd = _velocity_cmd_cms;
    return true;
}

 //Returns true if the command is new, false if its old
bool AC_Planck::get_position_cmd(Location &loc_cmd)
{
    if(_position_cmd_stale)
      return false;

     loc_cmd = _position_cmd;
    _position_cmd_stale = true;

     return true;
}
