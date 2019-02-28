#pragma once

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/Location.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AP_GPS/AP_GPS.h>

//Defines the interface to Planck's control software

class AC_Planck {
  
public:
  AC_Planck(void) {}
  
  ~AC_Planck(void) {}
  
  void handle_planck_mavlink_msg(const mavlink_channel_t &chan, const mavlink_message_t *mav_msg,
    AP_AHRS &ahrs);
    
  void send_stateinfo(const mavlink_channel_t &chan,
    uint8_t control_mode,
    bool armed,
    bool in_flight,
    bool failsafe,
    AP_AHRS_NavEKF &ahrs,
    AP_InertialNav &inertial_nav,
    Location &current_loc,
    AP_GPS &gps);
      
  void request_takeoff(const float alt);
  
  void request_rtb(const float alt, const float rate_up, const float rate_down);
  
  void request_land(const float descent_rate);
  
  void stop_commanding(void);
  
  bool ready_for_takeoff(void) { return _is_status_ok() && _status.takeoff_ready; };
  
  bool ready_for_land(void) { return _is_status_ok() && _status.land_ready; };

  bool get_accel_cmd(float &accel_n,
    float &accel_e,
    float &yaw_cd,
    float &vz_cms,
    bool &is_yaw_rate);

  bool get_attitude_cmd(float &roll_cd,
    float &pitch_cd,
    float &yaw_cd,
    bool &is_yaw_rate);
      
  bool get_attitude_z_rate_cmd(float &roll_cd,
    float &pitch_cd,
    float &yaw_cd,
    float &vz_cms,
    bool &is_yaw_rate);
    
  bool get_velocity_cmd_cms(Vector3f &vel_cmd, float &yaw_cmd_cd);
  
  bool get_position_cmd(Location &loc_cmd);
  
  bool is_sending_accel_cmds() { return (_last_cmd_type == ACCEL); };

  bool is_sending_attitude_cmds() { return (_last_cmd_type == ATTITUDE); };
  
  bool is_sending_velocity_cmds() { return (_last_cmd_type == VELOCITY); };
  
  bool is_sending_position_cmds() { return (_last_cmd_type == POSITION); };
  
  bool is_takeoff_complete() { return _status.takeoff_complete; };
          
private:
          
  struct
  {
    float accel_n = 0;
    float accel_e = 0;
    float yaw_cd = 0;
    float yaw_rate_cds = 0;
    bool use_yaw_rate = true;
    float vz_cms = 0;
    uint32_t timestamp_ms = 0;
  }_accel_cmd;
  
  struct
  {
    float roll_cd = 0;
    float pitch_cd = 0;
    float yaw_cd = 0;
    float yaw_rate_cds = 0;
    bool use_yaw_rate = true;
    float vz_cms = 0;
    uint32_t timestamp_ms = 0;
  }_attitude_cmd;
  
  struct
  {
    bool takeoff_ready = false;
    bool land_ready = false;
    bool commbox_ok = false;
    bool commbox_gps_ok = false;
    bool tracking_tag = false;
    bool tracking_commbox_gps = false;
    bool takeoff_complete = false;
    uint32_t timestamp_ms = 0;
  }_status;
  
  enum{
    ACCEL=0,
    ATTITUDE,
    VELOCITY,
    POSITION
  }_last_cmd_type = ATTITUDE;
  
  Vector3f _velocity_cmd_cms;
  float _velocity_yaw_cmd_cd = 0;
  uint32_t _velocity_timestamp_ms = 0;
  
  Location _position_cmd;
  bool _position_cmd_stale = true;
  
  mavlink_channel_t _chan = MAVLINK_COMM_1;
  
  const uint32_t _cmd_timeout_att_ms = 100;
  const uint32_t _cmd_timeout_accel_ms = 100;
  const uint32_t _cmd_timeout_vel_ms = 500;
  
  bool _is_status_ok(void) { return ((AP_HAL::millis() - _status.timestamp_ms) < 500); }
};