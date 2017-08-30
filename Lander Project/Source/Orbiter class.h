#ifndef ORBITER
#define ORBITER

#include "lander.h"

extern unsigned long throttle_buffer_length, throttle_buffer_pointer;
extern double *throttle_buffer;

class orbiter {
public: 
  orbiter(vector3d input_pos, vector3d input_vel, double input_mass, double input_radius);
  orbiter();
  virtual void numerical_dynamics();
  vector3d gravity();

  double get_altitude();
  double get_mass();
  vector3d get_position();
  vector3d get_old_position();
  vector3d get_velocity();
  vector3d get_planetary_rotation();
  vector3d get_relative_velocity();

  void set_velocity(vector3d input_vel);
  void set_position(vector3d input_pos);
  void set_altitude(double input_alt);
  virtual void update_members();

protected:
  double radius;
  double mass; 
  vector3d velocity;
  vector3d position;
  
  vector3d old_position;
  vector3d acceleration;
  double altitude;
  vector3d planetary_rotation;
  vector3d relative_velocity;
};


class lander : public orbiter
{
public:
  double throttle;
  double fuel;
  double Kh;
  float stabilized_attitude_angle;
  bool autopilot_enabled;
  bool stabilized_attitude;
  bool landed;
  bool safe_to_deploy_parachute;
  parachute_status_t parachute_status;
  autopilot_t autopilot_status;

  lander();
  virtual void numerical_dynamics();
  void attitude_stabilization();
  vector3d lander_drag();
  vector3d parachute_drag();
  vector3d thrust_wrt_world();

  void set_virt_obj(bool input);
  void set_orientation(vector3d input_orientation);
  vector3d get_orientation();
  double get_climb_speed();
  double get_ground_speed();
  virtual void update_members();
  void autopilot(bool reset = false);

protected:
  bool virt_obj;
  double front_facing_area;
  double ground_speed;
  double climb_speed;
  vector3d orientation;
  vector3d polar_velocity;
};
#endif


