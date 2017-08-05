#ifndef ORBITER
#define ORBITER

#include "lander.h"
#include "Dynamics.h"

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
  vector3d get_velocity();
  vector3d get_planetary_rotation();
  vector3d get_relative_velocity();

  void set_velocity(vector3d input_vel);
  void set_position(vector3d input_pos);
  void set_altitude(double input_alt);

protected:
  virtual void update_members();
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
  bool autopilot_enabled;
  bool landed;
  parachute_status_t parachute_status;
  autopilot_t autopilot_status;

  lander();
  virtual void numerical_dynamics();
  void attitude_stabilization();
  vector3d lander_drag();
  vector3d parachute_drag();
  vector3d thrust_wrt_world();

  void set_orientation(vector3d input_orientation);
  vector3d get_orientation();


protected:
  virtual void update_members();
  void autopilot();
  double front_facing_area;
  double ground_speed;
  double climb_speed;
  vector3d orientation;
};
#endif


