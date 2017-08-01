#ifndef ORBITER
#define ORBITER

#include "lander.h"
#include "Dynamics.h"

class orbiter {
public: 

  double mass; 
  void numerical_dynamics();
  vector3d gravity();

protected:
  double planetary_rotation;
  double radius;
  vector3d velocity;
  vector3d position;
  vector3d acceleration;
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
  vector3d orientation;

  lander(double construction_radius);
  void numerical_dynamics();
  void attitude_stabilization();
  vector3d lander_drag();
  vector3d parachute_drag();
  vector3d thrust_wrt_world();


protected:
  void autopilot();
  double front_facing_area;

};
#endif


