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
  parachute_status_t parachute_status;
  autopilot_t autopilot_status;
  vector3d orientation;

  lander(double construction_radius);
  void numerical_dynamics();
  vector3d lander_drag();
  vector3d parachute_drag();


protected:
  void autopilot();
  double front_facing_area;

};
#endif


