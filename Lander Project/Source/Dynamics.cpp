/* Functions to calculate the Dynamics on an object*/

#include "lander.h"

//calculates the drag on the lander returning a Force vector
//PERFORMANCE IMPROVEMENT: graphics also calculates drag so combine two if necessary
vector3d drag(void)
{
  vector3d F;
  constexpr double front_facing_area = M_PI*LANDER_SIZE*LANDER_SIZE;
  switch (parachute_status)
  {
  case DEPLOYED:
    F = -0.5*atmospheric_density(position)*(DRAG_COEF_LANDER*front_facing_area + DRAG_COEF_CHUTE*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE)*velocity.abs()*velocity;
    break;

  default:
    F = -0.5*front_facing_area*atmospheric_density(position)*(DRAG_COEF_LANDER)*velocity.abs()*velocity;
    break;
  }
  return F;
}

//calculates gravity on the lander returning a Force vector
vector3d gravity(const double &lander_mass)
{
  vector3d F = -(GRAVITY*MARS_MASS*lander_mass / position.abs2())*position.norm();
  return F;
}

bool open_chute_query(void)
{
  vector3d virt_velocity = velocity;
  double old_virt_velocity = velocity.abs();
  vector3d virt_position = position;
  vector3d virt_acceleration;
  double virt_dt = 0.1;
  double virt_mass;
  vector3d virt_drag;
  bool conclusion = 1;
  double virt_time = virt_dt;
  while (conclusion)
  {
    virt_drag = -0.5*atmospheric_density(position)*(DRAG_COEF_LANDER*M_PI*LANDER_SIZE*LANDER_SIZE + DRAG_COEF_CHUTE*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE)*velocity.abs()*velocity;
    virt_mass = UNLOADED_LANDER_MASS + fuel*FUEL_CAPACITY*FUEL_DENSITY; //note this assumes effectively constant mass in virtual calulation
    virt_acceleration = (gravity(virt_mass) + thrust_wrt_world() + virt_drag) / virt_mass;
    virt_position += virt_dt*virt_velocity;
    virt_velocity += virt_dt*virt_acceleration;
    if (virt_velocity.abs() >= MAX_PARACHUTE_SPEED || virt_position.abs() <= MARS_RADIUS)
    {
      break;
    }
    else if (virt_velocity.abs() < old_virt_velocity)
    {
      return 1;
    }
    else
    {
      old_virt_velocity = virt_velocity.abs();
    }
    virt_time += virt_dt;
  }
  return 0;
}