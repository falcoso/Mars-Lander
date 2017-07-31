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