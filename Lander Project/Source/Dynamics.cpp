/* Functions to calculate the Dynamics on an object*/

#include "lander.h"
#include "lander_graphics.h"
#include <random>
#include "Orbiter class.h"

double wind()
{
  constexpr double average_speed = -5; //(m/s)
  std::normal_distribution<double> distribution(average_speed, 100.0);
  std::default_random_engine generator;
  return wind_enabled*distribution(generator);

}
//vector3d fluid_rotation(void)
//{
//  double fluid_motion = ((MARS_RADIUS * 2 * M_PI) / MARS_DAY) - ((MARS_RADIUS * 2 * M_PI) / (EXOSPHERE*MARS_DAY))*(position.abs() - MARS_RADIUS);
//  vector3d tangential = { -position.norm().y, position.norm().x, 0 };
//  return tangential*fluid_motion;
//}

//calculates the drag on the lander returning a Force vector
vector3d lander::parachute_drag(void)
{
  vector3d rotation = planetary_rotation + wind()*planetary_rotation.norm();
  return -0.5*atmospheric_density(position)*DRAG_COEF_CHUTE*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE*(velocity - rotation).abs()*(velocity - rotation);
}

vector3d lander::lander_drag(void)
{
  vector3d rotation = planetary_rotation + wind()*planetary_rotation.norm();
  return -0.5*front_facing_area*atmospheric_density(position)*(DRAG_COEF_LANDER)*(velocity - rotation).abs()*(velocity - rotation);
}

//calculates gravity on the lander returning a Force vector
vector3d orbiter::gravity() { return -(GRAVITY*MARS_MASS*mass / position.abs2())*position.norm(); }
