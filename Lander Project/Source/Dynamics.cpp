/* Functions to calculate the Dynamics on an object*/

#include "lander.h"
#include "lander_graphics.h"
#include <random>
#include "Orbiter class.h"

extern lander mars_lander;
double wind()
{
  constexpr double average_speed = -5; //(m/s)
  std::normal_distribution<double> distribution(average_speed, 100.0);
  std::default_random_engine generator;
  return wind_enabled*distribution(generator);

}

void dynamics_wrapper()
{
  mars_lander.numerical_dynamics();
  mars_lander.autopilot();
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

double kh_tuner(const lander &mars_lander)
{
  double timer = 0;
  double Kh_upper = 0;
  double Kh_lower = 1;
  double Kh_mid;
  lander virt_lander = mars_lander;
  virt_lander.stabilized_attitude = true;
  virt_lander.autopilot_enabled = false;
  while(true)
  {
    Kh_mid = (Kh_lower + Kh_upper) / 2;
    virt_lander.numerical_dynamics();
    virt_lander.autopilot(Kh_mid);
    timer += delta_t;
    if (virt_lander.get_altitude() <= 0)
    {
      if (fabs(virt_lander.get_ground_speed()) > 1.0 || fabs(virt_lander.get_climb_speed()) > 1.0) Kh_lower = Kh_mid;
      else                                                                                       Kh_upper = Kh_mid;
    }

    if (fabs(Kh_upper - Kh_lower) < 0.001) break;
    if (timer > 1000) return 0.0; //add time out
  }
  return Kh_upper; //to ensure it does actually land
}
