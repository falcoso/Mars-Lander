/* Functions to calculate the Dynamics on an object*/
#ifndef DYNAMICS
#define DYNAMICS
#include "lander.h"
#include "lander_graphics.h"
#include <random>
#include "Orbiter class.h"
#include "Dynamics.h"

extern lander mars_lander;
double wind(const lander &mars_lander)
{
  constexpr double average_speed = 10; //(m/s)
  std::normal_distribution<double> distribution(average_speed, 1.0);
  std::default_random_engine generator((int)mars_lander.timer);
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
  vector3d rotation = planetary_rotation + wind(*this)*planetary_rotation.norm();
  return -0.5*atmospheric_density(position)*DRAG_COEF_CHUTE*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE*(velocity - rotation).abs()*(velocity - rotation);
}

vector3d lander::lander_drag(void)
{
  vector3d rotation = planetary_rotation + wind(*this)*planetary_rotation.norm();
  return -0.5*front_facing_area*atmospheric_density(position)*(DRAG_COEF_LANDER)*(velocity - rotation).abs()*(velocity - rotation);
}

//calculates gravity on the lander returning a Force vector
vector3d orbiter::gravity() { return -(GRAVITY*MARS_MASS*mass / position.abs2())*position.norm(); }

double kh_tuner(const lander &mars_lander, const bool mode)
{
  double timer = 0;
  lander virt_lander = mars_lander;
  virt_lander.set_virt_obj(true);
  double Kh_upper;
  double Kh_lower;
  if (mode)
  {
    Kh_upper = 0.2;
    Kh_lower = 0.01;
  }
  else
  {
    Kh_upper = 0.05;
    Kh_lower = 0.008;
  }
  virt_lander.Kh = (Kh_upper + Kh_lower) / 2;
  while(true)
  {
    virt_lander.numerical_dynamics();
    virt_lander.attitude_stabilization();
    virt_lander.autopilot();
    virt_lander.timer += delta_t;
    if (virt_lander.get_altitude() <= 0.5)
    {
      if (mode)
      {
        if (fabs(virt_lander.get_climb_speed()) > 1.0 || fabs(virt_lander.get_ground_speed()) > 1.0)
        {
          if(virt_lander.fuel <= 0) Kh_lower = virt_lander.Kh; //check that incase the mid value has skipped the 'landable range'
          else                      Kh_upper = virt_lander.Kh;
        }
        else Kh_lower = virt_lander.Kh;
      }
      else
      {
        if ((fabs(virt_lander.get_climb_speed()) > 1.0 || fabs(virt_lander.get_ground_speed() > 1.0)) && virt_lander.fuel <= 0.0008) Kh_lower = virt_lander.Kh;
        else Kh_upper = virt_lander.Kh;
      }
      
      if ((Kh_upper - Kh_lower) / Kh_upper < 0.01) break;
      //reset loop
      virt_lander = mars_lander;
      virt_lander.set_virt_obj(true);
      virt_lander.Kh = (Kh_upper + Kh_lower) / 2;
    }

    if (virt_lander.timer - simulation_time > 1500)
    {
      std::cout << "Tuner timed out\n";
      return 0.0; //add time out
    }
  }
  //to ensure it does actually land return the value within the safe bounds
  if (mode)
  {
    std::cout << "Tuned Kh Value: " << Kh_lower << "\n";
    return Kh_lower;
  }
  else
  {
    std::cout << "Tuned Kh Value: " << Kh_upper << "\n";
    return Kh_upper; //to ensure it does actually land
  }
}
#endif