/* Functions to calculate the Dynamics on an object*/

#include "lander.h"
#include "lander_graphics.h"
#include <random>
#include "Orbiter class.h"
#include "Dynamics.h"

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
  static bool calculated_kh;
  if (simulation_time == 0) calculated_kh = false;
  mars_lander.numerical_dynamics();
  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (mars_lander.stabilized_attitude) mars_lander.attitude_stabilization();

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (mars_lander.autopilot_enabled)
  {
    if (mars_lander.Kh == 0) mars_lander.Kh = 0.018;
    mars_lander.stabilized_attitude = 1;
    mars_lander.autopilot();
  }
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

double kh_tuner(const lander mars_lander)
{
  double timer = 0;
  double Kh_upper = 0.02;
  double Kh_lower = 0.012;
  double Kh_mid = (Kh_upper + Kh_lower) / 2;
  lander virt_lander = mars_lander;
  while(true)
  {
    virt_lander.numerical_dynamics();
    virt_lander.attitude_stabilization();
    virt_lander.autopilot(Kh_mid);
    timer += delta_t;
    if (virt_lander.get_altitude() <= 0)
    {
      if ((fabs(virt_lander.get_climb_speed()) > 1.0 || fabs(virt_lander.get_ground_speed()>1.0)) && virt_lander.fuel <= 0.0008) Kh_lower = Kh_mid;
      else Kh_upper = Kh_mid;
      if (fabs((Kh_upper - Kh_lower)/Kh_upper) < 0.0015) break;

      //reset loop
      timer = 0;
      virt_lander = mars_lander;
      Kh_mid = (Kh_upper + Kh_lower) / 2;
    }

    if (timer > 1000)
    {
      std::cout << "Tuner timed out\n";
      return 0.0; //add time out
    }
  }
  std::cout << "Tuned Kh value: " << Kh_upper << "\n";
  return Kh_upper; //to ensure it does actually land
}
