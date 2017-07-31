/* Functions to calculate the Dynamics on an object*/

#include "lander.h"
#include <random>

void planetary_rotation_update(void)
{
  planetary_rotation = pow(pow(position.x, 2) + pow(position.y, 2), 0.5)*(2 * M_PI / MARS_DAY);
}

double wind()
{
  constexpr double average_speed = -5; //(m/s)
  std::normal_distribution<double> distribution(average_speed, 100.0);
  std::default_random_engine generator;
  return wind_enabled*distribution(generator);

}

vector3d atmosphere_rotation()
{
  vector3d tangential = { -position.norm().y, position.norm().x, 0 };
  return planetary_rotation*tangential*0;
}

vector3d atmosphere_rotation(const vector3d &position)
{
  vector3d tangential = { -position.norm().y, position.norm().x, 0 };
  return tangential*pow(pow(position.x, 2) + pow(position.y, 2), 0.5)*(2 * M_PI / MARS_DAY);
}

vector3d fluid_rotation(void)
{
  double fluid_motion = ((MARS_RADIUS * 2 * M_PI) / MARS_DAY) - ((MARS_RADIUS * 2 * M_PI) / (EXOSPHERE*MARS_DAY))*(position.abs() - MARS_RADIUS);
  vector3d tangential = { -position.norm().y, position.norm().x, 0 };
  return tangential*fluid_motion;
}

//___________________________________________________________________________
//calculates the drag on the lander returning a Force vector
vector3d lander_drag(void)
{
  constexpr double front_facing_area = M_PI*LANDER_SIZE*LANDER_SIZE;
  vector3d rotation = atmosphere_rotation() + wind()*atmosphere_rotation().norm();
  return -0.5*front_facing_area*atmospheric_density(position)*(DRAG_COEF_LANDER)*(velocity-rotation).abs()*(velocity-rotation);
}

vector3d lander_drag(const vector3d &position, const vector3d &velocity)
{
  constexpr double front_facing_area = M_PI*LANDER_SIZE*LANDER_SIZE;
  vector3d rotation = atmosphere_rotation(position) + wind()*atmosphere_rotation(position).norm();
  return -0.5*front_facing_area*atmospheric_density(position)*(DRAG_COEF_LANDER)*(velocity - rotation).abs()*(velocity - rotation);
}
//___________________________________________________________________________

vector3d parachute_drag(void)
{
  vector3d rotation = atmosphere_rotation() + wind()*atmosphere_rotation().norm();
  return -0.5*atmospheric_density(position)*DRAG_COEF_CHUTE*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE*(velocity - rotation).abs()*(velocity - rotation);
}

vector3d parachute_drag(const vector3d &position, const vector3d &velocity)
{
  vector3d rotation = atmosphere_rotation(position) + wind()*atmosphere_rotation(position).norm();
  return -0.5*atmospheric_density(position)*DRAG_COEF_CHUTE*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE*(velocity - rotation).abs()*(velocity - rotation);
}

//_____________________________________________________________________________
//calculates gravity on the lander returning a Force vector
vector3d gravity(const double &lander_mass)
{
  vector3d F = -(GRAVITY*MARS_MASS*lander_mass / position.abs2())*position.norm();
  return F;
}

vector3d gravity(const double &lander_mass, const vector3d &position)
{
  vector3d F = -(GRAVITY*MARS_MASS*lander_mass / position.abs2())*position.norm();
  return F;
}


bool open_chute_query(void)
{
  vector3d virt_velocity   = velocity;
  vector3d virt_position   = position;
  double old_virt_velocity = velocity.abs();
  double virt_dt           = 0.1;
  double virt_time         = virt_dt;
  double virt_mass;
  vector3d virt_drag;
  vector3d virt_acceleration;
  while(TRUE)
  {
    virt_drag = lander_drag(virt_position, virt_velocity) + parachute_drag(virt_position, virt_velocity);
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

double kh_set(double kh_upper, double kh_lower)
{
  vector3d virt_velocity = velocity;
  vector3d virt_position = position;
  double old_virt_velocity = velocity.abs();
  double virt_dt = 0.1;
  double virt_time = virt_dt;
  double virt_mass, ver, delta, altitude, error, Pout, kh_mid;
  vector3d virt_drag;
  vector3d virt_acceleration;
  constexpr double ideal_ver = 0.5;
  constexpr double Kp = 1;
  while (TRUE)
  {
    virt_drag = lander_drag(virt_position, virt_velocity);
    virt_mass = UNLOADED_LANDER_MASS + fuel*FUEL_CAPACITY*FUEL_DENSITY; //note this assumes effectively constant mass in virtual calulation
    virt_acceleration = (gravity(virt_mass) + thrust_wrt_world() + virt_drag) / virt_mass;
    virt_position += virt_dt*virt_velocity;
    virt_velocity += virt_dt*virt_acceleration;

    ver = virt_velocity*virt_position.norm();
    altitude = virt_position.abs() - MARS_RADIUS;
    delta = gravity(virt_mass).abs() / MAX_THRUST;
    
    kh_mid = (kh_lower + kh_upper) / 2;
    error = -(ideal_ver + kh_mid*altitude + ver);
    Pout = Kp*error;

    //Proportional gain control
    if (Pout <= -delta)         throttle = 0;
    else if (Pout >= 1 - delta) throttle = 1;
    else                        throttle = delta + Pout;

    if (altitude <= 0)
    {
      if (velocity.abs() > 1) kh_lower = kh_mid;
      else                    kh_upper = kh_mid;
      if (kh_lower - kh_upper < 0.01) break;
    }
    virt_time += virt_dt;
  }
  return kh_mid;
}
