#include "lander.h"
#include "Dynamics.h"
#include "Orbiter class.h"
#include "lander_graphics.h"

void orbiter::numerical_dynamics()
{
  static vector3d old_position; //do not assign here, as will not reset when new scenario selected
  vector3d new_position;

  if (simulation_time == 0.0) old_position = position - delta_t*velocity;

  switch (intergrator) //switch based on intergration method chosen
  {
  case VERLET:
    new_position = 2 * position - old_position + delta_t*delta_t*acceleration;
    velocity = (1 / delta_t)*(new_position - position);
    //shift along positions
    old_position = position;
    position = new_position;
    break;
  case EULER:
    position += delta_t*velocity;
    velocity += delta_t*acceleration;
    break;
  }
  update_members();
  return;
}

vector3d orbiter::gravity()      { return -(GRAVITY*MARS_MASS*mass / position.abs2())*position.norm(); }
vector3d orbiter::get_position() { return position; }
vector3d orbiter::get_velocity() { return velocity; }
double orbiter::get_altitude()   { return altitude; }
double orbiter::get_mass()       { return mass; }
double orbiter::get_planetary_rotation() { return planetary_rotation; }

void orbiter::set_position(vector3d input_pos) { position = input_pos; }
void orbiter::set_velocity(vector3d input_vel) { velocity = input_vel; }

orbiter::orbiter(vector3d input_pos, vector3d input_vel, double input_mass, double input_radius)
{
  position = input_pos;
  velocity = input_vel;
  mass = input_mass;
  radius = input_radius;
  update_members();
}

orbiter::orbiter()
{
  position = vector3d{ 0,0,0 };
  velocity = vector3d{ 0,0,0 };
  mass = 0;
  radius = 0;
  update_members();
}

void orbiter::update_members()
{
  altitude = position.abs() - MARS_RADIUS;
  planetary_rotation = pow(pow(position.x, 2) + pow(position.y, 2), 0.5)*(2 * M_PI / MARS_DAY);
  acceleration = gravity() / mass;
}

//===========================================================================================================

lander::lander(double input_radius)
{
  fuel = 1;
  throttle = 0;
  autopilot_status = ORBIT_DESCENT;
  parachute_status = NOT_DEPLOYED;
  radius = input_radius;
  front_facing_area = M_PI*LANDER_SIZE*LANDER_SIZE;

  update_members();
  fuel = 1; //as update members will decrement fuel again
  return;
}

void lander::numerical_dynamics()
{
    //declare old and new potision variables for verlet intergrator
    static vector3d old_position; //do not assign here, as will not reset when new scenario selected
    vector3d new_position;
    static double kp_test;

    //so that if the simulation is reset so does the old position
    if (simulation_time == 0.0) old_position = position - delta_t*velocity;

    switch (intergrator) //switch based on intergration method chosen
    {
    case VERLET:
      new_position = 2 * position - old_position + delta_t*delta_t*acceleration;
      velocity = (1 / delta_t)*(new_position - position);
      //shift along positions
      old_position = position;
      position = new_position;
      break;
    case EULER:
      position += delta_t*velocity;
      velocity += delta_t*acceleration;
      break;
    }
    // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
    if (stabilized_attitude) attitude_stabilization();

    // Here we can apply an autopilot to adjust the thrust, parachute and attitude
    if (autopilot_enabled)
    {
      stabilized_attitude = 1;
      //  stabilized_attitude_angle = -(acos(position.norm()*(velocity-atmosphere_rotation()).norm()) + M_PI);
      autopilot();
    }
    update_members();
    return;
}


vector3d lander::parachute_drag(void)
{
  vector3d rotation = atmosphere_rotation() + wind()*atmosphere_rotation().norm();
  return -0.5*atmospheric_density(position)*DRAG_COEF_CHUTE*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE*(velocity - rotation).abs()*(velocity - rotation);
}

vector3d lander::lander_drag(void)
{
  vector3d rotation = atmosphere_rotation() + wind()*atmosphere_rotation().norm();
  return -0.5*front_facing_area*atmospheric_density(position)*(DRAG_COEF_LANDER)*(velocity - rotation).abs()*(velocity - rotation);
}

void lander::autopilot()
{
  constexpr double ideal_ver = 0.5;
  constexpr double Kp = 1;
  double Kh, ver, altitude, delta, error, Pout;

  /*If no parachute is available then Kh = 0.018 will land safely, if parachute is available
  a more efficient configuration is Kh = 0.03 Kp = 1 ideal_ver = 0.5
  ***MOST EFFICIENT Kh WITH PARACHUTE***        ***SOFEST LANDING Kh WITH PARACHUTE***
  Scenario 1  Kh = 0.1525   Fuel Used = 7.9         Kh = 0.0104
  Scenario 3  Kh = 0.04775  Fuel Used = 35          Kh = 0.0147
  Scenario 4  Kh = 0.1525   Fuel Used = 7.9         Kh = 0.01095
  Scenario 5  Kh = 0.05143  Fuel Used = 30.1        Kh = 0.0145

  ***Most EFFICIENT Kh WITHOUT PARACHUTE***
  Scenario 1  Kh = 0.04125  Fuel Used = 33.1        Kh = 0.0137
  Scenario 3  Kh = 0.01812  Fuel Used = 59.1        Kh = 0.01625
  Scenario 5  Kh = 0.01898  Fuel Used = 57.1        Kh = 0.01675
  */
  ver = velocity*position.norm();
  altitude = position.abs() - MARS_RADIUS;
  delta = gravity().abs() / MAX_THRUST; 
  
  Kh = 0.03;

  error = -(ideal_ver + Kh*altitude + ver);
  Pout = Kp*error;

  //Proportional gain control
  if (Pout <= -delta)          throttle = 0;
  else if (Pout >= 1 - delta)  throttle = 1;
  else                         throttle = delta + Pout;
}

vector3d lander::thrust_wrt_world(void)
// Works out thrust vector in the world reference frame, given the lander's orientation
{
  double m[16], k, delayed_throttle, lag = ENGINE_LAG;
  vector3d a, b;
  static double lagged_throttle = 0.0;
  static double last_time_lag_updated = -1.0;

  if (simulation_time < last_time_lag_updated) lagged_throttle = 0.0; // simulation restarted
  if (throttle < 0.0) throttle = 0.0;
  if (throttle > 1.0) throttle = 1.0;
  if (landed || (fuel == 0.0)) throttle = 0.0;

  if (simulation_time != last_time_lag_updated) {

    delayed_throttle = throttle;

    // Lag, with time constant ENGINE_LAG
    k = 0.0;

    last_time_lag_updated = simulation_time;
  }

  if (stabilized_attitude && (abs(stabilized_attitude_angle) < 1E-7)) { // specific solution, avoids rounding errors in the more general calculation below
    b = lagged_throttle*MAX_THRUST*position.norm();
  }
  else {
    a.x = 0.0; a.y = 0.0; a.z = lagged_throttle*MAX_THRUST;
    xyz_euler_to_matrix(orientation, m);
    b.x = m[0] * a.x + m[4] * a.y + m[8] * a.z;
    b.y = m[1] * a.x + m[5] * a.y + m[9] * a.z;
    b.z = m[2] * a.x + m[6] * a.y + m[10] * a.z;
  }
  return b;
}

void lander::attitude_stabilization(void)
// Three-axis stabilization to ensure the lander's base is always pointing downwards 
{
  vector3d up, left, out, axis;
  double m[16];
  //define axis of rotation
  if (autopilot_status == ORBIT_DESCENT && autopilot_enabled)
  {
    up = -(velocity - atmosphere_rotation()).norm();
  }
  else if (abs(stabilized_attitude_angle) < 1E-7) //skip expensive calulcations if aproximately 0
  {
    up = position.norm();
  }
  else
  { //define the axis of rotation
    axis = (position.norm() ^ closeup_coords.right).norm();
    //convert to quaternion 
    quat_t rotation_quat = axis_to_quat(axis, stabilized_attitude_angle);
    //create rotation matrix from quaternion, recycling m[] as it is re-assigned later
    quat_to_matrix(m, rotation_quat);
    //multiply position vector by rotation matrix
    up.x = position.norm().x*m[0] + position.norm().y*m[1] + position.norm().z*m[2]; // this is the direction we want the lander's nose to point in
    up.y = position.norm().x*m[4] + position.norm().y*m[5] + position.norm().z*m[6];
    up.z = position.norm().x*m[8] + position.norm().y*m[9] + position.norm().z*m[10];
  }
  // !!!!!!!!!!!!! HINT TO STUDENTS ATTEMPTING THE EXTENSION EXERCISES !!!!!!!!!!!!!!
  // For any-angle attitude control, we just need to set "up" to something different,
  // and leave the remainder of this function unchanged. For example, suppose we want
  // the attitude to be stabilized at stabilized_attitude_angle to the vertical in the
  // close-up view. So we need to rotate "up" by stabilized_attitude_angle degrees around
  // an axis perpendicular to the plane of the close-up view. This axis is given by the
  // vector product of "up"and "closeup_coords.right". To calculate the result of the
  // rotation, search the internet for information on the axis-angle rotation formula.

  // Set left to something perpendicular to up
  left.x = -up.y; left.y = up.x; left.z = 0.0;
  if (left.abs() < SMALL_NUM) { left.x = -up.z; left.y = 0.0; left.z = up.x; }
  left = left.norm();
  out = left^up;
  // Construct modelling matrix (rotation only) from these three vectors
  m[0] = out.x; m[1] = out.y; m[2] = out.z; m[3] = 0.0;
  m[4] = left.x; m[5] = left.y; m[6] = left.z; m[7] = 0.0;
  m[8] = up.x; m[9] = up.y; m[10] = up.z; m[11] = 0.0;
  m[12] = 0.0; m[13] = 0.0; m[14] = 0.0; m[15] = 1.0;
  // Decomponse into xyz Euler angles
  orientation = matrix_to_xyz_euler(m);
}

void lander::update_members()
{
  switch (parachute_status)
  {
  case(DEPLOYED):
    acceleration = (gravity() + thrust_wrt_world() + lander_drag() + parachute_drag()) / mass;
    break;
  default:
    acceleration = (gravity() + thrust_wrt_world() + lander_drag()) / mass;
    break;
  }

  mass = UNLOADED_LANDER_MASS + fuel*FUEL_CAPACITY*FUEL_DENSITY;
  altitude = position.abs() - MARS_RADIUS;
  planetary_rotation = pow(pow(position.x, 2) + pow(position.y, 2), 0.5)*(2 * M_PI / MARS_DAY);
  fuel -= delta_t * (FUEL_RATE_AT_MAX_THRUST*throttle) / FUEL_CAPACITY;
}