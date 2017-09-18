#include "lander.h"
#include "Dynamics.h"
#include "Orbiter class.h"
#include "lander_graphics.h"

void orbiter::numerical_dynamics()
{
  vector3d new_position;

  if (simulation_time == 0.0) old_position = position - delta_t*velocity;

  switch (intergrator) //switch based on intergration method chosen
  {
  case VERLET:
    new_position = 2 * position - old_position + delta_t*delta_t*acceleration;
    velocity = (new_position - position)/delta_t;
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


vector3d orbiter::get_position() { return position; }
vector3d orbiter::get_old_position() { return old_position; }
vector3d orbiter::get_velocity() { return velocity; }
vector3d orbiter::get_planetary_rotation() { return planetary_rotation; }

vector3d orbiter::get_relative_velocity()  { return relative_velocity; }
double orbiter::get_altitude()   { return altitude; }
double orbiter::get_mass()       { return mass; }

void orbiter::set_position(vector3d input_pos) 
{ 
  position = input_pos; 
  planetary_rotation = (pow(pow(position.x, 2) + pow(position.y, 2), 0.5))
    *(2 * M_PI / MARS_DAY)*vector3d { -position.norm().y, position.norm().x, 0 };
}
void orbiter::set_velocity(vector3d input_vel) { velocity = input_vel; }
void orbiter::set_altitude(double input_alt)   { altitude = input_alt; }

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
  position = vector3d{ MARS_RADIUS + 5 ,0,0 };
  velocity = vector3d{ 0,0,0 };
  mass = 0;
  radius = 0;
  update_members();
}

void orbiter::update_members()
{
  altitude           = position.abs() - MARS_RADIUS;
  acceleration       = gravity() / mass;
  planetary_rotation = (pow(pow(position.x, 2) + pow(position.y, 2), 0.5))
                      *(2 * M_PI / MARS_DAY)*vector3d { -position.norm().y, position.norm().x, 0 };
  relative_velocity  = velocity - planetary_rotation;
}

//===========================================================================================================

lander::lander()
{
  autopilot_status = ORBIT_DESCENT;
  parachute_status = NOT_DEPLOYED;
  front_facing_area = M_PI*LANDER_SIZE*LANDER_SIZE;
  radius   = LANDER_SIZE;
  landed   = FALSE;
  position = vector3d{ MARS_RADIUS + 5 ,0,0 };
  velocity = vector3d{ 0,0,0 };
  polar_velocity = vector3d{ 0,0,0 };
  virt_obj = false;
  fuel     = 1;
  throttle = 0;
  mass     = 0;
  update_members();
  return;
}

lander::~lander() 
{
  autopilot(true); //remove all the pointers stored in static variables
}

void lander::set_virt_obj(bool input) { virt_obj = input; }
void lander::set_orientation(vector3d input_orientation) { orientation = input_orientation; }
vector3d lander::get_orientation() { return orientation; }
double lander::get_climb_speed()   { return climb_speed; }
double lander::get_ground_speed()  { return ground_speed; }

vector3d lander::thrust_wrt_world(void)
// Works out thrust vector in the world reference frame, given the lander's orientation
{
  double m[16], k, delayed_throttle, lag = ENGINE_LAG;
  vector3d a, b;
  static double lagged_throttle = 0.0;
  static double last_time_lag_updated = -1.0;

  //if (simulation_time < last_time_lag_updated) lagged_throttle = 0.0; // simulation restarted
  if (throttle < 0.0) throttle = 0.0;
  if (throttle > 1.0) throttle = 1.0;
  if (landed || (fuel == 0.0)) throttle = 0.0;

  if (simulation_time != last_time_lag_updated) 
  {
    // Delayed throttle value from the throttle history buffer
    if (throttle_buffer_length > 0) {
      delayed_throttle = throttle_buffer[throttle_buffer_pointer];
      throttle_buffer[throttle_buffer_pointer] = throttle;
      throttle_buffer_pointer = (throttle_buffer_pointer + 1) % throttle_buffer_length;
    }
    else delayed_throttle = throttle;

    // Lag, with time constant ENGINE_LAG
    if (lag <= 0.0) k = 0.0;
    else k = pow(exp(-1.0), delta_t / lag);
    lagged_throttle = k*lagged_throttle + (1.0 - k)*delayed_throttle;


    last_time_lag_updated = simulation_time;
  }
    delayed_throttle = throttle;
  if (stabilized_attitude && (abs(stabilized_attitude_angle) < 1E-7) && !autopilot_enabled && (autopilot_status != ORBIT_DESCENT)) { // specific solution, avoids rounding errors in the more general calculation below
    b = throttle*MAX_THRUST*position.norm();
  }
  else {
    a.x = 0.0; a.y = 0.0; a.z = throttle*MAX_THRUST;
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
    up = -relative_velocity.norm();
  }
  else if (abs(stabilized_attitude_angle) < 1E-7) //skip expensive calculations if aproximately 0
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
  return;
}

void lander::update_members()
{
  fuel    -= delta_t * (FUEL_RATE_AT_MAX_THRUST*throttle) / FUEL_CAPACITY;
  mass     = UNLOADED_LANDER_MASS + fuel*FUEL_CAPACITY*FUEL_DENSITY;
  altitude = position.abs() - MARS_RADIUS;
  if (fuel < 0) fuel = 0.0;
  planetary_rotation = std::sqrt(pow(position.x, 2) + pow(position.y, 2))*(2 * M_PI / MARS_DAY)*vector3d { -position.norm().y, position.norm().x, 0 };
  relative_velocity  = velocity - planetary_rotation;

  //note originally taken from safe_to_deploy function
  if (parachute_drag().abs() > MAX_PARACHUTE_DRAG || (velocity.abs() > MAX_PARACHUTE_SPEED && (altitude < EXOSPHERE))) safe_to_deploy_parachute = false;
  else safe_to_deploy_parachute = true;

  //originally average of current and old position used
  vector3d av_p = (old_position + position) / 2;
  climb_speed   = velocity*av_p.norm();
  ground_speed  = (relative_velocity - climb_speed*av_p.norm()).abs();
  polar_velocity.x = climb_speed;
  polar_velocity.y = (velocity - climb_speed*av_p.norm()).abs();

  if (landed || (fuel == 0.0)) throttle = 0.0;

  return;
}