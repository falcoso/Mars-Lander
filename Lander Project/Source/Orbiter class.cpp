#include "lander.h"
#include "Dynamics.h"
#include "Orbiter class.h"

void orbiter::numerical_dynamics()
{
  static vector3d old_position; //do not assign here, as will not reset when new scenario selected
  vector3d new_position;

  acceleration = gravity() / mass;
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
  return;
}

vector3d orbiter::gravity()
{
  return -(GRAVITY*MARS_MASS*mass / position.abs2())*position.norm();
}

lander::lander(double construction_radius)
{
  fuel = 1;
  throttle = 0;
  autopilot_status = ORBIT_DESCENT;
  parachute_status = NOT_DEPLOYED;
  radius = construction_radius;
  front_facing_area = M_PI*pow(radius, 2);

}

void lander::numerical_dynamics()
{
  {
    //declare old and new potision variables for verlet intergrator
    static vector3d old_position; //do not assign here, as will not reset when new scenario selected
    vector3d new_position;
    static double kp_test;
    //calculate the lander's mass and acceleration
    planetary_rotation_update(); //update the rotation of the planet relative to the lander
    mass = UNLOADED_LANDER_MASS + fuel*FUEL_CAPACITY*FUEL_DENSITY;
    switch (parachute_status)
    {
    case(DEPLOYED):
      acceleration = (gravity() + thrust_wrt_world() + lander_drag() + parachute_drag()) / mass;
      break;
    default:
      acceleration = (gravity() + thrust_wrt_world() + lander_drag()) / mass;
      break;
    }

    //so that if the simulation is reset so does the old position
    if (simulation_time == 0.0)
    {
      old_position = position - delta_t*velocity;
      if (autopilot_status == ORBIT_DESCENT) kp_test = kh_set(0.01, 0.025);
    }

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
  }
}


vector3d lander::parachute_drag(void)
{
  vector3d rotation = atmosphere_rotation() + wind()*atmosphere_rotation().norm();
  return -0.5*atmospheric_density(position)*DRAG_COEF_CHUTE*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE*(velocity - rotation).abs()*(velocity - rotation);
}

vector3d lander::lander_drag(void)
{
  constexpr double front_facing_area = M_PI*LANDER_SIZE*LANDER_SIZE;
  vector3d rotation = atmosphere_rotation() + wind()*atmosphere_rotation().norm();
  return -0.5*front_facing_area*atmospheric_density(position)*(DRAG_COEF_LANDER)*(velocity - rotation).abs()*(velocity - rotation);
}

void lander::autopilot()
{
  constexpr double ideal_ver = 0.5;
  constexpr double Kp = 1;
  bool engage = 0;
  double ground_speed = ((velocity - atmosphere_rotation()) - (velocity - atmosphere_rotation())*position.norm()*position.norm()).abs();
  double direction = position.norm()*(velocity - atmosphere_rotation()).norm();
  static double timer;
  double Kh, ver, altitude, delta, error, Pout;

  if (simulation_time == 0)
  {
    timer = 0.0;
  }

  switch (autopilot_status)
  {
  case ORBIT_RE_ENTRY:
    stabilized_attitude_angle = acos(position.norm()*(velocity - atmosphere_rotation()).norm()) + M_PI;
    if ((abs(velocity*position.norm()) < 10 && velocity.abs() != 0) && timer <= 60)
    {
      throttle = 1;
      timer += delta_t;
    }
    else
    {
      autopilot_status = ORBIT_DESCENT;
      throttle = 0;
      stabilized_attitude_angle = 0;
      std::cout << "ORBITAL RE-ENTRY COMPLETE\n";
      std::cout << "Fuel level: " << fuel * 100 / FUEL_CAPACITY << "%\n";
      std::cout << "Descent Speed: " << velocity*position.norm() << "m/s\n";
      std::cout << "COMMENCE ORBITAL DESCENT" << std::endl;
    }
    break;

  case ORBIT_DESCENT:
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
    delta = gravity().abs() / MAX_THRUST; // - drag()*gravity(mass).norm() considering drag as part of the thrus seems to make fuel efficiency worse

    if (parachute_status == LOST)  Kh = 0.018;
    else                           Kh = 0.03;

    error = -(ideal_ver + Kh*altitude + ver);
    Pout = Kp*error;

    //Proportional gain control
    if (Pout <= -delta)
    {
      throttle = 0;
    }
    else if (Pout >= 1 - delta)
    {
      throttle = 1;
      engage = 1;
    }
    else
    {
      throttle = delta + Pout;
      engage = 1;
    }

    //in attitude_stabilization() up is set to velocity direction, though this is still required for it to stay stable???
    if (velocity*closeup_coords.right < 0) stabilized_attitude_angle = M_PI + acos(position.norm()*(velocity - atmosphere_rotation()).norm());
    else                                   stabilized_attitude_angle = M_PI - acos(position.norm()*(velocity - atmosphere_rotation()).norm());

    //Determine parachute release
    if (parachute_status == NOT_DEPLOYED && altitude < 50000) //if lost or already deployed, save processing and skip next
    {
      if ((safe_to_deploy_parachute() && ver < 0) && (engage == 1 || open_chute_query()))
      {//must always be safe to deploy and falling towards mars, as well as either, cause correct deceleration 
       //to not break or already have the throttle engaged, which will assist in braking
        parachute_status = DEPLOYED;
        std::cout << "PARACHUTE SUCCESSFULLY OPENED\n";
        std::cout << "Current Altitude: " << position.abs() - MARS_RADIUS << "m\n";
        std::cout << "Descent Speed: " << velocity*position.norm() << "m/s\n";
      }
    }
    else if (parachute_status == DEPLOYED && altitude < 1000) //reduce drag at lower level
    {
      if (ground_speed < abs(1.1*wind()) && wind() > 20)
      {
        parachute_status = LOST;
        std::cout << "PARACHUTE EJECTED TO REDUCE GROUND SPEED\n";
        std::cout << "Current Altitude: " << position.abs() - MARS_RADIUS << "m\n";
        std::cout << "Descent Speed: " << velocity*position.norm() << "m/s\n";
      }
    }
    break;
  case ORBIT_INJECTION:
    fuel = 1;
    if (position.abs() < 1.2*MARS_RADIUS)
    {
      Kh = 0.3;
      ver = 0;
      error = -Kh*(1.2*MARS_RADIUS - position.abs()) + ver;
      Pout = Kp*error;
    }
    else
    {
      stabilized_attitude_angle = M_PI / 2;
      error = pow(GRAVITY*MARS_MASS / (position.abs()), 0.5) - velocity.abs()*cos(M_PI / 2);
      Pout = Kp*error;
    }

    if (Pout < 0)      throttle = 0;
    else if (Pout > 1) throttle = 1;
    else               throttle = Pout;

    break;
  }
}