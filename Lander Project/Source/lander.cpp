// Mars lander simulator
// Version 1.9
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2016

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include "Dynamics.h"
#include "lander_graphics.h"


fstream fout;
void autopilot(const double &lander_mass)
// Autopilot to adjust the engine throttle, parachute and attitude control
{
  constexpr double ideal_ver = 0.5;
  constexpr double Kp        = 1;
  bool engage                = 0;
  double ground_speed        = ((velocity - atmosphere_rotation()) - (velocity - atmosphere_rotation())*position.norm()*position.norm()).abs();
  double direction           = position.norm()*(velocity - atmosphere_rotation()).norm();
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
    delta = gravity(lander_mass).abs() / MAX_THRUST; // - drag()*gravity(lander_mass).norm() considering drag as part of the thrus seems to make fuel efficiency worse
    
    if (parachute_status == LOST)  Kh = 0.018;
    else                           Kh = 0.03;

    error = -(ideal_ver + Kh*altitude + ver);
    Pout  = Kp*error;

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

void numerical_dynamics(void)
// This is the function that performs the numerical integration to update the
// lander's pose. The time step is delta_t (global variable).
{
  //declare old and new potision variables for verlet intergrator
  static vector3d old_position; //do not assign here, as will not reset when new scenario selected
  vector3d new_position;
  static double kp_test;
  //calculate the lander's mass and acceleration
  planetary_rotation_update(); //update the rotation of the planet relative to the lander
  double lander_mass = UNLOADED_LANDER_MASS + fuel*FUEL_CAPACITY*FUEL_DENSITY;
  vector3d acceleration;
  switch (parachute_status)
  {
  case(DEPLOYED):
    acceleration = (gravity(lander_mass) + thrust_wrt_world() + lander_drag() + parachute_drag()) / lander_mass;
    break;
  default:
    acceleration = (gravity(lander_mass) + thrust_wrt_world() + lander_drag()) / lander_mass;
    break;
  }

  //so that if the simulation is reset so does the old position
  if (simulation_time == 0.0)
  {
    old_position = position - delta_t*velocity;
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
    autopilot(lander_mass);
  }

}

void initialize_simulation(void)
// Lander pose initialization - selects one of 10 possible scenarios
{
  static double aerostationary_radius = pow((GRAVITY*MARS_MASS*MARS_DAY*MARS_DAY) / (4 * M_PI*M_PI), 0.333333333);
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "aerostationary orbit";
  scenario_description[7] = "descent at perfect rotation from 10km";
  scenario_description[8] = "descent at perfect rotation from 200km";
  scenario_description[9] = "";

  //reset common parameters
  delta_t = 0.1;
  parachute_status = NOT_DEPLOYED;
  stabilized_attitude_angle = 0;
  autopilot_status = ORBIT_DESCENT;
  wind_enabled = FALSE;

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -pow(GRAVITY*MARS_MASS / (1.2*MARS_RADIUS), 0.5), 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    stabilized_attitude = false;
    autopilot_enabled = false;
    autopilot_status = ORBIT_RE_ENTRY;
    break;

  case 1:
    // a descent from rest at 10km altitude
    //fout.open("Power_scenerio_1.txt", ios::app);
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    stabilized_attitude = false;
    autopilot_enabled = false;
    autopilot_status = ORBIT_RE_ENTRY;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE / 2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    stabilized_attitude = false;
    autopilot_enabled = false;
    autopilot_status = ORBIT_INJECTION;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    //fout.open("Power_scenerio_5.txt", ios::app);
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    //orbit above a fixed point on the martian equator
    position = vector3d(aerostationary_radius, 0.0, 0.0);
    velocity = vector3d(0.0, pow(GRAVITY*MARS_MASS / aerostationary_radius, 0.5), 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    stabilized_attitude = true;
    autopilot_enabled = false;
    autopilot_status = ORBIT_RE_ENTRY;
    break;

  case 7:
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    planetary_rotation_update();
    velocity = vector3d(planetary_rotation, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 8:
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    planetary_rotation_update();
    velocity = vector3d(planetary_rotation, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 9:
    break;

  }
}