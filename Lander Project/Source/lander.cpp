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
#include "Orbiter class.h"

extern lander mars_lander;
void lander::autopilot(bool reset)
{
  constexpr double ideal_ver = 0.5;
  constexpr double Kp = 1;
  double direction    = position.norm()*relative_velocity.norm();
  static double *timer = new double (0.0);
  double delta, error, Pout;
  static bool *burst_complete = new bool(false);

  //constants for TRANSFER_ORBIT
  constexpr double apogee_radius = EXOSPHERE/3 +MARS_RADIUS;
  double unit_impulse = delta_t*MAX_THRUST;
  double transfer_impulse_time;
  static double *initial_radius = new double(position.abs());
  double perigee_velocity;
  static double *transfer_radius = nullptr;


  if (reset)
  {
    delete burst_complete;
    burst_complete = nullptr;
    delete initial_radius;
    initial_radius = nullptr;
    delete timer;
    timer = nullptr;
    delete transfer_radius;
    transfer_radius = nullptr;
    return;
  }

  switch (autopilot_status)
  {
  case TRANSFER_ORBIT:
    fuel = 1;
    stabilized_attitude_angle = (float)(acos(position.norm()*relative_velocity.norm()) + M_PI);
    if (velocity.abs() > perigee_velocity && !*burst_complete)
    {
      throttle = 1;
    }
    else
    {
      *burst_complete = true;
      throttle = 0;
    }
    break;

  case ORBIT_RE_ENTRY:
    if (burst_complete == nullptr) burst_complete = new bool(false);
    if (initial_radius == nullptr) initial_radius = new double(position.abs());
    if (transfer_radius == nullptr)
    {
      transfer_radius = new double;
      std::cout << "Input transfer radius:" << std::endl;
      std::cin >> *transfer_radius;
      *transfer_radius *= MARS_RADIUS;
    }

    perigee_velocity = std::sqrt((2 * GRAVITY*MARS_MASS*(*transfer_radius)) / ((*initial_radius)*((*transfer_radius) + (*initial_radius))));
    stabilized_attitude_angle = (float)(acos(position.norm()*relative_velocity.norm()) + M_PI);
    if (velocity.abs() > perigee_velocity && !*burst_complete)
    {
      throttle = 1;
    }
    else
    {
      *burst_complete = true;
      throttle = 0;
      if (altitude < EXOSPHERE)
      {
        autopilot_status = ORBIT_DESCENT;
        Kh = kh_tuner(this, tuning_mode);
        stabilized_attitude_angle = 0;
        std::cout << "***ORBITAL RE-ENTRY COMPLETE***\n"
          << "Fuel level:\t" << fuel * 100 << "%\n"
          << "Descent Speed:\t" << fabs(velocity*position.norm()) << " m/s\n"
          << "***COMMENCE ORBITAL DESCENT***" << std::endl;
        delete timer; //reset timer here, as will not reset unless autopilot is called exactly at t = 0 otheriwise
        delete burst_complete;
      }
      //else if (fabs(climb_speed) < 5)
      //{
      //  delete timer;
      //  delete burst_complete;
      //  std::cout << "***OBRITAL TRANSFER PHASE 1 COMPLETE***\n"
      //    << "Fuel level:\t" << fuel * 100 << "%\n"
      //    << "***COMMENCE ORBITAL TRANSFER PHASE 2***" << std::endl;
      //}
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
    delta = gravity().abs() / MAX_THRUST; // - drag()*gravity(lander_mass).norm() considering drag as part of the thrus seems to make fuel efficiency worse

    //if (parachute_status == LOST)  Kh = 0.018;
    //else                           Kh = 0.03;

    error = -(ideal_ver + Kh*altitude + climb_speed);
    Pout  = Kp*error;

    //Proportional gain control
    if (Pout <= -delta)          throttle = 0;
    else if (Pout >= 1 - delta)  throttle = 1;
    else                         throttle = delta + Pout;

    //in attitude_stabilization() up is set to velocity direction, though this is still required for it to stay stable???
    if (velocity*closeup_coords.right < 0) stabilized_attitude_angle = (float)(M_PI + acos(direction));
    else                                   stabilized_attitude_angle = (float)(M_PI - acos(direction));

    //Determine parachute release
    if (parachute_status == NOT_DEPLOYED && altitude < 50000 && safe_to_deploy_parachute) //if lost or already deployed, save processing and skip next
    {
      //must always be safe to deploy and falling towards mars, as well as either, cause correct deceleration 
      //to not break or already have the throttle engaged, which will assist in braking
      parachute_status = DEPLOYED;
      if (!virt_obj)
      {
        std::cout << "***PARACHUTE SUCCESSFULLY OPENED***\n"
          << "Altitude:\t" << altitude << " m\n"
          << "Descent Speed:\t" << fabs(velocity*position.norm()) << " m/s" << std::endl;
      }
    }
    else if (parachute_status == DEPLOYED && altitude < 1000) //reduce drag at lower level
    {
      if (ground_speed < abs(1.1*wind()) && wind() > 20)
      {
        parachute_status = LOST;
        if (!virt_obj)
        {
          std::cout << "***PARACHUTE EJECTED TO REDUCE GROUND SPEED***\n"
            << "Altitude:\t" << altitude << "m\n"
            << "Descent Speed:\t" << velocity*position.norm() << "m/s" << std::endl;
        }
      }
    }
    break;
  case ORBIT_INJECTION:
    fuel = 1;
    if (position.abs() < 1.2*MARS_RADIUS)
    {
      Kh    = 0.3;
      //ideal_ver   = 0;
      error = -Kh*(1.2*MARS_RADIUS - position.abs()) + climb_speed;
      delta = gravity().abs() / MAX_THRUST;
      Pout  = Kp*error;
    }
    else
    {
      stabilized_attitude_angle = (float)(M_PI / 2);
      error = pow(GRAVITY*MARS_MASS / (position.abs()), 0.5) - velocity.abs()*cos(M_PI / 2);
      Pout  = Kp*error;
    }

    if (Pout <= -delta)          throttle = 0;
    else if (Pout >= 1 - delta)  throttle = 1;
    else                         throttle = delta + Pout;

    break;
  }
}

void lander::numerical_dynamics()
// This is the function that performs the numerical integration to update the
// lander's pose. The time step is delta_t (global variable).
{
  //declare old and new potision variables for verlet intergrator
  vector3d new_position;

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

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled)
  {
    if (Kh == 0) Kh = 0.018;
    stabilized_attitude = 1;
    autopilot();
  }
  return;
}

void initialize_simulation(void)
// Lander pose initialization - selects one of 10 possible scenarios
{
  mars_lander.autopilot(true);
  static double aerostationary_radius = (double)pow((GRAVITY*MARS_MASS*MARS_DAY*MARS_DAY) / (4 * M_PI*M_PI), 0.333333333);
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
  mars_lander.parachute_status = NOT_DEPLOYED;
  mars_lander.stabilized_attitude_angle = 0;
  mars_lander.autopilot_status = ORBIT_DESCENT;
  wind_enabled = FALSE;

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    mars_lander.set_position (vector3d(1.2*MARS_RADIUS, 0.0, 0.0));
    mars_lander.set_velocity (vector3d(0.0, -pow(GRAVITY*MARS_MASS / (1.2*MARS_RADIUS), 0.5), 0.0));
    mars_lander.set_orientation(vector3d(0.0, 90.0, 0.0));
    mars_lander.stabilized_attitude = false;
    mars_lander.autopilot_enabled = false;
    mars_lander.autopilot_status = ORBIT_RE_ENTRY;
    break;

  case 1:
    // a descent from rest at 10km altitude
    //fout.open("Power_scenerio_1.txt", ios::app);
    mars_lander.set_position(vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0));
    mars_lander.set_velocity(vector3d(0.0, 0.0, 0.0));
    mars_lander.set_orientation(vector3d(0.0, 0.0, 90.0));
    mars_lander.stabilized_attitude = true;
    mars_lander.autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    mars_lander.set_position(vector3d(0.0, 0.0, 1.2*MARS_RADIUS));
    mars_lander.set_velocity(vector3d(3500.0, 0.0, 0.0));
    mars_lander.set_orientation(vector3d(0.0, 0.0, 90.0));
    mars_lander.stabilized_attitude = false;
    mars_lander.autopilot_enabled = false;
    mars_lander.autopilot_status = ORBIT_RE_ENTRY;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    mars_lander.set_position(vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE / 2.0));
    mars_lander.set_velocity(vector3d(0.0, 0.0, 5027.0));
    mars_lander.set_orientation(vector3d(0.0, 0.0, 0.0));
    mars_lander.stabilized_attitude = false;
    mars_lander.autopilot_enabled = false;
    mars_lander.autopilot_status = ORBIT_DESCENT;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    mars_lander.set_position(vector3d(0.0, 0.0, MARS_RADIUS + 100000.0));
    mars_lander.set_velocity(vector3d(4000.0, 0.0, 0.0));
    mars_lander.set_orientation(vector3d(0.0, 90.0, 0.0));
    mars_lander.stabilized_attitude = false;
    mars_lander.autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    //fout.open("Power_scenerio_5.txt", ios::app);
    mars_lander.set_position(vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0));
    mars_lander.set_velocity(vector3d(0.0, 0.0, 0.0));
    mars_lander.set_orientation(vector3d(0.0, 0.0, 90.0));
    mars_lander.stabilized_attitude = true;
    mars_lander.autopilot_enabled = false;
    break;

  case 6:
    //orbit above a fixed point on the martian equator
    mars_lander.set_position(vector3d(aerostationary_radius, 0.0, 0.0));
    mars_lander.set_velocity(vector3d(0.0, pow(GRAVITY*MARS_MASS / aerostationary_radius, 0.5), 0.0));
    mars_lander.set_orientation(vector3d(0.0, 0.0, 90.0));
    mars_lander.stabilized_attitude = true;
    mars_lander.autopilot_enabled = false;
    mars_lander.autopilot_status = ORBIT_RE_ENTRY;
    break;

  case 7:
    mars_lander.set_position(vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0));
    mars_lander.set_velocity(vector3d(mars_lander.get_planetary_rotation().abs(), 0.0, 0.0));
    mars_lander.set_orientation(vector3d(0.0, 0.0, 90.0));
    mars_lander.stabilized_attitude = true;
    mars_lander.autopilot_enabled = false;
    break;

  case 8:
    mars_lander.set_position(vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0));
    mars_lander.set_velocity(vector3d(mars_lander.get_planetary_rotation().abs(), 0.0, 0.0));
    mars_lander.set_orientation(vector3d(0.0, 0.0, 90.0));
    mars_lander.stabilized_attitude = true;
    mars_lander.autopilot_enabled = false;
    break;

  case 9:
    break;
  }
  mars_lander.update_members();
}
