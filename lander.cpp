// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
  // INSERT YOUR CODE HERE
  double K_h = 0.04;
  double K_p = 0.7;
  double h = position.abs() - MARS_RADIUS;  // altitude
  double e = - (0.5 + K_h*h + velocity*position/position.abs());
  double P_out = K_p * e;
  double density = atmospheric_density(position);
  double frontal_area = 3.14159 * LANDER_SIZE*LANDER_SIZE;
  vector3d f_grav = - GRAVITY * MARS_MASS * (UNLOADED_LANDER_MASS + fuel*FUEL_DENSITY*FUEL_CAPACITY) * position / (position.abs()*position.abs()* position.abs());
  vector3d f_drag;
  
  if (parachute_status == NOT_DEPLOYED || parachute_status == LOST) { // either or
    f_drag = -velocity * 0.5 * density * (DRAG_COEF_LANDER)*frontal_area * (velocity.abs());
  }
  else {
    f_drag = -velocity * 0.5 * density * (DRAG_COEF_LANDER)*frontal_area * (velocity.abs())
    - velocity * 0.5 * density * (DRAG_COEF_CHUTE) * 4 * frontal_area * (velocity.abs());
  }
    double Delta = 0.7 * ((f_grav - f_drag).abs() / MAX_THRUST);
    
    if (P_out <= -Delta){
      throttle = 0.0;
    }
    else if (-Delta < P_out < 1-Delta){
      throttle = Delta + P_out;
    }
    else{
      throttle = 1.0;
    }
    
    
    
  }
  
  void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
  {
    // INSERT YOUR CODE HERE
    vector3d acc, force, f_grav, f_thrust, f_drag; // local acceleration variable
    static vector3d prev_position, new_position;
    double object_mass, density, frontal_area;
    
    density = atmospheric_density(position);
    frontal_area = 3.14159 * LANDER_SIZE*LANDER_SIZE; //pi*r^2
    object_mass = UNLOADED_LANDER_MASS + fuel*FUEL_DENSITY*FUEL_CAPACITY;
    f_grav = - GRAVITY * MARS_MASS * (object_mass) * position / (position.abs()*position.abs()* position.abs());
    f_thrust = thrust_wrt_world();
    if (parachute_status == NOT_DEPLOYED || parachute_status == LOST) { // either or
      f_drag = -velocity * 0.5 * density * (DRAG_COEF_LANDER)*frontal_area * (velocity.abs());
    }
    else {
      f_drag = -velocity * 0.5 * density * (DRAG_COEF_LANDER)*frontal_area * (velocity.abs())
      - velocity * 0.5 * density * (DRAG_COEF_CHUTE) * 4 * frontal_area * (velocity.abs()); // I set parachute's frontal area to be four times that of the lander
    }
    force = f_grav + f_thrust + f_drag;
    acc = force / object_mass;
    
    // use Euler for first integration step and Verlet otherwise
    if (simulation_time == 0.0){
      // Euler Integrator
      prev_position = position;
      new_position = position + delta_t * velocity;
      velocity = velocity + delta_t * acc;
    }
    else{
      // Verlet Integrator
      new_position = 2*position - prev_position + acc * delta_t* delta_t;
      velocity = 1/(2*delta_t) * (new_position - prev_position);
      // update previous position:
      prev_position = position;
    }
    position = new_position;
    
    
    // Here we can apply an autopilot to adjust the thrust, parachute and attitude
    if (autopilot_enabled) autopilot();
    
    // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
    if (stabilized_attitude) attitude_stabilization();
  }

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
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
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
