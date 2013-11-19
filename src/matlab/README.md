This directory contains several Matlab files for working with the Gyrobike
model:

- `example_linear_analysis.m`: Script showing how to work with the linear
  model.
- `example_nonlinear_simulation.m`: Script showing how to work with the
  non-linear model.
- `gyrobike_constants_without_rider.txt`: Constants for all the geometry, mass,
  and inertia of the system (bicycle only).
- `gyrobike_constants_with_rider.txt`: Constants for all the geometry, mass,
  and inertia of the system (bicycle with rigid rider).
- `gyrobike_linear.m`: Function that generates the linear model.
- `gyrobike.m`: Autolev generated code.
- `gyrobike_rhs.m`: Function containing the right hand side of the ODEs which
  computes the derivatives of the states.
- `par_text_to_struct.m`: Loads the constants file into a Matlab structure.
- `steer_axis_tilt.m`: Function that computes the steer axis tilt in the
  nominal configuration.
