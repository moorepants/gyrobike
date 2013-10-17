This contains code for generating and simulating the non-linear and linear
gyrobike models explained in:

http://moorepants.github.io/dissertation/extensions.html#front-wheel-flywheel

`src/autolev` contains the code which generates the equations of motion of the
system and gives a detailed explanation of the system. `src/c` contains the
Autolev generated C code for simulation purposes. `src/matlab` contains the
Autolev generated Matlab code for simulation purposes and some hand crafted
functions for simulation of the non-linear and linear system.

The original model written in Python is available here:

https://github.com/moorepants/dissertation/tree/master/src/extensions/gyro

License
=======

See the UNLICENSE file.
