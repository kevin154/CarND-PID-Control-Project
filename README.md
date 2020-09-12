# CarND-PID-Controller-Project

## Overview 

The aim of this project is to implement a PID (Proportional Integral Derivative) Controller that will direct a virtual self-driving vehicle around a test track within a simulated environment. The contoller will receive real-time feedback from the environment comprised of the prevailing cross track error, steering angle and speed of the vehicle; these will be used to calculate the optimal throttle and steering angle that should be applied, which will be relayed back to the environment and form part of the next feedback loop.

The project must satisfy the following criteria:

* The vehicle must successfully complete a lap of the test track
* The PID controller must be implemented as instructed within the preceding lessons
* The PID controller algorithm parameters must be tuned appropriately to ensure that the vehicle does not oscillate wildly around the test track

## Code Layout

There are four main files associated with the project:

main.cpp: The central interface to the simulator. Receives live data from the environment and returns the calculated steering angle and throttle values 
pid.hpp: The outline of the PID class, which contains functions needed to calculate and optimise the outputs from the PID controller
car.cpp: The implementations of the functions within the PID class
helpers.h: Some utility functions for interpreting the sensor information


## Implementation

Two PID controllers were implemented, one for controlling the steering angle of the vehicle and the other for controlling the speed. In order to prevent excessive oscillations the three PID Controller parameters (corresponding to the weights placed on the proportional, integral, and derivative errors respectively) needed to be fine tuned; this was carried out using a mixture of manual tuning from observing test runs and using the Twiddle optimisation algorithm. 



![](/output/PID-controller.gif)




