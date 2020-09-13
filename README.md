# CarND-PID-Controller-Project

## Overview 

The aim of this project is to implement a PID (Proportional Integral Derivative) Controller that will assist with navigating a virtual self-driving vehicle around a test track within a simulated environment. The contoller will receive real-time feedback from the environment comprised of the prevailing cross track error, steering angle and speed of the vehicle; these will be used to calculate the optimal throttle and steering angle that should be applied, which will then be relayed back to the environment and form part of the next feedback loop.

The project must satisfy the following criteria:

- The vehicle must successfully complete a lap of the test track without leaving the driveable surface
- The PID controller must be implemented as instructed within the preceding lectures
- The PID controller algorithm parameters must be tuned appropriately to ensure that the vehicle does not oscillate wildly while navigating test track


## Code Layout

There are four main scripts associated with the project:

- main.cpp: The central interface to the simulator. Receives live data from the environment and returns the calculated steering angle and throttle values 
- pid.h: The outline of the PID class, which contains functions needed to calculate and optimise the outputs from the PID controller
- pid.cpp: The implementations of the functions within the PID class
- helpers.h: Some utility functions for interpreting the sensor information


## Implementation and Parameter Tuning

Two PID controllers are implemented, one for controlling the steering angle of the vehicle and the other for controlling the speed. In order to effectively control the vehicle and prevent excessive oscillations the three PID Controller parameters P, I and D corresponding to the weights placed on the proportional, integral, and derivative errors respectively need to be fine tuned.

The three parameters are defined as follows:

- P: This sets the control weight proportionate to the size of the error, so larger errors result in larger magnitudes of feedback. Working alone this will correct for error size and but will result in the vehicle oscillating wildly and likely losing control and leaving the test track as speed increases
- I: This accumulates the observed errors in the system and adjusts for these in order to account for biases in the system
- D: This is a control related to the the error's rate of change, which can be used to effectivey dampen the observed oscillations

Parameter optimsation was performed using a mixture of manual tuning and the Twiddle algorithm. The steps to set the parameters are outlined as follows:

- Set the baseline parameter values to some sample figures taken from the lectures:
  - P = 0.3
  - I = 0.0004
  - D = 3
- Adjust the P parameter until the observed vehicle behaviour is a steady state of oscillations
- Increase the D parameter until the oscillations are sufficiently dampened 
- Use the Twiddle algorithm to fine-tune the parameters. In order to account for noise in the system the errors are accumulated over 100 simulation loops before the Twiddle algorithm is applied. In an ideal setting the parameters would be updated upon each complete lap however this is much too time consuming to be realistically considered for the purposes of the project. 
- The final steering PID parameters for were set as follows:
  - P = 0.15
  - I = 0.0001
  - D = 3.5
- The final speed PID parameters for were set as follows:
  - P = 0.3
  - I = 0.0004
  - D = 1

A sample of the lap using the final parameters is displayed in the video below; we observe that the vehicle is able to traverse the test track at a target speed of 30m/h in a similar way that one would expect from a human driver.

![](/output/PID-controller.gif)
