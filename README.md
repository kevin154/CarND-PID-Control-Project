# CarND-PID-Controller-Project

## Overview 

The aim of this project is to implement a PID (Proportional Integral Derivative) Controller that will direct a virtual self-driving vehicle around a test track within a simulated environment. The contoller will receive real-time feedback from the environment comprised of the prevailing cross track error, steering angle and speed of the vehicle; these will be used to calculate the optimal throttle and steering angle that should be applied, which will be relayed back to the environment and form part of the next feedback loop.

The project must satisfy the following criteria:

- The vehicle must successfully complete a lap of the test track
- The PID controller must be implemented as instructed within the preceding lessons
- The PID controller algorithm parameters must be tuned appropriately to ensure that the vehicle does not oscillate wildly around the test track


## Code Layout

There are four main scripts associated with the project:

- main.cpp: The central interface to the simulator. Receives live data from the environment and returns the calculated steering angle and throttle values 
- pid.h: The outline of the PID class, which contains functions needed to calculate and optimise the outputs from the PID controller
- pid.cpp: The implementations of the functions within the PID class
- helpers.h: Some utility functions for interpreting the sensor information


## Implementation and Parameter Tuning

Two PID controllers are implemented, one for controlling the steering angle of the vehicle and the other for controlling the speed. In order to effectively control the vehicle and prevent excessive oscillations the three PID Controller parameters P, I and D corresponding to the weights placed on the proportional, integral, and derivative errors respectively need to be fine tuned.

The three parameters are outlined as follows:

- P: This sets the control weight proportionate to the size of the error, so larger errors result in larger magnitudes of feedback. Working alone this will correct for error size and but will result in the vehicle oscillating wildly and likely losing control as speed increases
- I: This accumulates the observed errors in the system and adjusts for these in order to account for biases in the system
- D: This parameter controls the error's rate of change, resulting in dampening the observed oscillations

The final parameter set was achieved using a mixture of manual tuning and the Twiddle optimisation algorithm. The steps to tune the parameters were carried out as follows:

- Set the baseline parameter values to some sample figures taken from the lectures:
  - P = 0.3
  - I = 0.0004
  - D = 3
- Adjust the P parameter until the observed vehicle behaviour is a steady set of oscillations
- Increase the D parameter until the oscillations are dampened 
- Use the Twiddle algorithm to fine-tune the parameters. In order to account for noise in the system the errors were accumulated over 100 simulation loops before the Twiddle algorithm was applied
- The final steering PID parameters for were set as follows:
  - P = 0.15
  - I = 0.0001
  - D = 3.5
  - The final speed PID parameters for were set as follows:
  - P = 0.3
  - I = 0.0004
  - D = 1

A sample of the final result is displayed in the video; we can observe that the vehicle is able to traverse the test track at a constant target speed of 30m/h in a similar way that one would expect in a real-life environent.


![](/output/PID-controller.gif)




