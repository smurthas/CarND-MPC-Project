# MPC


### NOTE:
The project submission
[page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/project)
suggests that there should be a reflect in text form "addressing the questions
in the rubric", however the rubric doesn't have any questions listed.

As such, I'll simply describe my implementation and the choices I made.

### Reflection

My implementation is based off of the quizzes and lectures, with some extra work
done to handle the fact that there is a 100ms actuator delay. The general
technique is to use the vehicle's equations of motion and a cost function to do
an iterative, constrained optimization that calculates a lowest-cost time
series of poses and actuations. From there, we simply use the first actuation
values as our output to the vehicle controls.

*Hyperparameters*:

I choose to use six time steps (N=6) each of which is 200ms long (dt=0.2). I
experimented with higher and lower values for each and found that with shorted
time horizons (N or dt lower), the vehicle was too erratic and with more
timestamps (N > 6), the solver time began to negatively impact the
responsiveness, often taking up to 100ms to converge, which tended to cause the
car to respond too late or incorrectly.

*Coordinate transform*:

Given a set of waypoints from the simulator, I fit a third order polynomial to
them to pass to the solver. However, to make that processes easier, I first
transformed the coordinates to the frame of the vehicle. This also made it
possible to display green and yellow lines in the simulator for the MPC
predicted path and waypoints (respectively).

*Actuator delay*:

To handle the actuator delay (100ms), I estimated the vehicle's pose at
actuation time using the equations of motion. I then used that pose as the input
to the solver, i.e. the coordinate transform of the waypoints referenced above
was done relative to the estimated actuation pose and that pose is passed in to
the solver as the initial state (which was simply x=0, y=0, psi=0 since
everything was in the vehicle frame)

*Cost function*:
The cost function used in the constrained optimization above uses the sum of
square of several values:

1. CTE: this encourages the solution to remain near the waypoint path
2. PSI Error: this encourages the solution to be longitudinally aligned with the
waypoint path
3. Velocity: this ensures that the solution doesn't simply drive the vehicle to
the waypoint pat and stop (which would not be useful, but would minimize all
other costs)
4. Steering angle: steering angle is directly penalized to avoid extreme angles
5. Accelerator: accelerator position is directly penalized to avoid extreme
acceleration and braking
6. Steering angle change: the change in angle from one step to the next is
penalized to encourage smooth changes
7. Accelerator position change: the change in accelerator position from one step
to the next is penalized to encourage smooth changes

