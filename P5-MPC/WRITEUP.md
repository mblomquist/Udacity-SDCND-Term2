# Model Predictive Controller
## Overview

This project consists of a model-predictive controller that is being used to control the throttle and steering of a car running in the Udacity Simulator. The model-predictive controller assumes a simplified kinematic model that can be manipulated through two actuators that control the acceleration (throttle) and the steering angle. A cost-function is applied to the kinematic model and the cross-track error is minimized to achieve a stable driving condition (i.e. the car stays on the track). 

## Model Implementation
### Kinematics

A simplified kinematic model was implemented in the model predictive controller design that consists of the state equations listed below.

State Equations: 

```
State - [x, y, psi, v]

- x_t+1 = x_t + v_t * cos(psi_t) * dt
- y_t+1 = y_t + v_t * sin(psi_t) * dt
- psi_t+1 = psi_t + v_t / L_f * delta_t * dt
- v_t+1 = v_t + alpha_t * dt
```

Where:
- x is the global x position of the vehicle.
- y is the global y position of the vehicle.
- psi is the orientation of the vehicle in radians.
- v is the velocity magnitude of the vehicle.

The control parameters (actuators) integrate into the state equations by manipulating the acceleration and steering angle. The steering angle of the car is implmented in the equation for the orientation of the vehicle (psi) and the acceleration is implemented in the velocity equation of the vehicle. In the code, the actuators are represented by ```delta``` and ```alpha```.

Actuators: [delta, alpha]

Where: 
- delta is the steering angle
- alpha is the throttle control parameter (throttle/brake)

Note: L_f measures the distance between the front of the vehicle and its center of gravity. The larger the vehicle, the slower the turn rate.

Additionally, a set of limitations were placed on the steering angle and the acceleration parameters. The constraints represent the limitations of the car and are shown below.

Actuator Constraints:
- Steering Angle: [-30, 30] 
- Acceleration (Throttle Parameter): [-1, 1]

The steering angle of the car is limited by a maximum 30 degree travel and the throttle is controlled to a value of +/- 1. At 1, the throttle represents the maximum forward acceleration (-1 represents the maximum reverse throttle).

### Cost Function and Optimization

In order to optimize the control parameters (actuators), a cost function is applied to minimize performance metrics with respect to a set of constraints. A cost function was applied to minimize the cross-track error, change in orientation, and the change in velocity from a reference value. Additionally, to speed the optimization process and provide useful results, each of these cost functions were weighted.

```
// The part of the cost based on the reference state.
for (int t = 0; t < N; t++) {
      fg[0] += c_cte*CppAD::pow(vars[cte_start + t] - ref_cte, 2);
      fg[0] += c_epsi*CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
      fg[0] += c_vel*CppAD::pow(vars[v_start + t] - ref_vel, 2);
}
```

The control parameters themselves were also minimized, shown here:

```
// Minimize the use of actuators.
for (int t = 0; t < N - 1; t++) {
      fg[0] += c_steering*CppAD::pow(vars[delta_start + t], 2);
      fg[0] += c_throttle*CppAD::pow(vars[a_start + t], 2);
}
```

Finally, each sequential control parameter value was minimized to smooth the resulting control inputs.

```
// Minimize the value gap between sequential actuations.
for (int t = 0; t < N - 2; t++) {
      fg[0] += c_s_seq*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += c_t_seq*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```

Note: Each of these functions penalize the vehicle for not maintaining the reference parameters with respect to constraints.

The references values for each of the cost functions can be seen below:
```
double ref_cte = 0.0;  // Cross-track error (CTE)
double ref_epsi = 0.0; //
double ref_vel = 36.0; // Reference velocity
```

The weights for each of the cost function were found through trial, error, and a bit of help from fellow Udacity students via slack. I've provided them below for reference:

```
const double c_cte = 0.4;
const double c_epsi = 0.32;
const double c_vel = 0.261;
const double c_throttle = 25.0;
const double c_steering = 300.0;
const double c_t_seq = 0.00001;
const double c_s_seq = 0.01;
```

### Constraints

In order to bound the optimization process (to always find a solution), constraints were placed on each of the state and actuator parameters. Upper and lower bound were globally placed on each of these values of 1e19 and -1e19, respectively. These values are placed globally to gaurantee numerical stability in the solver. Furthermore, the actuator constraints were also placed on the steering and acceleration (throttle) parameters to match the limitations of the vehicle. 

### Optimization Solver

Finally, the cost functions and constraints were fed into a non-linear optimization routine, Ipopt. The Ipopt routine is a standard non-linear optimization solver. The implementation of the solver can be seen below.

```
// solve the problem
CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
```

### Trajectory Length and Time-step Duration (N and dt)

In order to maintain computational efficiency, two parameters were implemented into the processing pipeline for determining the path length for the optimization code. These parameters N and dt represent the time step size (dt) and the number of time steps (N). The combination of these two parameters estimate the total length of the path for optimization through the state equations. 

After trial and error modifications, the final values of N and dt were as follows:

```
size_t N = 8;
double dt = 0.22;
```

### Polynominal Fitting

The opimization model uses waypoints from the Udacity simulator to find the approprate path to guide the vechicle autonomously. These way points are fed into the MPC code as point values and a standard polynomial fitting algorithm is applied to create a 3rd order line. The implamentation of the polyfit algorithm can be seen below.

```
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
```

This model is adapted from: // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716

### Model Predictive Control with Latency

In order to simulate real-world effects, a latency parameter was implemented in the data process flow that delays the time between determining a control parameter and implementing that control parameter on the vehicle. This delay simulates real world processing effects. The implementation of the latency parameter can be seen below.

```
this_thread::sleep_for(chrono::milliseconds(100));
```

## MPC Algorithm

The MPC algorithm follows the following process flow:

Initialization
1.	Define trajectory length (N) and the time-step duration (dt).
2.	Define the vehicle dynamics (state model) and the actuator limitations (constraints).
3.	Define the cost function.

Loop
1.	Set current state to the initial state in the MPC.
2.	Call optimization solver and minimize the cost function with respect to the initial state and way points.
3.	Apply actuator control parameters.

## Results
