# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

# Compiler Build and Run Instructions

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

---

# Model Predictive Controller
## Overview

![start](https://github.com/mblomquist/Udacity-SDCND-Term2/blob/master/P5-MPC/pics/start.JPG?raw=true)

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
- Steering Angle: [-25, 25]
- Acceleration (Throttle Parameter): [-1, 1]

The steering angle of the car is limited by a maximum 25 degree travel and the throttle is controlled to a value of +/- 1. At 1, the throttle represents the maximum forward acceleration (-1 represents the maximum reverse throttle).

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
double ref_epsi = 0.0; //  Orientation Error
double ref_vel = 10.0; // Reference velocity
```

The weights for each of the cost function were found through trial, error, and a bit of help from fellow Udacity students via slack. I've provided them below for reference:

```
const double c_cte = 0.35; // Weight of CTE Correction
const double c_epsi = 10.0; // Weight of EPSI Correction
const double c_vel = 1.0; // Weight of Velocity Correction
const double c_throttle = 10.0; // Minimize Throttle Commands
const double c_steering = 46.0; // Minimize Steering Commands
const double c_t_seq = 1.0; // Minimize between Throttle Commands
const double c_s_seq = 450.0; // Minimize between Steering Commands
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
// Set latency value
double latency = 0.100; // 100 milliseconds

// Predict future state with kinematic model and latency (100 milliseconds)
double px_t1 = v * latency;
double py_t1 = 0.0;
double psi_t1 = (v / 2.67) * (-1 * delta) * latency;
double v_t1 = v + alpha * latency;
double cte_t1 = cte + v * sin(epsi) * latency;
double epsi_t1 = epsi + (v / 2.67) * (-1 * delta) * latency;
```

To account for latency, the future state of the vehicle was predicted using a value of 100 milliseconds (latency value). Additionally, predictions were made for the CTE and EPSI after 100 milliseconds by adding the current CTE and EPSI with the kinematic equations.

Note: The initial values for x, y, and psi were 0 as they represent the current location of the vehicle.


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

The MPC model ended up performing quite well after tuning the cost weights and time-step parameters. In straight-aways, the model performed in a similar way to the previous PID controller project. However, it is important to note that the MPC controller had to over come 100 milliseconds of latency (the PID controller did not).

![mid](https://github.com/mblomquist/Udacity-SDCND-Term2/blob/master/P5-MPC/pics/mid.JPG?raw=true)

Without taking into consideration the latency of the system, the vehicle wasn't able to safely make it around the track. There were a few areas where the car slightly went off the drivable portion of the road.

![wild1](https://github.com/mblomquist/Udacity-SDCND-Term2/blob/master/P5-MPC/pics/wild1.JPG?raw=true)

![wild2](https://github.com/mblomquist/Udacity-SDCND-Term2/blob/master/P5-MPC/pics/wild2.JPG?raw=true)

Once latency compensation was added to the MPC and the model was re-tuned, the vehicle was able to safely make it around the track without any problems.

![safe1](https://github.com/mblomquist/Udacity-SDCND-Term2/blob/master/P5-MPC/pics/safe1.JPG?raw=true)

![safe2](https://github.com/mblomquist/Udacity-SDCND-Term2/blob/master/P5-MPC/pics/safe2.JPG?raw=true)
