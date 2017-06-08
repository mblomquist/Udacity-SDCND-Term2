# Model Predictive Controller
## Overview

## Implementation
### Model

State: [x, y, psi, v]

Where:
- x is the global x position of the vehicle.
- y is the global y position of the vehicle.
- psi is the orientation of the vehicle in radians.
- v is the velocity magnitude of the vehicle.

Actuators: [delta, alpha]

Where: 
- delta is the steering angle
- alpha is the throttle control parameter (throttle/brake)

State Equations:
- x_t+1 = x_t + v_t * cos(psi_t) * dt
- y_t+1 = y_t + v_t * sin(psi_t) * dt
- psi_t+1 = psi_t + v_t / L_f * delta_t * dt
- v_t+1 = v_t + alpha_t * dt

Note: L_f measures the distance between the front of the vehicle and its center of gravity. The larger the vehicle, the slower the turn rate.

Actuator Constraints:
- Steering Angle: [-30, 30] 
- Acceleration (Throttle Parameter): [-1, 1]

Cost Functions:

A simple solution is to capture the velocity error in the cost function. This will penalize the vehicle for not maintaining the reference velocity.

 - cost += pow(v[t] - 35, 2) // (Assuming 35 mph is the speed limit)
 - cost += pow(delta[t], 2)
 
 ```
 for (int t = 0; t < N-1; t++) {
  cost += pow(delta[t+1] - delta[t], 2)
  cost += pow(a[t+1] - a[t], 2)
}
```

### Trajectory Length and Time-step Duration (N and dt)

### Polynominal Fitting and MPC Preprocessing

### Model Predictive Control with Latency

#### MPC Algorithm
Initialization
1.	Define trajectory length (N) and the time-step duration (dt).
2.	Define the vehicle dynamics (state model) and the actuator limitations (constraints).
3.	Define the cost function.

Loop
1.	Set current state to the initial state in the MPC.
2.	Call optimization solver and minimize the cost function with respect to the initial state and way points.
3.	Apply actuator control parameters.

## Results
