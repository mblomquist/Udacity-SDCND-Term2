# Review 2

## Status - Meets Specifications

You put a lot of effort into this project and I think it was worth it because you must have learnt a lot from it. Congratulations! 

Keep up the good work for the next term.

## Compilation 

### Requirement

Code must compile without errors with cmake and make.

Given that we've made CMakeLists.txt as general as possible, it's recommend that you do not change it unless you can guarantee that your changes will still compile on any platform.

### Comments

I changed the linear_solver setting, because I have another solver installed. With this change the code could be compiled without any errors. Great job.

## Implementation

### Requirement

Student describes their model in detail. This includes the state, actuators and update equations.

### Comments

The details of the model (including the states, actuators and update equations) is described well in the README.md. Nice work.

### Requirement

Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

### Comments

You chose the values for N and dt empirically. In engineering the trial and error approach is an often used technique. Nice work.

* N = 8
* dt = 0.22

### Requirement

A polynomial is fitted to waypoints.

If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

### Comments

A 3rd order polynomial is fitted to the waypoints after the preprocessing step (transforming points to vehicle frame). Well done.

You might want to provide more details about this transformation.

### Requirement 

The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

### Comments

You described how you coped with latency (you projected the state for the duration of latency and used this as the starting state) and the simulation shows that it works well. Awesome.

## Simulation

### Requirement

No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

### Comments

Great job! With an average speed of ~20 MPH the car drives itself safely. I let it run for several laps. Sometimes the car went close to the side of the road, but never left the driveable portion of the road.

However you might want to enhance the project so that the vehicle could drive itself faster. With constant speed it is possible to drive safely up to 70mph, if the speed can depend on the strenght of the curve, it is possible to exceed the speed of 100mph.

![review2_img1](https://github.com/mblomquist/Udacity-SDCND-Term2/blob/master/P5-MPC/pics/review2_img1.png?raw=true)

---

# Review 1

## Status - Requires Changes (2 Specifications require changes)

You are nearly there! Your controller already steers the car around the track but sometimes leaves the safe portion of the road.

In order to improve your solution, I would ask you to:

- Implement latency compensation by predicting the vehicle state into the future before passing it to the solver.
- Adjust the weights of your cost function so that the car keeps to the center of the road.
I hope that I have given you enough information to successfully improve your implementation. If you get stuck, contact me on slack @wsteiner and I will try to further assist you in getting through this demanding project.

## Compilation 

### Requirement

Code must compile without errors with cmake and make.

Given that we've made CMakeLists.txt as general as possible, it's recommend that you do not change it unless you can guarantee that your changes will still compile on any platform.

### Comments

Your code builds without any errors or warnings!

## Implementation

### Requirement

Student describes their model in detail. This includes the state, actuators and update equations.

### Comments

You clearly describe your model in the included report!

### Requirement

Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

### Comments

You clearly describe how you arrived at your choice of N and dt.

### Requirement

A polynomial is fitted to waypoints.

If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

### Comments

You convert the waypoint coordinates from global to vehicle coordinates before you fit the 3rd order polynomial.

### Requirement 

The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

### Comments

I would ask you to implement latency compensation by predicting the vehicle state 100ms into the future before passing it to the solver. You can do this by applying the following equations.

![review1_img1](https://github.com/mblomquist/Udacity-SDCND-Term2/blob/master/P5-MPC/pics/review1_img1.png?raw=true)

Some tips:

- Convert the velocity to m/s.
- Implement all of the model update equations as shown above.
- Pay attention to the sign of the steering angle.
- The yaw angle psi_t is zero after you have transformed the problem into vehicle coordinates.
- You can get the current steering angle in radians from the simulator: double delta= j[1]["steering_angle"].
- You could try a longer latency time like 125ms in order to account for the processing time of the solver and the latency of the communication with the simulator.

## Simulation

### Requirement

No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

### Comments

Your controller already steers the car around the track. Unfortunately it sometimes leaves the safe portion of the road:

![review1_img2](https://github.com/mblomquist/Udacity-SDCND-Term2/blob/master/P5-MPC/pics/review1_img2.png?raw=true)
![review1_img3](https://github.com/mblomquist/Udacity-SDCND-Term2/blob/master/P5-MPC/pics/review1_img3.png?raw=true)
