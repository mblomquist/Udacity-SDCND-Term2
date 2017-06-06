# Udacity SDCND Term 2 - PID Controller Project

## Project Description

In this project, a PID controller was designed and implemented in C++ to drive the car from the Udactiy Simulator around a track autonomously using the cross-track error (CTE).

## Determining the PID Control Parameters

The PID control parameters were determined using trial and error. To expidite the process, I ended up using starting with PID values that had shown sucess with other students. These parameter values were taken from the carnd slack channel. Using the initial parameters without modification, however, resulted in the car immediately driving off-of the road. To correct this issue, I ended up reducing all of the parameters by approximately a factor of 2. This resulted in smoother dring and the car managed to make it further along the track.

![start](https://github.com/mblomquist/Udacity-SDCND-Term2/blob/master/P4-PID/pics/pid_start.JPG?raw=true)

From that point, I spend time turning parameters as needed to reduce the oscillations around the center of the track while driving. The general method I used for tuning the parameters is described below by the effect of P, I, and D. Here are the initialized values:

```
P = 0.255
I = 0.004
D = 4.000
```

### Tuning the P parameter

I always began turning the PID parameters with the proportional parameter. I believe this methodology comes from my background in mechanical engineering. Simply put, I would modify the propotional control parameter such that I started getting consistent oscillations around the center of the track. At this point, I would move onto the D parameter. In the end, I ended up with a propotional value of 0.255.

### Tuning the D parameter

The dirivitive parameter in the PID controller would be modified next. I would typically try to choose a value, after choosing P, such that I minimize the oscillations around the center of the track. The dirivitative term is a prediction-style term that can reduce overshoot by minimizing the impact of the proportional parameter near steady-state conditions. In the end, I ended up with a proportional value of 4.0.

![lake](https://github.com/mblomquist/Udacity-SDCND-Term2/blob/master/P4-PID/pics/pid_lake.JPG?raw=true)

### Tuning the I parameter

Tuning the I parameter of the PID controller comes last and is primarily used to reduce the overall steady-state error. This parameter can be a bit tricky, but with some help choosing the starting values (from the fellow classmates on slack), I started near the ideal position. In the end, I stuck with a parameter value of 0.004.

### Error Initialization

One of the initial difficulties I had during this project was based around initializing the PID controller error to 0. This intuitively makes sense, but was one of the more difficult debugging procedures as I wasn't sure why my controller was performing so aweful. Remember to initialize!
