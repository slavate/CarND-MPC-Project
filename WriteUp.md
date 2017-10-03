# Write-Up for Project CarND-MPC 
Self-Driving Car Engineer Nanodegree Program, Term 2

---
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it

## Vehicle Model
I choosed the model represented in the class with the following state vector:
```
state = [x, y, psi, v]
```

Where `x` and `y` are coordinates, `psi` is the orientation and `v` is velocity of the vehicle in the world. 
Moreover actuators (control) are throttle (acceleration and bracking) and steering angle, which we want to estimate with MPC. Further inputs to the model predictive controller are cross track error (`cte`) and orientation error (`epsi`).

Update equations:

      x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      v_[t] = v[t-1] + a[t-1] * dt
      cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt

Here we simplify that actuator throttle value in range from -1 to 1 is acceleration which will be used in the above update equation for velocity.

As proposed in Q&A Video I also shifting vehicle reference point to `0,0` and orientation `0` - see lines 110 - 120 in main.cpp.

## Timestep Length and Elapsed Duration (N & dt)
At first I used proposed values in the Q&A video, i.e. `N = 10` and `dt = 0.1`. This values seemed for me to be appropriate, we have 10 points for polynomial fitting and predicting 1 second, which is not a lot. At the end I tried to use following combinations: 

* `N = 20` and `dt = 0.1`
* `N = 20` and `dt = 0.05`
* `N = 10` and `dt = 0.05`

All of them are failed, I believe because weights for cost function components which I defined (tuned) manually are good only for `N = 10` and `dt = 0.1`.

## Cost function
Additionally to proposed elements in Q&A/Class for the cost function, I added element which penalizes high velocity with high steering angle, see line 69 in MPC.cpp. All the coefficients where tuned manually based on the suggestions from class. I believe/hope there is way to tune them automatically. Why not to use them as input parameters for ipopt?


