# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Introduction

This projects implements a Model Predictive Control to actuate steering angle and throttle (acceleration) of a car driving around a track.

The controller is tested with Udacity Self-Driving Car Simulator.

There are many possibilities when designing a control algorithm, starting from simple PID controllers to complicated optimal control algorithms relying on the Maximum Principle or HJB equations.  Additionally, one could implement adaptive and robust control algorithms.

The MPC algorithm is an optimal control algorithm, that differs from classical maximum principle algorithms in that instead of solving an infinite dimensional optimization problem we discretize time and assume constant control on each time interval.  Hence, the control value on each time interval is in effect a new control variable.  This drastically increases the dimension of the control, and makes the problem into a `simple` finite dimensional optimization problem (with a large number of dimensions depending on the time-step).  Many finite-dimensional optimization algorithms are available.  The one used in this implementation is `ipopt`.

## Model Description

The state vector consists of the `x`,`y` position of the car, the orientation `psi` of the car as well as velocity, `v`, of the car.  These are augmented with the cross-track error, `cte`, and the orientation error, `epsi`.  This makes optimization with `ipopt` possible.  The controls are the steering angle `delta` and the throttle (pos/neg acceleration), `a`.

I used a slightly more general model than the one provided by Udacity, the difference being that in the equation for psi `tan(delta)` is used instead of its linear approximation, `delta`.  The model derivation is described here, https://gurgentus.github.io/Intro-Control-and-Estimation/Module1-Introduction.html.

Discretizing time and assuming constant control parameters during each time-step results in the following update equations:

```
x_{i+1} = x_i + v_i * cos(psi_i) * dt
y_{i+1} = y_i + v_i * sin(psi_i) * dt
psi_{i+1} = psi_i + v_i/Lf * tan(delta_i) * dt
v_{i+1} = v_i + a * dt
```

In addition, the cross-track error and orientation error satisfy

```
cte_{i+1} = f(x_{i+1}) - y_{i+1}
epsi_{i+1} = psi_{i+1} - tan(f'(x_{i+1}))
```

All of the dynamic equations are implemented in the code as constraints (see lines 87-94 of MPC.cpp).

## Cost Functional

One of the most important pieces of an optimal control algorithm is the cost functional.  Here I chose

```
L = p_1*cte^2 + p_2*epsi^2 + p_3*(v-ref_v)^2 + p_4*delta^2 + p_5*a^2 + p_6*(delta')^2 + p_7*(a')^2.
```

where the coefficients p_i were selected by experimentation and testing in the simulator.  Also, the derivatives were approximated with differences in the code.  Hence, not only are the errors minimized, but also the control inputs and their derivatives.

## Timestep Length and Elapsed Duration (N & dt)

To get best results one would have to choose small `dt` (smaller error) and hence larger `N` (to keep the same time into the future). As discussed before, this increases the dimensionality of the optimization problem and requires more computations.

Due to limitations of my machine I chose `N = 10`, and decreased `dt` until getting good results. (I started with N=25 and dt = 0.01).

On a faster machine `N` can be increased.

## Preprocessing and Fitting

I chose to do the trajectory fitting and optimization in vehicle coordinates. The coordinate transformation is done in `lines 108-116` of main.cpp.  Polynomial is fitted in `line 119`

Since fitting is done in new coordinates every time (since the car is moving) this introduces some instability, however I felt that the benefits of using a local coordinate systems were greater, especially if in the future using a local pathfinding algorithm.

## Latency

Latency is handled by putting a constraint on the controls for the first 100ms (or first two time-steps).  This is done in lines 186-193.  After 100ms the corresponding control is used, i.e. in `line 242` the controls after two time-steps are passed back from the solver.

## Future improvements

Due to limitations of my machine I couldn't test with higher speed as update computations were not being done in time introducing unpredictable lag.  I plan to experiment and improve the algorithm on a faster machine.

I also plan to use optimal control methods based on the maximum principle (e.g. LQR) to see if I can improve the computational cost by doing more of the analysis before-hand with the full infinite-dimensional problem.

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
