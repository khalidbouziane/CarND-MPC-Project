# CarND-MPC-P5
Self-Driving Car Engineer Nanodegree Program Moldel Predictive Control project submission

---

## The Project

This project consisted of implementing a Model Predictive Controller that allows the vehicle in the simulator to traverse the track. The model accounts for introduced latency to simulate a realistic scenario and computes the best actuator values for each time step using the measured state.

* Calculating the best fit polynomial for the defined waypoints.
* Defining the current vehicle state using measured values and computing the error terms.
* Calculating actuator values based on current state using MPC.
* Sending the computed parameters to the vehicle.

### Model
We get certain parameters from the simulator including:

* ptsx: x position of waypoints in map coordinates
* ptsy: y position of waypoints in map coordinates
* px: current position of the vehicle
* py: current y position of the vehicle
* psi: current orientation angle of the vehicle
* v: velocity of the vehicle
* delta: current steering angle of the vehicle
* a: current throttle

We used this information as described in the following sections basing our prediction on the vehicle model. The equations for predicting the state of the vehicle are given by the following equations:

* x_t+1 = x_t + v_t * cos(psi_t) * dt
* y_t+1 = y_t + v_t * sin(psi_t) * dt
* psi_t+1 = psi_t + (v_t/ Lf) * delta * dt
* v_t+1 = v_t + a_t * dt
* cte_t+1 = cte_t + (v_t * sin(epsi_t) * dt)
* epsi_t+1 = epsi_t + (v_t/ Lf) * delta * dt

Where Lf is the distance from the from of the vehicle to its center of gravity, and cte and epsi are defined as:

* cte = f(x_t) - y_t
* epsi = psi_t - arctan(f'(x_t))

An important consideration is that we need to add a negative sign to the psi value that we get to the simulator as it is inverted to the defined psi from our model.

### Preprocessing
The first step after obtaining the parameters from the simulator is to transform the x and y
waypoints from map coordinates to vehicle coordinates. This is accomplished with a coordinate
transformation assuming that the vehicle is at the origin. The x and y position and psi will be then 0 and the waypoints will be in reference to this position. The equations for the transforms are:

* shift_x = ptsx[i]-px;
* shift_y = ptsy[i]-py;
* ptsx[i] = shift_x*cos(0-psi) - shift_y*sin(0-psi);
* ptsy[i] = shift_x*sin(0-psi) + shift_y*cos(0-psi);

Using the transformed waypoints we can now calculate a polynomial fit for the points to obtain a third degree polynomial coefficients. We can use this polynomial to calculate our errors cte and epsi. We assume that cte is the y distance to from the vehicle, meaning that we only need to evaluate the polynomial at x = 0 and the use atan(f'()) simplified to the atan(coeff[1]) for psi. This yield a complete state vector for the vehicle that includes x position, y position, psi, velocity, cte, and epsi.

### Latency
Latency is added to account for a more realistic scenario and it's therefore important to incorporate this into our MPC calculations. To mitigate the effects of latency we can predict the state of the vehicle at the time of latency and then feed this state to our MPC. Using the motion model equations we can calculate the value for each of the state variables at a time t + latency (dt). This new values will be used to compute our actuator values in the MPC solver.

### Model Predictive Control Calculations
At this point we have now a reference state of the vehicle at time t + latency and the coefficients for our trajectory polynomial. We define N = 10 steps for the trajectory with a time step of 100 ms for our prediction meaning that we are predicting 1 second into the future. I tried different values like 0.15 and 0.2 for the time step but that results in a path that too much into future and add additional latency.

In MPC::Solve() we'll set the variable limits and constraints for the optimizer. We need to define the limits for the actuator based on the max steering angle or 25 degrees in radians and the throttle to between -1 and 1. We then set the constraints to be defined by the initial state.

We can now pass this values to our optimizer that uses our defined cost and motion model to optimize our desired result. We set the cost for FG_eval based on our errors cte and epsi as well as our desired speed. I also add cost to minimize the use of actuators and the gap between sequential actuations to produce a more stable motion of the vehicle. We can tune our controller by adding a factor to all weights to increase or reduce the impact of each of the cost parameters. We compute the rest of the constraints using our motion model and calculating at our defined time step for the defined number of points, in this case 10*0.1 or 1 second each using state variables at time t and t+1.

After the solver finishes, we will get a vector containing our predicted variables that can then be used to set the actuations and later passed to our simulator. I return a vector containing the steering angle and throttle along with x and y positions to display on the simulator.

### Sending Prediction to simulator
We use the result vector that contains the actuator values, normalizing the steering angle to be between -25 degrees instead of radians. We then pass the x and y values in order to trace the green line showing the predicted trajectory. The final step is to pass the reference trajectory using our fitted polynomial. The result is that for every step we get the actuator values and trajectories for the vehicle to move throughout the track.

Here are some examples of the vehicle in a sample run:

![Sample Run 1 in Simulator](outputs/mpc1.gif)

![Sample Run 2 in Simulator](outputs/mpc2.gif)

![Sample Run 2 in Simulator](outputs/mpc3.gif)


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.
# CarND-MPC-Project
# CarND-MPC-Project
