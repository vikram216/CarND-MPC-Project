# Overview

The goal of this project is to implement Model Predictive Control (MPC) and to drive a vehicle around a simulated road track. A cross track error (CTE) is calculated in the project code. Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

This solution makes use of the IPOPT and CPPAD libraries to calculate an optimal trajectory and its associated actuation commands in order to minimize error with a third-degree polynomial fit to the given waypoints. The optimization considers only a short duration's worth of waypoints, and produces a trajectory for that duration based upon a model of the vehicle's kinematics and a cost function based mostly on the vehicle's cross-track error (roughly the distance from the track waypoints) and orientation angle error, with other cost factors included to improve performance.

## Model

The implemented MPC is using a global kinematic model to calculate a predicted vehicle trajectory. In the project the kinematic model receives a vehicle ```state``` as a vector of 6 elements:

* vehicle position coordinates ```x``` and ```y```
* vehicle orientation ```psi```
* vehicle velocity ```v```
* vehicle cross track error ```cte``` and
* vehicle orientation error ```epsi```

The first four state vector elements at a timestep ```t``` are provided by the vehicle simulator:

* current vehicle position coordinates ```x[t]``` and ```y[t]```
* vehicle orientation ```psi[t]``` in radians
* vehicle velocity ```v[t]``` in mph

The other two state vector elements, ```cte[t]``` and ```epsi[t]```, are calculated.

The cross track error ```cte[t]``` is calculated as a error of the vehicle position in ```y``` direction by evaluating the vehicle position at ```x``` and substracting the ```y``` coordinate. The simulator provides 6 track waypoints at any timestep ```t``` as 2 vectors ```ptsx``` and ```ptsy``` in global map coordinate system. In order to be evaluated against the actual vehicle position the track waypoints coordinates are transformed from global coordinates to the vehicle coordinate system. This is done by shifting the center of the coordinate system to the vehicle position and rotating it (counterclockwise) ```-psi``` radians to get zero orientation angle in ```x``` direction:

```
double shift_x = ptsx[i] - npx;
double shift_y =  ptsy[i] - npy;
waypoints_x[i] = (shift_x * cos(0-npsi) - shift_y * sin(0-npsi));
waypoints_y[i] = (shift_x * sin(0-npsi) + shift_y * cos(0-npsi));
```

The transformed track waypoints are used to build a 3rd order polynomial ```f(x)``` by polyfit method. The resulting polymomial coeffitients are evaluated against the vehicle ```y``` position by ```polyeval``` method to calculate the cross track error ```cte```. Since the vehicle is located in the center of the coordinate system, ```x``` and ```y``` are ```0```, and 
```cte = polyeval(coeffs, 0);```.

The vehicle orientation error ```epsi``` is calculated as a difference between the actual vehicle orientation angle ```psi``` and a desired orientation. The desired orientation is an angle of the tangent line to the 3rd order polynomial curve ```f(x)``` or ```arctan(f'(x))```, where ```f'(x)``` is a derivative of the polinomial ```f```. Since in the center of the coordinate system ```x = 0```, the desired orientation is calculated ```atan(f'(x)) = atan(coeffs[1])```. Also after the transformation of the coordinates the vehicle is heading in ```x``` direction and the orientation ```psi = 0```, so ```epsi = psi - atan(f'(x)) = -atan(coeff[1])```.

The resulting state vector provided to the MPC is ```[0, 0, 0, v, cte, epsi]```. The MPC is using an optimizer to calculate the vehicle control inputs (actuators), steering angle ```delta``` and acceleration ```a```, and to minimize the cost function.

The kinematic model is using following equations to update the vehicle state after ```a``` timestep ```dt```:

```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
The model is using 2 constraints as the actuator limitations of the steering angle ```delta``` and acceleration ```a```, which are defined as the lower and upper boundaries. Such as the steering angle boundaries are from ```-25``` to ```25``` degrees. The acceleration boundaries are from ```-1``` (full brake) to ```1``` (full acceleration).

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

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
