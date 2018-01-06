# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
# Answer to Rubric Questions

### 1. Model Explanation in Details

* State
    - car location: x, y, 
    - car orientation: psi, 
    - car velocity: v,
    - errors to the central point in the path: cte,
    - errors or car orientations: epsi.
   
* actuators
    - steer_value: the degree of steering,
    - throttle_value: the acceleration (positive) and break (negative).
     
* Update Equations:
    The equations are updated as follows (MPC.cpp line 121-128): 
    ```
    fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
    fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
    fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
    fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
    fg[1 + cte_start + t] = cte1 - ((ref0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
    fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);    
    ```
    
* Cost function: The cost function is defined in lines 48-67 in MPC.cpp as follows.
It utilize factors of cte, epsi, velocity, steering (delta), acceleration (a),
and the smoothness of the steering and acceleraton. Here a trick for the cte and epsi
factors, I assigned more weights to the remote points (future points).

```
fg[0] = 0;
// The part of the cost based on the reference state.
for( int i = 0; i < N; i++ ) {
     fg[0] += (cte_weight + i*2)*CppAD::pow(vars[cte_start + i], 2);
     fg[0] += (epsi_weight + i*2)*CppAD::pow(vars[epsi_start + i], 2);
     fg[0] += v_weight*CppAD::pow(vars[v_start + i] - v_ref, 2);
}

// Minimize the use of actuators.
for (int i = 0; i< N - 1; i++) {
    fg[0] += delta_weight*CppAD::pow(vars[delta_start + i], 2);
    fg[0] += a_weight*CppAD::pow(vars[a_start + i], 2);
    }

// Smoothing sequential actuations.
for (int i = 0; i < N - 2; i++) {
    fg[0] += smooth_delta_weight*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
    fg[0] += smooth_a_weight*CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }
```

Each factors are weighted correspondingly, and the weights are defined in MPC.cpp lines 26-32.
The weight values and the reason of using such a factor are as follows: 

```
const double cte_weight = 10; //minimize the cte
const double epsi_weight = 10; //minimize the orientation errors
const double v_weight = 1; //try to match to the target velocity
const double delta_weight = 1; //try to reduce the wheeler control angle
const double a_weight = 1; //try to avoid too much acceleration/break
const double smooth_delta_weight = 1e5; //weight to minimize wheeler control changes;
const double smooth_a_weight = 10; //minimize the acceleration changes
```


### 2. The Choice of N and dt
* The idea to choice N and dt: I tried with various pair of N and dt, and the values 
that I finally chose are N=20, and dt = 0.03 (or 30 ms).

  - The larger of N, the optimization yeilds a more accurate control, but get 
less controllability, especially this could be problem for sharp turns. Recall that
there are two continous sharp turns (left, and then right) very close to each 
other. In addition, larger N incurs more computation costs. 

  - The dt is the step size of the prediction. The smaller the dt, the higher the 
  resolution of prediction. Also like N, too high resolution (smaller value of dt)
  will incur the computational burden.

* History of values tested: 
  - Besides the final choice of N=20, I also tried N to be large like 100, 50, but it appears hard 
to take turns.
  - I tried the dt to be small as 0.01, or larger 0.1. I finally choose dt=0.03 for 
  better performance.


### 3. Waypoints Fitting

The MPC predicted points are then fitted with 3rd order polynomial coefficient and evaluated as follows.
This is in the main.cpp lines 172-184: 

```
vector<double> next_x_vals;
vector<double> next_y_vals;

int n_next_points = 30;
int step_size =3;
for (int i = 0; i < n_next_points; i ++){
    double x = i * step_size;
    next_x_vals.push_back(x);
    next_y_vals.push_back(polyeval(coeffs, x));
    }
```

### 4. Latency Handling 

The 100ms latency is handled by updating the current state with future state in 100ms later. The latency is first 
defined in code as (main.cpp, line 124):

```python
const double latency = 0.100; // 100 ms
```
The latency is then applied as "dt", the state with latency is then predicted as used (main.cpp lines 132-137):

```
//Predicted state based on 100ms latency
double x1 = x0 + ( v * cos(psi0) * latency );
double y1 = y0 + ( v * sin(psi0) * latency );
double psi1 = psi0 - ( v * steer_value / Lf * latency);
double v1 = v + throttle_value * latency;
double cte1 = cte0 + ( v * sin(epsi0) * latency );
double epsi1 = epsi0 - ( v * atan(coeffs[1]) * latency / Lf );

// Update the state vector with latency considerated
state << x1, y1, psi1, v1, cte1, epsi1;

```
   

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
