# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

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
Spent a ton of time on installing this on Mac and Ubuntu as well. Finally settled down with Ipopt 3.12.7 on Linux.

* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


### Model

The vehicle model used in this project is a kinematic bicycle model. It is under the assumption of negligible lateral weight shift, roll and compliance steer while traveling on a smooth road, which is pretty similar to the environment of out simulator.

### Timestep tunning

The final values I chose are predicting 8 points, with 0.65s in total.

I started with 10 steps at 0.1 seconds with which the car runs pretty well at low speed or straight lines. But when it reached the curvy road, it became extremely unstable and tried to correct with a big steering angle. It easily run out of the track and got lost. 

If the parameters was set too high, the vehicle would tend to drive too conservatively.

### Cost function tunning

In order to find the path with maintaining a set of expectation values of cte, epsi and v, a cost function is needed by applying different parameters to each part.

* Cross Track Error^2 * 1, reference cte = 0;
* Orientation Error^2 * 2, reference orientation = 0;
* Deviation from reference velocity^2 * 1, reference v = 60;
* Use of steering actuator^2 * 1
* Use of acceleration actuator^2 * 1
* Difference of sequential actuations for steering^2 * 20000
* Difference of sequential actuations for acceleration^2 * 10

### Polynomial Fitting and MPC processing

First, I transformed the waypoints into vehile space and used a cubic polynomial to fit the curve.

Then set that the initial position of the car and heading direction are always zero in this frame. Thus the state of the car in the vehicle cordinate system is

```state << 0, 0, 0, v, cte, epsi;```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

