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
```
      x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      v_[t+1] = v[t] + a[t] * dt
      cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
 ```
 
 Symbol|Expalaination     
 ---   | --- 
 x, y  | position of the car in a global coordinate system                      
 psi   |heading value                                                           
 v     | speed of the vehile                                                     
 cte   | cross-track error                                                      
 epsi  | orientation error                                                      
 Lf    | distance between the center of mass of the vehicle and the front wheels 

### Timestep tunning

The final values I chose are predicting 20 points, with 800ms delay for each step.

I started with 10 steps at 0.1 seconds with which the car runs pretty well at low speed or straight lines. But when it reached the curvy road, it became extremely unstable and tried to correct with a crazy steering angle. It easily run out of the track and got lost. 

For a smaller dt, the step space would be smaller and you will get a smoother curve with reasonable jerk, which is good for future control and a better response of acuators.

I also tried to increase N to 30, which gave a bad result. As the prediction steps getting bigger, the solver needs more computation time.

The time horizon bascially tells us the prediction spaces ahead if a speed is given. 

### Cost function tunning

In order to find the path with maintaining a set of expectation values of cte, epsi and v, a cost function is needed by applying different parameters to each part.

* Cross Track Error^2 * 1, reference cte = 10;
* Orientation Error^2 * 2, reference orientation = 10;
* Deviation from reference velocity^2 * 1, reference v = 60;
* Use of steering actuator^2 * 2000
* Use of acceleration actuator^2 * 100
* Difference of sequential actuations for steering^2 * 20000
* Difference of sequential actuations for acceleration^2 * 1

### Polynomial Fitting and MPC processing

First, I transformed the waypoints into vehile space and used a cubic polynomial to fit the curve.

Then set that the initial position of the car and heading direction are always zero in this frame. Thus the initial state of the car in the vehicle cordinate system is

```state << 0, 0, 0, v, cte, epsi;```

### Model Predictive Control with Latency

Just like other applications, this MPC project has to take latency (processing time and actuators reponse time) into account. I set the dt = 80 ms, all of the state would change during this period of time.

```
//  change of sign because turning left is negative sign in simulator but positive yaw for MPC
psi = 0; // in coordinate now, you can also use psi = delta
px = px + v*cos(psi)*dt; 
py = py + v*sin(psi)*dt;
cte= cte + v*sin(epsi)*dt;
epsi = epsi + v*delta*dt/Lf;
psi = psi + v*delta*dt/Lf;
v = v + a*dt;
```
Then we pass this updated vector back to the MPC solver. The MPC will give us the throttle and steering angle outputs.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

