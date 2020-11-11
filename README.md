# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program
---
## Description

Implementation of the Extended Kalman Filter based on sensor data from both LIDAR and RADAR measurements for estimatetion of the state of a moving object.
The project starter code was given [here](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) and it was completely written in the object oriented programming style. 
However, whole source code changed to Functional programming style in order to make it:
* More re-usable
* More testable (unit test)
All initial given source codes were modified, except json.hpp file. The main algorithm is exactly same, but its implementation is different due to FP style.

---
## Project Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* boost.test >= 1.45 (optional)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `
5. Click on the "Simulator" button in the bottom of the Udacity workspace, which will open a new virtual desktop. You should see a "Simulator" link on the virtual desktop. Double-click the "Simulator" link in that desktop to start the simulator.

## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Unit test

Unit test's performed using [Boost Test Library](https://www.boost.org/doc/libs/1_45_0/libs/test/doc/html/index.html). Example simple test cases are created in test_case.cpp file. It can be compiled using Boost.UTF library, but it is not mandatory for the project.
It was used to carry out the unit test for Predict() and Update() functions.

## Project Rubric (RMSE)

The following numbers are final results for each case and as shown below, it satisfied requirements of the project rubric.

### Dataset 1 

Data from both Lidar and Radar.
* X	: 0.0975
* Y	: 0.0855
* VX	: 0.4460
* VY	: 0.4455

### Dataset 2

Data from both Lidar and Radar.
* X	: 0.0726
* Y	: 0.0967
* VX	: 0.4518
* VY	: 0.4839

For Dataset 1, additional expirements have been perfored by disabling Lidar or Radar measurements. When one of sensors are enabled, its RMSE accuracy dropped and Radar's accuracy was lower than Lidar's.
Data from only Lidar.
* X	: 0.1474
* Y	: 0.1154
* VX	: 0.6294
* VY	: 0.5346
Data from only Radar.
* X	: 0.2296
* Y	: 0.3467
* VX	: 0.5743
* VY	: 0.8054
