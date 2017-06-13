# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.
# How to run the program
1. Clone this repo
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF
6. Run the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

# Result
Here is the RMSE result for dataset 1
![Simulator output](/output_images/sim_dataset1.png)
I also graph the datatset 1 using python:
![Dataset1 with update](/output_images/dataset_1.png)

Here is the RMSE result for dataset 2
![Simulator output](/output_images/sim_dataset2.png)
I also graph the datatset 2 using python:
![Dataset2 with update](/output_images/dataset_2.png)


## Other Important Dependencies

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