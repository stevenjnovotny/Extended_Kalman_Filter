# Extended Kalman Filter

Implementation of a kalman filter to estimate the state of a moving object with noisy lidar and radar measurements. The goal was to obtain a threshold RMSE values. 

Visulaiztion used the Udacity Unity-based Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

[//]: # (Image References)

[image1]: ./KFflow.jpg "Kalman Filter Flow"
[image2]: ./KF.jpg "Kalman Filter"
[image3]: ./EKF.jpg "Extended Kalman Filter"
[image4]: ./Ekf_simulation1.jpg "Extended Kalman Filter"

## Code Overview

The approach is based on the Kalman filter as descibed in the following flow diagram.

![alt text][image1]

The math for the regular Kalman filter is shown here:

![alt text][image2]

And the math for the Extened Kalman Filter is shown here:

![alt text][image3]

The simulation results are presented below.

![alt text][image4]