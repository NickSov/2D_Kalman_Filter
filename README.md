# Extended Kalman Filter Project
**Part of the Self-Driving Car Engineer Nanodegree Program**

Given a provide framework, an Extended Kalman Filter (EKF) was developed in order to estimate position and velocity from noisy lidar and radar measurements. The filter is able to use the various types of measurements interchangeably and to effective provide for sensor fusion.

In order to verify the performance of the filter, a root-mean-square error (RMSE) was used to determine the difference between ground truth values and the predict state. Furthermore, a simulator was used to visualize the performance of the filter by plotting the measured sensor positions and the predicted ones.

**Reduction of the RMSE value with increased number of measurements:**

![alt text](https://github.com/NickSov/2D_Kalman_Filter/blob/master/images/RMSE_Result.png)


**Successful example of the simulator (simulator can be found here:** https://github.com/udacity/self-driving-car-sim/releases):

![alt text](https://github.com/NickSov/2D_Kalman_Filter/blob/master/images/Simulation_Result.png)
