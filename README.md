# Extended Kalman Filter Project
**Self-Driving Car Engineer Nanodegree Program**

Given a provided framework, an Extended Kalman Filter (EKF) was developed in order to estimate position and velocity from noisy lidar and radar measurements. The filter is able to use the various types of measurements interchangeably and to effectively provide for sensor fusion. Some challenges of building the filter included account for the different coordinate systems that each sensor type uses. The lidar sensor is in the cartesian coordinate system whereas the radar is the polar coordinate space. The differences in the systems is shown in the image below:

![alt text](https://github.com/NickSov/2D_Kalman_Filter/blob/master/images/frame_of_reference.png)

**Reduction of the RMSE value with increased number of measurements:**

In order to verify the performance of the filter, a root-mean-square error (RMSE) was used to determine the difference between ground truth values and the predict stated. Furthermore, a simulator was used to visualize the performance of the filter by plotting the measured sensor positions and the predicted ones.

**Reduction of the RMSE value with increased number of measurements:**

![alt text](https://github.com/NickSov/2D_Kalman_Filter/blob/master/images/RMSE_Result.png)


**Successful example of the simulator (simulator can be found here:** https://github.com/udacity/self-driving-car-sim/releases):

![alt text](https://github.com/NickSov/2D_Kalman_Filter/blob/master/images/Simulation_Result.png)
