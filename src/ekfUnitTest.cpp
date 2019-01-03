// Unit test for the Kalman Filter program

#include "kalman_filter.h"
#include "tools.h"
#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <string>

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main()
{

  int incr = 200;

  //// Read in measurement and groundtruth values for radar and lidar ////

  // Create matrices and variables used for reading data and creating matrices

  Eigen::MatrixXd laserDataRead;
  laserDataRead = MatrixXd(250,7);

  Eigen::MatrixXd radarDataRead;
  radarDataRead = MatrixXd(250,8);

  std::string line;
  std::string type;
  double px_r;
  double py_r;
  long long time_r;
  double px_gt_r;
  double py_gt_r;
  double vx_gt_r;
  double vy_gt_r;
  double y_gt_r;
  double yr_gt_r;
  double rho_r;
  double phi_r;
  double rhodot_r;
  int intLaser = 0;
  int intRadar = 0;

  std::ifstream dataFile ("dataFile.txt");
  std::ofstream dataFileOut;
  dataFileOut.open("RMSE_Out.txt", ios::trunc);

  if (dataFile.is_open())
  {
    for (int i=0; i<500; i++)
    {
      dataFile >> type;
      if (type == "L")
      {

        // Read in lidar values

        dataFile >> px_r >> py_r >> time_r >> px_gt_r >> py_gt_r >> vx_gt_r >> vy_gt_r >> y_gt_r >> yr_gt_r;
        laserDataRead(intLaser, 0) = px_r;
        laserDataRead(intLaser, 1) = py_r;
        laserDataRead(intLaser, 2) = time_r;
        laserDataRead(intLaser, 3) = px_gt_r;
        laserDataRead(intLaser, 4) = py_gt_r;
        laserDataRead(intLaser, 5) = vx_gt_r;
        laserDataRead(intLaser, 6) = vy_gt_r;
        intLaser++;

      } else {

        // Read in radar values

        dataFile >> rho_r >> phi_r >> rhodot_r >> time_r >> px_gt_r >> py_gt_r >> vx_gt_r >> vy_gt_r >> y_gt_r >> yr_gt_r;

        radarDataRead(intRadar, 0) = rho_r;
        radarDataRead(intRadar, 1) = phi_r;
        radarDataRead(intRadar, 2) = rhodot_r;
        radarDataRead(intRadar, 3) = time_r;
        radarDataRead(intRadar, 4) = px_gt_r;
        radarDataRead(intRadar, 5) = py_gt_r;
        radarDataRead(intRadar, 6) = vx_gt_r;
        radarDataRead(intRadar, 7) = vy_gt_r;

        intRadar++;
      }
    }
  }

  dataFile.close();

  // print out matrices with values

  std::cout << "LASER MATRIX" << std::endl;
  std::cout << laserDataRead << std::endl;
  std::cout << "RADAR MATRIX" << std::endl;
  std::cout << radarDataRead << std::endl;

//// Initialization state ////

bool is_initialized_ = false;

//// RMSE Vectors ////

vector<VectorXd> estimations;
vector<VectorXd> ground_truth;

//// Test values vectors ////

int measType = 0;

// 0 - lidar
// 1 - radar

//// Laser data /////

Eigen::MatrixXd laserData;
laserData = MatrixXd(5,7);

if (measType == 1){

  std::cout << "Using laser data..." << std::endl;
  std::cout << " " << std::endl;

  laserData << 3.122427e-01, 5.803398e-01, 1477010443000000,6.000000e-01,	6.000000e-01,	5.199937e+00,	0,
              1.173848e+00, 4.810729e-01,	1477010443100000,1.119984e+00,	6.002246e-01,	5.199429e+00,	5.389957e-03,
              1.650626e+00,	6.246904e-01,	1477010443200000,1.639904e+00,	6.013473e-01,	5.198392e+00,	1.795970e-02,
              2.188824e+00,	6.487392e-01,	1477010443300000,2.159704e+00,	6.040855e-01,	5.196776e+00,	3.769324e-02,
              2.655256e+00,	6.659798e-01,	1477010443400000, 2.679323e+00,	6.091545e-01,	5.194504e+00,	6.456542e-02;

  for (int i = 0; i<5; i++){
    for (int j = 0; j<3; j++){
      std::cout << laserData(i,j) << ",";
    }
    std::cout << " " << std::endl;
  }

}

//// establish variables/vectors/matrices needed for testing ////

long long previous_timestamp_;
long long timestamp_;

Eigen::MatrixXd R_laser_;
Eigen::MatrixXd R_radar_;
Eigen::MatrixXd H_laser_;
Eigen::MatrixXd Hj_;

// tool object used to compute Jacobian and RMSE
Tools tools;

// noise components
float noise_ax;
float noise_ay;

//// initialize a kalman filter object ////

KalmanFilter ekf_;

//// create matrices for use in testing ////

// previous timestamp

previous_timestamp_ = 0;
timestamp_ = 0;

// measurement vector

Eigen::VectorXd meas;
meas = VectorXd(2);
meas << 1, 1;

// initialize state vector

ekf_.x_ = VectorXd(4);
ekf_.x_ << 1, 1, 1, 1;

//measurement covariance matrix - laser
R_laser_ = MatrixXd(2, 2);
R_laser_ << 0.0225, 0,
            0, 0.0225;

//measurement covariance matrix - radar
R_radar_ = MatrixXd(3, 3);
R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;


// state convariance matrix
ekf_.P_ = MatrixXd(4, 4);
ekf_.P_ << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1000, 0,
           0, 0, 0, 1000;

// initial state transition Matrix

ekf_.F_ = MatrixXd(4, 4);
ekf_.F_ << 1, 0, 1, 0,
           0, 1, 0, 1,
           0, 0, 1, 0,
           0, 0, 0, 1;

// measurement matrix laser
H_laser_ = MatrixXd(2, 4);
H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

// measurement matrix radar - jacobian
Hj_ = MatrixXd(3, 4);

//measurement noise components
noise_ax = 9;
noise_ay = 9;

//// --- tests --- ////

// initialize

for (int i=0 ; i<incr; i++){

  if (!is_initialized_)
  {
    if(measType == 0)
    {

    // lidar test

    double px = laserDataRead(i,0);
    double py = laserDataRead(i,1);
    timestamp_ = laserDataRead(i,2);

    ekf_.x_ << px,py, 0, 0;

    std::cout << "lidar" << std::endl;

    } else {

    // radar test

    double px;
    double py;
    timestamp_ = 1477010443000000;

    ekf_.x_ << px,py, 0, 0;

    }
    is_initialized_ = true;

  }

  //// prediction test ////

  timestamp_ = laserDataRead(i,2);
  float dt = (timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = timestamp_;
  std::cout << "timestamp_ = " << timestamp_ << std::endl;
  std::cout << "previous_timestamp_ = " << previous_timestamp_ << std::endl;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set the process covariance matrix Q

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

   ekf_.Predict();

   //// Update test ////

   if (measType == 1) {
     // Radar updates

     double px = laserDataRead(i,0);
     double py = laserDataRead(i,1);
     meas << px, py;

     ekf_.H_ = tools.CalculateJacobian(ekf_.x_); // calculate Jacobian and associate it with the H_ matrix
     ekf_.R_ = R_radar_; // update the radar measurement covariance matrix
     ekf_.UpdateEKF(meas); //update EKF

   } else {
     // Laser updates

     double px = laserDataRead(i,0);
     double py = laserDataRead(i,1);
     meas << px, py;

     ekf_.H_ = H_laser_; // update the measurement matrix
     ekf_.R_ = R_laser_; // update the measurement covariance matrix
     ekf_.Update(meas); // update KF

   }

   // extract ground truth values for position and speed

   VectorXd groundTruth(4);

   groundTruth(0) = laserDataRead(i,3);
   groundTruth(1) = laserDataRead(i,4);
   groundTruth(2) = laserDataRead(i,5);
   groundTruth(3) = laserDataRead(i,6);

   ground_truth.push_back(groundTruth);

   // Calculate RMSE gt_values

   VectorXd estimate(4);

   double p_x = ekf_.x_(0);
   double p_y = ekf_.x_(1);
   double v1  = ekf_.x_(2);
   double v2 = ekf_.x_(3);

   estimate(0) = p_x;
   estimate(1) = p_y;
   estimate(2) = v1;
   estimate(3) = v2;

   estimations.push_back(estimate);

   VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

   // print the output

   //std::cout << "F_ = " << ekf_.F_ << std::endl;
   //std::cout << "x_ = " << ekf_.x_ << std::endl;
   //std::cout << "P_ = " << ekf_.P_ << std::endl;
   //std::cout << "RMSE = " << RMSE << std::endl;
   //std::cout << "- - - - - - - - - - - - - - -" << std::endl;

   // Write data to a file

   dataFileOut << i << "  " << RMSE(0) << "  " << RMSE(1) << "  " << RMSE(2) << "  " << RMSE(3) << "\n" ;


}

dataFileOut.close();

}
