#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  //*** initializing matrices *** //

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

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    // first measurement

    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1; //initial state matrix, filler

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // initialize state vector via radar measurement

      double px = measurement_pack.raw_measurements_[0];
      double py = measurement_pack.raw_measurements_[1];

      tools.convertPolarToCart(px,py); // convert to cartesian coordinate

      ekf_.x_ << px,py, 0, 0; // initialize state based on first radar measurement
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // initialize state vector via lidar measurements

      ekf_.x_(0) = measurement_pack.raw_measurements_[0];
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];

    }

    // initial time

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;

  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/


 //compute the time elapsed between the current and previous measurements

 float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
 previous_timestamp_ = measurement_pack.timestamp_;

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

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO: DONE
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    ekf_.H_ = tools.CalculateJacobian(ekf_.x_); // calculate Jacobian and associate it with the H_ matrix
    ekf_.R_ = R_radar_; // update the radar measurement covariance matrix
    ekf_.UpdateEKF(measurement_pack.raw_measurements_); //update EKF

  } else {
    // Laser updates

    ekf_.H_ = H_laser_; // update the measurement matrix
    ekf_.R_ = R_laser_; // update the measurement covariance matrix
    ekf_.Update(measurement_pack.raw_measurements_); // update KF

  }

}
