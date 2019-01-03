#include "kalman_filter.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

// NOT USING THIS - DELETE LATER

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict()
{
  x_ = F_ * x_;
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
  // Measurement residual (innovation)
  VectorXd y_ = z - (H_ * x_);

  // Calculate the Kalman gain

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K_ = (P_* H_.transpose())*Si;

  // Update the state vector and the covariance matrix

  x_ = x_ + K_* y_; // update the estimate by weighting via the Kalman gain
  long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
  P_ = (I_ - K_ * H_)*P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{

  // check for division by zeros

  // limit angles

  // Measurement input
  y_ = z - H_;

  // Calculate the Kalman gain
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K_ = (P_* H_.transpose())*Si;

  // Update the state vector and the covariance matrix
  x_ = x_ + K_* y_;
  long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
  P_ = (I_ - K_ * H_)*P_;


}
