#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}
Tools::~Tools() {}

// h function for mapping cartesian coordinates to polar

VectorXd Tools::radarMeasFunc(VectorXd x)
{

  VectorXd polarVals(3);

  double px = x(0);
  double py = x(1);
  double vx = x(2);
  double vy = x(3);

  double rho_f = sqrt(pow(px,2)+pow(py,2));
  double phi_f = atan2(py,px);
  double rho_dot_f = (px*vx+py*vy)/sqrt(pow(px,2)+pow(py,2));

  polarVals << rho_f, phi_f, rho_dot_f;

  return polarVals;

}

// Convert polar to cartesian coordinates

void Tools::convertPolarToCart(double &px, double &py)
{
  float rho;
  float phi;

  rho = px;
  phi = py;

  px = rho*sin(phi);
  py = rho*cos(phi);
}

// calculation of RMSE

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{

  // declare additional variables that will be used

  Eigen::VectorXd xSub(4); // subtracted vectors
  Eigen::VectorXd squared(4); // squared vectors
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  ///Error Checks ///

  // vector size must not be zeros

  if (estimations.size() == 0 || ground_truth.size() == 0 )
  {
    std::cerr << "The size of either the estimation or gound truth vector is 0" << std::endl;
    return rmse;
  }

  // estimation vector size should be equal to ground truth size

  if (estimations.size() != ground_truth.size())
  {
    std::cerr << "The size of the estimation matrix does not equal the ground_truth matrix" << std::endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;

}

// calculation of the Jacobian Matrix

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{

  MatrixXd Hj(3,4);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  //cout << "c1: " << c1 << endl;

  //check division by zero
  if(fabs(c1) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;

}
