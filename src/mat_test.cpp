#include <iostream>
#include <math.h>
#include "Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

int main(){

  VectorXd y(4);
  y << -0.0001, 3,4,5;
  VectorXd x(4);
  x << 0, -3,4,5;




  if(fabs(y(0)) || fabs(y(1)) || fabs(y(2))|| fabs(y(3)) < 0.0001 ){
    cout << "Error - Division by Zero: y" << endl;
  }

  if(fabs(x(0)) || fabs(x(1)) || fabs(x(2))|| fabs(x(3))< 0.0001){
    cout << "Error - Division by Zero: x" << endl;
  }


}
