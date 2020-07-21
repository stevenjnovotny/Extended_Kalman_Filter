#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check input validitity
  // * estimatationsize not zero
  // * estimation and ground truth are same size   
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
     std::cout << "CalculateRMSE - Error - invalid estimations and/or ground truth" << std::endl;
     return rmse;
  }
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

   MatrixXd Hj(3,4);

   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   float c1 = px*px+py*py;
   float c2 = sqrt(c1);
   float c3 = (c1*c2);

   // check divide by zero
   if (fabs(c1) < 0.0001) {
      std::cout << "CalculateJacobian() - Error - Divide by zero" << std::endl;
      return Hj;
   }

   // Compute jacobian
   Hj << (px/c2), (py/c2), 0, 0,
         -(py/c1), (px/c1), 0, 0,
         py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
   return Hj; 
}
