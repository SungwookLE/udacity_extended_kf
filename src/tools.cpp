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
   * @brief Calculate the RMSE.
   */

  VectorXd RMSE = VectorXd::Zero(4);

  if (estimations.size() == ground_truth.size() && estimations.size() != 0){
     VectorXd error_square = VectorXd::Zero(4);

      for(int i = 0; i < estimations.size(); ++i){
         // `.array()` member can support elelment-wise product: 벡터 연산에서 요소끼리의 곱을 지원 (1,1) * (1,1)
         error_square = (estimations[i] - ground_truth[i]).array() * (estimations[i] - ground_truth[i]).array();
         RMSE += error_square;
      }

      RMSE = RMSE.array()/estimations.size();
      RMSE = RMSE.array().sqrt();
  }

  return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * @brief Calculate a Jacobian.
   * Radar Outputs follows {radial distance, velocity, degree} as Polar Coordinates.
   * 
   * But, States follows {px, py, vx, vy} as Cartesian Coordinates.
   * To transform from Cartesian information to Polar Coordinates, h_radar is not linear matrix. That is non-linear function
   * To linearize the non-linear function(h_radar), Jacobian linearization is used as follow.
   */
   MatrixXd Hj = MatrixXd::Zero(3,4);

   double px = x_state(0);
   double py = x_state(1);
   double vx = x_state(2);
   double vy = x_state(3);

   double c1 = px*px + py*py;
   double c2 = sqrt(c1);
   double c3 = (c1*c2);

   //Defense divide zero
   if (fabs(c1) < 0.0001){
      std::cout << "Defense: divide by zero..." << std::endl;
      return Hj;
   }
   Hj << (px/c2), (py/c2), 0, 0,
         -(py/c1), (px/c1), 0, 0,
         py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

   return Hj;
}
