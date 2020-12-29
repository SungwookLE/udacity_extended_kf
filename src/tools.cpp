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
  Eigen::VectorXd RMSE = Eigen::VectorXd(4);
  RMSE << 0, 0, 0, 0;

  if( estimations.size() == ground_truth.size() && estimations.size() !=0){
     
     Eigen::VectorXd error_square = VectorXd(4);
     
     error_square << 0, 0, 0, 0;
     //std::cout << "SIZE: " << estimations.size() << std::endl;
     for (int i; i < estimations.size(); ++i)
     {
        error_square = (estimations.at(i) - ground_truth.at(i)).array()* (estimations.at(i) - ground_truth.at(i)).array(); //array() 는 벡터 연산에서 요소끼리의 곱을 하게 해주는 것 (1,1) * (1,1)
        RMSE += error_square;
     }
     RMSE = RMSE / estimations.size();
     RMSE = error_square.array().sqrt();
  }
  
  return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   *     
   */
   Eigen::MatrixXd Hj = Eigen::MatrixXd(3, 4);
   for (int i = 0; i < 3; ++i){
      for (int j = 0; j < 4; ++j){
         Hj(i, j) = 0;
      }
   }
   
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   // pre-compute a set of terms to avoid repeated calculation
   float c1 = px*px+py*py;
   float c2 = sqrt(c1);
   float c3 = (c1*c2);
   // check division by zero
   if (fabs(c1) < 0.0001 ) {
      std::cout << "divide by zero defense!" << std::endl;
      return Hj;
   }
   // Hj = Eigen::MatrixXd(3, 4);
   // Hj(0, 0) = px / sqrt(px * px + py * py);
   // Hj(0, 1)=  py  / sqrt(px*px + py*py);
   // Hj(0, 2) = 0;
   // Hj(0, 3) = 0;

   // Hj(1, 0) = -py / (px * px + py * py);
   // Hj(1, 1)=  px  / (px*px + py*py);
   // Hj(1, 2) = 0;
   // Hj(1, 3) = 0;
   // Hj(2, 0)=  py  * (vx*py-vy*px) / pow((px*px+py*py),1.5);
   // Hj(2, 1)=  px  * (vy*px-vx*py) / pow((px*px+py*py),1.5);
   // Hj(2, 2)=  px  / sqrt(px*px+py*py);
   // Hj(2, 3)=  Hj(0,1);

   //compute the Jacobian matrix
   Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

   return Hj;
}
