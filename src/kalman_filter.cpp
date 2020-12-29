#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  
  //pseudo code of input vector as zero.
  Eigen::VectorXd u = VectorXd(4);
  u << 0, 0, 0, 0;

  x_ = F_ * x_ + u;
  P_ = F_ * P_ * F_.transpose() + Q_;
  //P_ = 0.9 * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
  Eigen::MatrixXd I = Eigen::MatrixXd(x_.size(),x_.size());
  for (int i = 0; i < x_.size(); ++i){
    for (int j = 0; j < x_.size(); ++j){
      I(i, j) = 0;
    }
    I(i, i) = 1;
  }
  
  Eigen::VectorXd y = z - H_ * x_; // error
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  Eigen::MatrixXd I = Eigen::MatrixXd(x_.size(),x_.size());
  for (int i = 0; i < x_.size(); ++i){
    for (int j = 0; j < x_.size(); ++j){
      I(i, j) = 0;
    }
    I(i, i) = 1;
  }
  
  //Eigen::VectorXd y = z - H_ * x_; // error
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt((px * px) + (py * py));
  float phi = atan2(py, px);
  float rho_dot=0;

  if (fabs(rho) < 0.0001){
    rho_dot = 0;
  }
  else{
    rho_dot = ( px * vx + py * vy) / rho;
  }

  Eigen::VectorXd z_pred = Eigen::VectorXd(3);
  z_pred << rho, phi, rho_dot;

  Eigen::VectorXd y = z - z_pred;
  
  /*
  * Normalizing Angles
  * In C++, atan2() returns values between -pi and pi.
  * When calculating phi in y = z - h(x) for radar measurements,
  * the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi.
  * The Kalman filter is expecting small angle values between the range -pi and pi.
  * HINT: when working in radians, you can add 2π or subtract 2π until the angle is within the desired range
  */

  if (y(1) < -M_PI) // y(1) refers to phi
  {
    y(1) += 2 * M_PI;
  }
    
  else if (y(1) > M_PI)
  {
    y(1) -= 2 * M_PI;
  }
  
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_; // 3x3
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse(); // 4x4 * 4x3 * x 3x3 = 4x3

  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;

}
