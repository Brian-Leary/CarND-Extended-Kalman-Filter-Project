#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

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
  TODO:
    * predict the state
  */
  x_ = F_ * x_; // (lesson 5, section 8)
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  //angle normalization
  while (y(1) > M_PI) y(1) -= 2. * M_PI;
  while (y(1) < -M_PI) y(1) += 2. * M_PI;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  //cout << "x_ = " << endl << x_ << endl;
  //cout << "P_ = " << endl << P_ << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1)); // range
  float phi;
  // float phi = atan2(x_(1), x_(0)); // bearing
  // float rho_dot = (x_(0 * x_(2) + x_(1) * x_(3)) / rho;
  float rho_dot; // range rate
  if (fabs(rho) > 0.001) {
    phi = atan2(x_(1), x_(0));
    rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
  } else {
      phi = 0;
      rho_dot = 0;
  }
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;

  VectorXd y = z - z_pred;

  //angle normalization
  while (y(1) > M_PI) y(1) -= 2. * M_PI;
  while (y(1) < -M_PI) y(1) += 2. * M_PI;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  //cout << "x_ = " << endl << x_ << endl;
  //cout << "P_ = " << endl << P_ << endl;
}
