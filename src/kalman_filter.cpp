#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ *  P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd Pht = P_ * Ht;
    MatrixXd K = Pht * Si;

    SetNewEstimate(x_, K, y, H_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

    VectorXd z_polar = CartesianToPolar(x_);
    VectorXd y = z - z_polar;
    y(1) = fmod(y(1), M_PI);
    MatrixXd Hj = CalculateJacobian(x_);
    MatrixXd Hjt = Hj.transpose();
    MatrixXd S = Hj * P_ * Hjt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd Pht = P_ * Hjt;
    MatrixXd K = Pht * Si;

    SetNewEstimate(x_, K, y, H_);
};

void KalmanFilter::SetNewEstimate(VectorXd x_, MatrixXd K,
                                  VectorXd y, MatrixXd H_) {
    x_ = x_ + (K * y);
    long x_size = x_.size()
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
};
