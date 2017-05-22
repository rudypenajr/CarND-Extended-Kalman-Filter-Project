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
    MatrixXd Pht = P_ * Ht;
    MatrixXd S = H_ * Pht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = Pht * Si;

    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

    float rho = sqrt((x_[0] * x_[0]) + (x_[1] * x_[1]));
    float phi = 0;
    float rho_dot = 0;

    phi= atan2(x_[1], x_[0]);
    rho_dot = (rho > 0.0001) ? ((x_[0] * x_[2] + x_[1] * x_[3]) / rho) : 0;

    MatrixXd z_pred(3, 1);
    z_pred << rho, phi, rho_dot;
    // z_pred << tool.CartesianToPolar(x_);

    VectorXd y = z - z_pred;
    while (y(1) > M_PI)
    {
      y(1) -= 2 * M_PI;
    }
    while (y(1) <- M_PI)
    {
      y(1) += 2 * M_PI;
    }
    // fmod - returns the floating-point remainder of numer/denom (rounded towards zero)
    // y(1) = fmod(y(1), M_PI);
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd Pht = P_ * Ht;
    MatrixXd K = Pht * Si;

    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
