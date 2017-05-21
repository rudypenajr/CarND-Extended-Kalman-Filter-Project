#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO: COMPLETED
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size() || estimations.size() == 0){
        cout << "Invalid estimation or ground truth data" << endl;
        return rmse;
    }

    for (unsigned int i=0; i < estimations.size(); i++) {
        VectorXd residual = estimations[i] - ground_truth[i];

        // Coefficient-wise Multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    // Calculate the mean
    rmse = rmse/estimations.size();

    // Calculate the squared root
    rmse = rmse.array().sqrt();

    return rmse;
};

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO: COMPLETED
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // Precompute set of terms to avoid confusion
    float c1 = px*px + py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    // check division by zero
    if (fabs(c1) < 0.0001) {
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

    Hj << (px/c2), (py/c2), 0, 0,
            -(py/c1), (px/c1), 0, 0,
            px*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, (px/c2), (py/c2);

    return Hj;
};

MatrixXd Tools::CartesianToPolar(const Eigen::VectorXd& x_cartesian) {
    cout << "Converting Cartesian Parameters to Polar" << endl;
    // - Function will return/output 3 x 1 Vector with:
    // {
    //  rho - radial distance from origin to our object
    //  phi - angle between the ray and x direction
    //  rho dot - radial velocity (Doppler)
    // }
    // - Input is in cartesian so it is given to us as:
    // {
    //  px
    //  py
    //  vx
    //  vy
    // }
    VectorXd polar_vector(3);
    float thresh = 0.0001;
    float px = x_cartesian(0);
    float py = x_cartesian(1);
    float vx = x_cartesian(2);
    float vy = x_cartesian(3);

    // Compute Radial distance
    float rho = sqrt(px*py + px*py);

    // Compute Bearing Angle (phi)
    // Note: Per Tips from Udacity, Radians must be in range[-pi, pi].
    float phi = atan2(px, py);

    // Compute Radial Velocity
    if (rho < thresh) {
      cout <<  "WARNING - CartesianToPolar() - Division by Zero!" <<endl;
    }
    // Like Javascript!
    float rho_dot = (rho > thresh) ? ( px * vx + py * vy ) / rho : 0.0;

    polar_vector << rho, phi, rho_dot;
    return polar_vector;
};

MatrixXd Tools::PolarToCartesian(const Eigen::VectorXd& x_polar) {
    cout << "Converting Polar Parameters to Catesian" << endl;
    // - Function will return/output 4 x 1 Vector with:
    // {
    //  px
    //  py
    //  vx
    //  vy
    // }
    // - Input is in polar os it is given to us as:
    // {
    //  rho - radial distance from origin to our object
    //  phi - angle between the ray and x direction
    //  rho dot - radial velocity (Doppler)
    // }
    VectorXd cartesian_vector(4);

    // Grab State Parameters
    float rho = x_polar(0);
    float phi = x_polar(1);
    float rho_dot = x_polar(2);

    // Get Cartesian Positioning
    float px = rho * cos(phi);
    float py = rho * sin(phi);
    float vx = rho_dot * cos(phi);
    float vy = rho_dot * sin(phi);

    cartesian_vector << px, py, vx, vy;

    return cartesian_vector;
};
