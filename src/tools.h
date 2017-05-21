#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
  * A helper method to convert cartesian state to polar.
  */
  MatrixXd CartesianToPolar(const Eigen::VectorXd& x_cartesian);

  /**
  * A helper method to convert polar state to cartesian.
  */
  MatrixXd PolarToCartesian(const Eigen::VectorXd& x_polar);
};

#endif /* TOOLS_H_ */
