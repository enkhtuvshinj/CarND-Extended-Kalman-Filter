#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include <iostream>
#include "Eigen/Dense"


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

/**
 * A helper method to calculate RMSE
 */
VectorXd CalculateRMSE(const std::vector<VectorXd> &estimations, const std::vector<VectorXd> &ground_truth);

/**
 * A helper method to calculate Jacobians
 */
MatrixXd CalculateJacobian(const VectorXd& x_state);

#endif  // TOOLS_H_