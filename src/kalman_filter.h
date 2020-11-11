#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

 typedef struct kalmanState{
	VectorXd x;	// state vector
	MatrixXd P;	// state covariance matrix
} t_kalmanState;


typedef struct kalmanPredict{
	MatrixXd F;	// process state transition matrix
	MatrixXd Q;	// process covariance matrix
} t_kalmanPredict;


typedef struct kalmanUpdate{
	MatrixXd H;	// measurement state transition matrix
	MatrixXd R;	// measurement covariance matrix
} t_kalmanUpdate;


/**
 * Prediction Predicts the state and the state covariance
 * using the process model
 * @param delta_T Time between k and k+1 in s
 */
t_kalmanState Predict(const t_kalmanState& state, const t_kalmanPredict& process);

/**
 * Updates the state by using standard Kalman Filter equations
 * @param z The measurement at k+1
 */
t_kalmanState Update(const VectorXd& z, const t_kalmanState& state, const t_kalmanUpdate& update);

/**
 * Convert Cartesian coordinates to Polar coordinates
 * @param x The coordinates in cartesian system
 */
VectorXd ConvertCartesianToPolar(const VectorXd& x);

/**
 * Updates the state by using Extended Kalman Filter equations
 * @param z The measurement at k+1
 */
t_kalmanState UpdateEKF(const VectorXd& z, const t_kalmanState& state, const t_kalmanUpdate& update);

#endif // KALMAN_FILTER_H_
