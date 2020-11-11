#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;


typedef struct extendedKalmanUpdate {
 	float 			noise_ax;
	float 			noise_ay;
	bool 			is_initialized;
	long long 		previous_timestamp;
	MatrixXd 		R_laser;		// Laser measurement noise covariance matrix
	MatrixXd 		R_radar;		// Radar measurement noise covariance matrix
	MatrixXd 		H_laser;		// Laser measurement matrix
	MatrixXd 		Hj;			// Radar measurement Jacobian matrix

} t_extendedKalmanUpdate;

/**
 * @purpose:	Convert data in cartesian system to polar system
 * @parameters:	VectorXd x - coordinates in cartesian system
 * @return: 	VectorXd data in polar system
 **/
t_extendedKalmanUpdate InitializeProcessMeasurement();

/**
 * @purpose:	Convert data in cartesian system to polar system
 * @parameters:	VectorXd x - coordinates in cartesian system
 * @return: 	VectorXd data in polar system
 **/
t_kalmanState InitializeState(const t_measurementPackage &measurement_pack);


/**
 * @purpose:	Run the whole flow of the Kalman Filter from here.
 * @parameters:	VectorXd x - coordinates in cartesian system
 * @return: 	VectorXd data in polar system
 **/
t_kalmanState ProcessMeasurement(const t_extendedKalmanUpdate& extend, const t_kalmanState& state, const t_measurementPackage &measurement_pack);

#endif // FusionEKF_H_
