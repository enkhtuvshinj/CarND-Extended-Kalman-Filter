#include "FusionEKF.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;


/**
 * @purpose	:	Initialize variables of extended kalman filter before start of measurement
 * @param	:	None
 * @return	: 	t_extendedKalmanUpdate initialized variables
 **/
t_extendedKalmanUpdate InitializeProcessMeasurement()
{
	t_extendedKalmanUpdate extend;

	// Initializing matrices
	extend.R_laser = MatrixXd(2,2);
	extend.R_radar = MatrixXd(3,3);
	extend.H_laser = MatrixXd(2,4);
	extend.Hj = MatrixXd(3,4);

	// Measurement noise covariance matrix - laser
	extend.R_laser << 0.0225, 0,
					  0, 0.0225;

	// Measurement noise covariance matrix - radar
	extend.R_radar << 0.09, 0, 0,
					  0, 0.0009, 0,
					  0, 0, 0.09;

	// Measurement matrix - laser
	extend.H_laser << 1,0,0,0,
					  0,1,0,0;

	// Acceleration noises
	extend.noise_ax = 9.0;
	extend.noise_ay = 9.0;

	return extend;
}


/**
 * @purpose	:	Initialize the state and covariance matrix based on first measurement data
 * @param	:	t_measurementPackage measurement_pack
 * @return	: 	t_kalmanState initialized state
 **/
t_kalmanState InitializeState(const t_measurementPackage &measurement_pack)	
{
	t_kalmanState state;
	
    cout << "Extended Kalman Filter:" << endl;
    state.x = VectorXd(4);
    state.x << 1, 1, 1, 1;

    if (measurement_pack.sensor_type == measurementPackage::RADAR)	{
    	// Convert radar from polar to cartesian coordinates
        float rho = measurement_pack.raw_measurements[0];
		float theta = measurement_pack.raw_measurements[1];
//		float rho_dot = measurement_pack.raw_measurements[2];
		
		// Initialize state
		state.x(0) = rho*cos(theta);
		state.x(1) = rho*sin(theta);
//		state.x(2) = rho_dot*cos(theta);
//		state.x(3) = rho_dot*sin(theta);
    }
    else if (measurement_pack.sensor_type == measurementPackage::LASER)	{
    	// Initialize state
		state.x(0) = measurement_pack.raw_measurements[0];
		state.x(1) = measurement_pack.raw_measurements[1];
    }

	// Initialize the covariance matrix, P. Adjust the variance values to reflect uncertainty in initial state
	state.P = MatrixXd(4,4);
	state.P << 	1,0,0,  0,
				0,1,0,  0,
				0,0,500,0,
				0,0,0,500;

	return state;
}


/**
 * @purpose	: 	Run whole process of extended kalman filter
 * @param	: 	t_extendedKalmanUpdate extend
 * 				t_measurementPackage measurement_pack,
 *				t_kalmanState state
 * @return	: 	t_kalmanState new state
 **/
t_kalmanState ProcessMeasurement(const t_extendedKalmanUpdate& extend,
								const t_kalmanState& state,
								const t_measurementPackage &measurement_pack) 
{
	t_kalmanState 	eState;
	t_kalmanPredict ePredict;
	t_kalmanUpdate	eUpdate;
	
    /*******************************
     * 1. Prediction
     *******************************/

    // 1.1. Compute the time elapsed between the current and previous measurements (in seconds)
    float dt = (measurement_pack.timestamp - extend.previous_timestamp) / 1000000.0;
    float dt_2 = dt * dt;
  	float dt_3 = dt_2 * dt;
  	float dt_4 = dt_3 * dt;
  
    // 1.2. Modify the state transition matrix F so that the time is integrated
	ePredict.F = MatrixXd(4, 4);
	ePredict.F << 1,0,1,0,
				0,1,0,1,
				0,0,1,0,
				0,0,0,1;
    ePredict.F(0,2) = dt;
    ePredict.F(1,3) = dt;

    // 1.3. Set the process covariance matrix Q
    ePredict.Q = MatrixXd(4, 4);
    ePredict.Q << dt_4/4*extend.noise_ax, 0, dt_3/2*extend.noise_ax, 0,
				0, dt_4/4*extend.noise_ay, 0, dt_3/2*extend.noise_ay,
				dt_3/2*extend.noise_ax, 0, dt_2*extend.noise_ax, 0,
				0, dt_3/2*extend.noise_ay, 0, dt_2*extend.noise_ay;

	// 1.4. Predict the state and covarience
  	eState = Predict(state, ePredict);

  	/******************************
   	* 2. Update
   	*******************************/

	// 2.1. Check whether measurement from radar or lider
  	if (measurement_pack.sensor_type == measurementPackage::RADAR) {
    	// 2.2. Radar updates
		// 2.2.1. Calculate the Jacobian matrix about the current predicted state and set the EKF state transition matrix, H
		eUpdate.H = CalculateJacobian(eState.x);

		// 2.2.2. Initialize the EKF object measurement covariance matrix, R, to the right size and assign the correct values
		eUpdate.R = MatrixXd(3,3);
		eUpdate.R = extend.R_radar; 
		
		// 2.2.3. Update state and covarience matrix
		eState = UpdateEKF(measurement_pack.raw_measurements, eState, eUpdate);
  	} else {
    	// 2.3. Laser updates
		// 2.3.1. Set the EKF object to use the LASER sensor matrix, H
		eUpdate.H = extend.H_laser;
		
		// 2.3.2. Initialize the EKF object measurement covariance matrix, R, to the right size and assign the correct values
		eUpdate.R = MatrixXd(2,2);
		eUpdate.R = extend.R_laser;
		
		// 2.3.3. Update state and covarience matrix
		eState = Update(measurement_pack.raw_measurements, eState, eUpdate);
  	}

	// print the output
	cout << "x_ = " << eState.x << endl;
	cout << "P_ = " << eState.P << endl;
		
	return eState;
}
