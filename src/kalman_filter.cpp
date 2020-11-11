#include "kalman_filter.h"


/**
 * @purpose	:	Convert data in cartesian system to polar system
 * @param	:	VectorXd x - coordinates in cartesian system
 * @return	: 	VectorXd data in polar system
 **/
VectorXd ConvertCartesianToPolar(const VectorXd& x)
{
	VectorXd temp = VectorXd(3);
	temp << 0, 0, 0;
	
	float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);

    // Convert previous prediction into polar coordinates
    float rho_p = sqrt(px*px + py*py);
    float theta_p = atan2(py,px);

  	// Prevent the division by zero
    if (rho_p < 0.0001)	{
		cout << "ERROR: ConvertCartesianToPolar() - Division by Zero" << endl;
      	//rho_p = 0.0001;
      	return temp;
    }

    float rho_dot_p = (px*vx + py*vy)/rho_p;

  	// Save the previous prediction in polar system to vector
    temp << rho_p, theta_p, rho_dot_p;
	
	return temp;
}


/**
 * @purpose	:	Predict the state using kinematic equations
 * @param	:	t_KalmanState state
 *				t_kalmanPredict process
 * @return	: 	t_KalmanState - estimated states
 **/
t_kalmanState Predict(const t_kalmanState& state, const t_kalmanPredict& process)
{
    t_kalmanState temp;
    temp.x = process.F * state.x;
    temp.P = process.F * state.P * process.F.transpose() + process.Q;

    return temp;
}


/**
 * @purpose	:	Update the state based on data from Lidar by using Kalman Filter equations
 * @param	:	VectorXd z - raw measurement data from Lidar in cartesian system
 * 				t_KalmanState state
 *				t_kalmanPredict update
 * @return	: 	t_KalmanState - updated states
 **/
t_kalmanState Update(const VectorXd& z, const t_kalmanState& state, const t_kalmanUpdate& update)
{
	t_kalmanState temp;

    VectorXd y = z - update.H * state.x;
	MatrixXd Ht = update.H.transpose();
	MatrixXd PHt = state.P * Ht;
    MatrixXd S = update.H * PHt + update.R;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;

    // Estimate state and covariance
	long x_size = state.x.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
	
    temp.x = state.x + (K * y);
    temp.P = (I - K * update.H) * state.P;
	
	return temp;
}


/***
 * @purpose	:	Update the state based on data from Radar by using Extended Kalman Filter equations
 * @param	:	VectorXd z - raw measurement data from Radar in polar system
 * 				t_KalmanState state
 *				t_kalmanPredict update
 * @return	: 	t_KalmanState - updated states
 ***/
t_kalmanState UpdateEKF(const VectorXd& z, const t_kalmanState& state, const t_kalmanUpdate& update) 
{
  /**
   * Update the state by using Extended Kalman Filter equations for Radar
   * Radar sends data in polar system that needs to be converted to Cartesian system
   */
   	t_kalmanState temp;

	// Calculate the error equation
	VectorXd z_pred = ConvertCartesianToPolar(state.x);
    VectorXd y = z - z_pred;

    // Normalizing Angles. it adjusts theta if it is outside of [-pi, pi]
	y(1) = (y(1) < -M_PI) ? (y(1) + 2*M_PI): (y(1) > M_PI) ? (y(1) - 2*M_PI): y(1);

	MatrixXd Ht = update.H.transpose();
    MatrixXd PHt = state.P * Ht;
    MatrixXd S = update.H * PHt + update.R;
    MatrixXd K = PHt * S.inverse();

    // Update state and covariance
	long x_size = state.x.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    temp.x = state.x + (K * y);
    temp.P = (I - K * update.H) * state.P;
	
	return temp;
}
