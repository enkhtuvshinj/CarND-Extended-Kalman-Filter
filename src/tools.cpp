#include "tools.h"


/**
 * @purpose	:	Calculate RMSE
 * @param	:	VectorXd estimations 
 *				VectorXd round_truth
 * @return	: 	VectorXd rmse
 **/
VectorXd CalculateRMSE(	const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) 
{
  /**
   * Calculate the RMSE here.
   */
    VectorXd rmse(4);
  	rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    unsigned int n = estimations.size();
    if(n!=0 &&  n==ground_truth.size())
    {
        for(unsigned int i=0; i<n; i++)
        {
          	// Element-wise subtraction
            VectorXd residual = estimations[i] - ground_truth[i];
            // Element-wise multiplication
            residual = residual.array()*residual.array();
            rmse += residual;        
        }
        // Calculate the mean for element-wise
        rmse = rmse/n;
        // Calculate the squared root for element-wise
        rmse = rmse.array().sqrt();
    }
  	else
  	{
  		cout<<"ERROR: estimation vector is zero or vectors' size is not equal."<<endl;
  	}
  return rmse;
}


/**
 * @purpose	:	Calculate Jacobian matrix using x state
 * @param	:	VectorXd x state
 * @return	: 	MatrixXd Jacobian matrix
 **/
MatrixXd CalculateJacobian(const VectorXd& x_state) 
{
  /**
   * Calculate a Jacobian here.
   */
    MatrixXd Hj(3,4);
  	// Recover state parameters
  	float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // Pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    // Check division by zero
    if (fabs(c1) < 0.0001) {
      cout << "ERROR: CalculateJacobian() - Division by Zero" << endl;
      return Hj;
    }

    // Compute the Jacobian matrix
    Hj << (px/c2), (py/c2), 0, 0,
        -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

    return Hj;
}
