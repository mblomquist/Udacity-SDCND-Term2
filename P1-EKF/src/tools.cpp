#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	  VectorXd rmse(4);
	  rmse << 0, 0, 0, 0;

	  // check the validity of the following inputs:
	  //  * the estimation vector size should not be zero
	  //  * the estimation vector size should equal ground truth vector size
	  if (estimations.size() != ground_truth.size()
			|| estimations.size() == 0) {
			cout << "Invalid estimation or ground_truth data" << endl;
			return rmse;
	  }

	  //accumulate squared residuals
	  for (unsigned int i = 0; i < estimations.size(); ++i) {

			VectorXd residual = estimations[i] - ground_truth[i];

			//coefficient-wise multiplication
			residual = residual.array()*residual.array();
			rmse += residual;
	  }

	  //calculate the mean
	  rmse = rmse / estimations.size();

	  //calculate the squared root
	  rmse = rmse.array().sqrt();

	  //return the result
	  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	  MatrixXd Hj(3, 4);

	  //recover state parameters
	  float px = x_state(0);
	  float py = x_state(1);
	  float vx = x_state(2);
	  float vy = x_state(3);

	  // check for 0 like values
	  if (fabs(px) < 0.00001 && fabs(py) < 0.00001) {
			px = 0.00001;
			py = 0.00001;
	  }
	  else if (fabs(px) < 0.00001) {
			px = 0.00001;
	  }

	  //pre-compute a set of terms to avoid repeated calculation
	  float c1 = px*px + py*py;
	  float c2 = sqrt(c1);
	  float c3 = (c1*c2);

	  //compute the Jacobian matrix
	  Hj << (px / c2), (py / c2), 0, 0,
			-(py / c1), (px / c1), 0, 0,
			py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;

	  return Hj;
}
