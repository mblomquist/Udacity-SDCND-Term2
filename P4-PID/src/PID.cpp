#include "PID.h"
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	  this->Kp = Kp;
	  this->Kd = Kd;
	  this->Ki = Ki;
}

void PID::UpdateError(double cte) {
	  d_error = cte - p_error;
	  p_error = cte;
	  i_error = i_error + cte;

}

double PID::TotalError() {

	  double steer = -Kp * p_error - Kd * d_error - Ki * i_error;

	  if (fabs(steer)>1.0) {
			if (steer> 0) {
				  steer = 1.0;
			}
			else {
				  steer = -1.0;
			}
	  }

	  return steer;
}
