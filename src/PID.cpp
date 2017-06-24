#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;
  sq_err = 0;
  mean_sq_err = 0;
  count = 0;
  is_initialized = false;       // for the very first calculation
}

void PID::UpdateError(double cte) {
  if (!is_initialized){
    // make the d_error as zero since this is the first calculation
    d_error = 0;
    is_initialized = true;
  }
  else{
    d_error = cte - p_error;    // since p_error is not updated, this has the previous value of cte
  }
  p_error = cte;
  i_error += cte;
  sq_err += cte*cte;
  ++count;
  mean_sq_err = sq_err/count;
}

double PID::TotalError() {
  return Kp*p_error + Ki*i_error + Kd*d_error;
}

