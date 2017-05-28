#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	this->p_error = 0.0;
	this->i_error = 0.0;
	this->d_error = 0.0;
}

void PID::UpdateError(double cte) {

	double prev_cte = p_error;

	p_error = cte;

	// I controller will accumulate the error
	i_error = i_error + cte;

	// assume timestep is 1
	d_error = cte - prev_cte;
}

double PID::TotalError() {
	return -Kp * p_error - Ki * i_error - Kd * d_error;
}

