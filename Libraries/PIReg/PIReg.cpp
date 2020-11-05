#include "PIReg.h"
#include "math.h"

PIReg::PIReg(double k_int, double k_pr, int dt, double integral_lim, double output_lim)
{
	this->k_int = k_int;
	this->k_pr = k_pr;
	this->dt = dt;

	this->integral = 0;
	this->error = 0;
	this->output = 0;

	this->integral_lim = integral_lim;
	this->output_lim = output_lim;
}

void PIReg::setError(double error)
{
	this->error = error;
}

void PIReg::calcOutput(void)
{
	double proportional = this->k_pr*this->error;

	this->integral += this->error*this->k_int;

	if (this->integral > this->integral_lim)
		this->integral = this->integral_lim;

	else if (this->integral < -this->integral_lim)
		this->integral = -this->integral_lim;

	this->output = proportional+this->integral;

	if (this->output > this->output_lim)
		this->output = this->output_lim;

	else if (this->output < -this->output_lim)
		this->output = -this->output_lim;
}

double PIReg::getOutput(void)
{
	return this->output;
}

void PIReg::integralReset(void)
{
	this->integral = 0;
}
