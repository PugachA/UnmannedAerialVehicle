#include "stm32f4xx_hal.h"
#include "PIReg.h"
#include "math.h"

PIReg::PIReg(double k_int, double k_pr, int overflows_to_integrate)
{
	this->k_int = k_int;
	this->k_pr = k_pr;
	dt = 0.001*(double)overflows_to_integrate; //0.001 is a timer period, idk how to put it universally

	this->integral = 0;
	this->error = 0;
	this->output = 0;
}

void PIReg::setError(double error)
{
	this->error = error;
}

void PIReg::calcOutput(void)
{
	double proportional = 0;

	proportional = this->k_pr*this->error;
	this->integral += this->error*this->k_int;

	this->output = proportional+this->integral;
}

double PIReg::getOutput(void)
{
	return this->output;
}

void PIReg::integralReset(void)
{
	this->integral = 0;
}