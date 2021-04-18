#ifndef _PIREG_LIB_H_
#define _PIREG_LIB_H_

#include "stm32f4xx_hal.h"

class PIReg
{
private:
	double k_pr;
	double k_int;
	double dt;
	double error;
	double integral;
	double output;
	double integral_lim;
	double airSpeed;
	double *speed_ref_points;
	double *k_pr_points;
	double *k_int_points;
	uint8_t num_of_ref_points;


public:
	PIReg(double k_pr, double k_int, double dt, double integral_lim, double*, double*, double*, uint8_t); //constructor
	PIReg(double k_pr, double k_int, double dt, double integral_lim); //constructor with no coeff changing
	void setError(double error);
	void calcOutput(void);
	double getOutput(void);
	void integralReset(void);
	void setGainParams(double k_pr, double k_int);
	void setAirSpeed(double airSpeed);
	void calcGainParams(void);

};
#endif
