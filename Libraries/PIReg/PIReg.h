#ifndef _PIREG_LIB_H_
#define _PIREG_LIB_H_

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

public:
	PIReg(double k_pr, double k_int, double dt, double integral_lim); //constructor
	void setError(double error);
	void calcOutput(void);
	double getOutput(void);
	void integralReset(void);
	void setGainParams(double k_pr, double k_int);
	void setAirSpeed(double airSpeed);

};
#endif
