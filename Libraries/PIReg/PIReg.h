#ifndef _PIREG_LIB_H_
#define _PIREG_LIB_H_

class PIReg
{
private:
	double k_int;
	double k_pr;
	double dt;
	double error;
	double integral;
	double output;
	double integral_lim;

public:
	PIReg(double k_int, double k_pr, int dt, double integral_lim); //constructor
	void setError(double error);
	void calcOutput(void);
	double getOutput(void);
	void integralReset(void);

};
#endif
