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

public:
	PIReg(double,double,int); //constructor
	void setError(double);
	void calcOutput(void);
	double getOutput(void);
	void integralReset(void);

};
#endif