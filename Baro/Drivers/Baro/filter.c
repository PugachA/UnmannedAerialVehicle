#include "filter.h"
double filter_first(double input, double output_prev, float K)
{
	static double output = 0;
	double error = 0 ;
	
	error = K * (input - output_prev);
	output = output + error * 0.1;
	
	return output;
} 

double filter_second(double input, double output_prev, float K, double *error)
{
	static double output = 0;
	
	*error = K * (input - output_prev);
	output = output + *error * 0.1;
	
	return output;
} 