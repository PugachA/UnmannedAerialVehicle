#include "PIReg.h"

PIReg::PIReg(double k_pr, double k_int, double dt, double integral_lim, double *speed_ref_points, double *k_pr_points, double *k_int_points, uint8_t num_of_ref_points)
{
	this->k_pr = k_pr;
	this->k_int = k_int;
	this->dt = dt;

	this->integral = 0.0;
	this->error = 0.0;
	this->output = 0.0;

	this->integral_lim = integral_lim;

	this->speed_ref_points = speed_ref_points;
	this->k_pr_points = k_pr_points;
	this->k_int_points = k_int_points;
	this->num_of_ref_points = num_of_ref_points;

	this->air_speed = 0.0;
}

PIReg::PIReg(double k_pr, double k_int, double dt, double integral_lim)
{
	this->k_pr = k_pr;
	this->k_int = k_int;
	this->dt = dt;

	this->integral = 0;
	this->error = 0;
	this->output = 0;

	this->integral_lim = integral_lim;
}

void PIReg::setError(double error)
{
	this->error = error;
}

void PIReg::calcOutput(void)
{
	double proportional = this->k_pr*this->error;

	this->integral += this->error * (this->dt * this->k_int);

	if (this->integral > this->integral_lim)
		this->integral = this->integral_lim;

	else if (this->integral < -this->integral_lim)
		this->integral = -this->integral_lim;

	this->output = proportional + this->integral;
}

double PIReg::getOutput(void)
{
	return this->output;
}

void PIReg::integralReset(void)
{
	this->integral = 0;
}

void PIReg::setGainParams(double k_pr, double k_int)
{
	this->k_pr = k_pr;
	this->k_int = k_int;
}

void PIReg::setAirSpeed(double air_speed)
{
	this->air_speed = air_speed;
}

void PIReg::calcGainParams(void)
{
	uint8_t i = 0;
	double k_pr_buf = 0.0;
	double k_int_buf = 0.0;

	if (this->air_speed <= this->speed_ref_points[0]) //minimum
		{
			k_pr_buf = this->k_pr_points[0];
			k_int_buf = this->k_int_points[0];
		}
		else
			if (this->air_speed >= this->speed_ref_points[this->num_of_ref_points-1]) //maximum
			{
				k_pr_buf = this->k_pr_points[this->num_of_ref_points-1];
				k_int_buf = this->k_int_points[this->num_of_ref_points-1];
			}
			else
				while (i < this->num_of_ref_points-1) //linear interpolation
				{
					if ((this->air_speed >= this->speed_ref_points[i]) && (this->air_speed <= this->speed_ref_points[i+1]))
					{
						k_pr_buf = this->k_pr_points[i]+(this->k_pr_points[i+1]-this->k_pr_points[i])/(this->speed_ref_points[i+1]-this->speed_ref_points[i])\
								*(this->air_speed-this->speed_ref_points[i]);
						k_int_buf = this->k_int_points[i]+(this->k_int_points[i+1]-this->k_int_points[i])/(this->speed_ref_points[i+1]-this->speed_ref_points[i])\
								*(this->air_speed-this->speed_ref_points[i]);

					i++;
					}
				}

	setGainParams(k_pr_buf,k_int_buf);
}

double PIReg::getProportGain(void)
{
	return this->k_pr;
}

double PIReg::getIntGain(void)
{
	return this->k_int;
}
