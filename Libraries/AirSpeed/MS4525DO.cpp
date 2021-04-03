#include "MS4525DO.h"

MS4525DO::MS4525DO(I2C_HandleTypeDef *hi2c, float dt) //constructor
{
	this->MS4525DO_addr = 0x28;
	this->data_read_cmd = 0x01;
	this->data_size = 2;
	this->hi2c = hi2c;
	
	this->timeout = 1000;
	this->dt = dt;
	this->air_speed_offset = 2;
	
	this->lpFilterOutput = 0;
	this->k_lp_alt = 11;// 1/k = T - time constant for lpFilter, cut-off frequency = 20 rad/s ~ 3 Hz

	//------------------------MS4525DO Initialising---------------------------

	this->offset = calcOffseet();
}

uint16_t MS4525DO::getSensorData()
{
	  uint8_t data[2] = {0};
	  uint16_t sensor_data = 0;
	  uint8_t check = 0;

	  HAL_I2C_Master_Transmit(this->hi2c, MS4525DO_addr << 1, &data_read_cmd, 1, timeout);
	  HAL_Delay(5);
	  HAL_I2C_Master_Receive(this->hi2c, MS4525DO_addr << 1, data, data_size, timeout);

	  check |= (data[0] >> 6);
	  if(check == 0)
	  {
		  sensor_data = (data[0] << 8) + data[1];
		  sensor_data = 0x3FFF & sensor_data;
	  }
	  else
		  return 0;

	  return sensor_data;
}

void MS4525DO::calcPressure()
{
	  const float P_min = -1.0f;
	  const float P_max = 1.0f;
	  const float PSI_to_Pa = 6894.757f;

	  uint16_t sensor_data = getSensorData();

	  double pressure_PSI = 0;

	  pressure_PSI = -((((float)sensor_data)- 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
	  this->raw_pressure_pa = pressure_PSI * PSI_to_Pa;

	  if(sensor_data == 0)
		  this->raw_pressure_pa = -1;
}

float MS4525DO::getPressure()
{
	return (this->raw_pressure_pa - this->offset);
}

float MS4525DO::getAirSpeed()
{
	calcPressure();
	if(this->raw_pressure_pa < 0)
	{
		return this->air_speed;
	}
	lpfilter();
	calcAirSpeed();

	return this->air_speed;
}

void MS4525DO::lpfilter()
{
  	float error = 0;

  	error = k_lp_alt*(this->raw_pressure_pa - this->lpFilterOutput);
  	this->lpFilterOutput += error*dt;
  	this->filter_pressure_pa = this->lpFilterOutput;
}

void MS4525DO::calcAirSpeed()
{
	if((this->filter_pressure_pa - this->offset - this->air_speed_offset) >= 0)
	{
		this->air_speed = sqrt(2*(this->filter_pressure_pa - this->offset)/this->rho) - this->air_speed_offset;
	}
	else
	{
		this->air_speed = 0;
	}
}

float MS4525DO::calcOffseet()
{
	float offset = 0;
	uint8_t offset_counter = 0;

	while(offset_counter < 20)
	{
		calcPressure();
		if(this->raw_pressure_pa < 0)
			continue;

		offset += this->raw_pressure_pa;
		offset_counter++;
	}

	offset = offset/(offset_counter);

	return offset;
}
