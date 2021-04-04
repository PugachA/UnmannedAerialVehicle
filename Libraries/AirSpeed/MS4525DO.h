#ifndef _MS4525DO_LIB_H_
#define _MS4525DOLIB_H_

#include "stm32f4xx_hal.h"
#include "math.h"
#include "cmsis_os.h"

class MS4525DO
{
	private:
		const float rho = 1.2041f;

		uint8_t MS4525DO_addr;
		uint8_t data_read_cmd;
		uint8_t data_size;
		I2C_HandleTypeDef *hi2c; //pointer to the i2c bus
		
		uint16_t timeout;
		float dt;
		
		float k_lp_alt;
		float offset;
		float air_speed_offset;
		
		float raw_pressure_pa = 0;
		float filter_pressure_pa = 0;
		float air_speed = 0;
		float lpFilterOutput = 0;
		
		uint16_t getSensorData();
		void calcPressure();
		float calcOffseet();
		void calcAirSpeed();
		void lpfilter();
		
	public:
		MS4525DO(I2C_HandleTypeDef *hi2c, float dt);
		float getPressure();
		float getAirSpeed();
	
};
#endif
