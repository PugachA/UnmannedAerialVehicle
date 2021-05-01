/*
 * Gps.h
 *
 *  Created on: Apr 28, 2021
 *      Author: foton
 */

#include "stdio.h"
#include "string.h"
#include "Gps\minmea.h"
#include "stm32f4xx_hal.h"

#ifndef Gps_H_
#define Gps_H_
#define BUFFER_SIZE 128

class Gps
{
	public:
		Gps(UART_HandleTypeDef* huart,
				GPIO_TypeDef* monitorGPIO,
				uint16_t monitorPin);
		virtual ~Gps();
		minmea_sentence_rmc gpsData;
		void Start();
		void UartCallback();

	private:
		UART_HandleTypeDef *huart;
		GPIO_TypeDef* monitorGPIO;
		uint16_t monitorPin;
		uint8_t rxData = 0;
		uint8_t rxBuffer[BUFFER_SIZE];
		uint8_t rxIndex = 0;

		bool Validate(char* data);
		void ErrorMonitor();
		void SuccessMonitor();
};

#endif /* Gps_H_ */
