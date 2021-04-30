/*
 * Gps.cpp
 *
 *  Created on: Apr 28, 2021
 *      Author: foton
 */

#include "Gps.h"

Gps::Gps(UART_HandleTypeDef* huart,
		GPIO_TypeDef* monitorGPIO,
		uint16_t monitorPin)
{
	this->huart = huart;
	this->monitorGPIO = monitorGPIO;
	this->monitorPin = monitorPin;
}

Gps::~Gps()
{}

void Gps::Start()
{
	HAL_UART_Receive_IT(this->huart, &rxData, 1);
}

void Gps::UartCallback()
{
	if (rxData != '\n' && rxIndex < BUFFER_SIZE)
	{
		if(!(rxData == '$' && rxIndex == 1))
			rxBuffer[rxIndex++] = rxData;
	}
	else
	{
		gpsData.valid = false;

		if(Validate((char*) rxBuffer))
		{
			if(minmea_parse_rmc(&gpsData, (char*) rxBuffer))
				SuccessMonitor();
			else
				ErrorMonitor();
		}
		else
			ErrorMonitor();

		rxIndex = 0;
		memset(rxBuffer, 0, BUFFER_SIZE);
	}

	HAL_UART_Receive_IT(this->huart, &rxData, 1);
}

bool Gps::Validate(char* data)
{
	return true;
}

void Gps::ErrorMonitor()
{
	HAL_GPIO_WritePin(this->monitorGPIO, this->monitorPin, GPIO_PIN_SET);
}

void Gps::SuccessMonitor()
{
	HAL_GPIO_WritePin(this->monitorGPIO, this->monitorPin, GPIO_PIN_RESET);
}
