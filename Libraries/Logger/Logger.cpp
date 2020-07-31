/*
 * Logger.cpp
 *
 *  Created on: Jul 29, 2020
 *      Author: foton
 */

#include "Logger.h"

Logger::Logger(const char* loggerName,
		SDFileManager& fileManager,
		GPIO_TypeDef* successGPIO,
		uint16_t successPin,
		GPIO_TypeDef* errorGPIO,
		uint16_t errorPin) : loggerName(loggerName), fileManager(fileManager)
{
	this->successGPIO = successGPIO;
	this->successPin = successPin;
	this->errorGPIO = errorGPIO;
	this->errorPin = errorPin;

	CreateLogFile();
}

Logger::Logger():fileManager(fileManager)
{
	// TODO Auto-generated destructor stub
}


Logger::~Logger()
{
	// TODO Auto-generated destructor stub
}

void Logger::CreateLogFile()
{
	if(!this->fileManager.IsPathExists("/logs"))
		this->fileManager.CreateDirectory("logs");

	bool createLogFile = false;
	uint16_t counter = 0;

	char *buf = (char*)malloc(100*sizeof(char));
	while(!createLogFile && counter < 100000)
	{
		sprintf(buf, "logs/log-file-%d.log", counter);

		if(!this->fileManager.IsPathExists(buf))
		{
			FRESULT result = this->fileManager.CreateFile(buf, false);

			if(result == FR_OK)
			{
				this->filePath = buf;
				createLogFile = true;
			}
		}

		counter++;
	}

	if(createLogFile)
		this->SuccessMonitor();
	else
		this->ErrorMonitor();

	free(buf);
}

void Logger::ErrorMonitor()
{
	HAL_GPIO_WritePin(this->errorGPIO, this->errorPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(this->successGPIO, this->successPin, GPIO_PIN_RESET);
}

void Logger::SuccessMonitor()
{
	HAL_GPIO_WritePin(this->errorGPIO, this->errorPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(this->successGPIO, this->successPin, GPIO_PIN_SET);
}

void Logger::Info(const char* message)
{
	uint32_t bufferSize = (200 + strlen(message)) * sizeof(char);
	char *buffer = (char*)malloc(bufferSize);
	sprintf(buffer, "{ \"timestamp\": \"%lu\", \"logger\": \"%s\", \"level\": \"%s\", \"message\": \"%s\" }",
			HAL_GetTick(),
			this->loggerName,
			"Info",
			message);

	FRESULT result = this->fileManager.AppendLineToFile(this->filePath, buffer, false);

	if(result == FR_OK)
		this->SuccessMonitor();
	else
		this->ErrorMonitor();

	free(buffer);
}
