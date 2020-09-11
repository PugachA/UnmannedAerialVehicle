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

	char* buf = new char[100*sizeof(char)];
	sprintf(buf, "log-file-%s.log", this->loggerName);

	CreateLogFile(buf);

	delete[] buf;
}

Logger::Logger(const char* fileName,
		const char* loggerName,
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

	CreateLogFile(fileName);
}

Logger::~Logger()
{
	delete[] filePath;
	// TODO Auto-generated destructor stub
}

void Logger::CreateLogFile(const char* fileName)
{
	if(!this->fileManager.IsPathExists("/logs"))
		this->fileManager.CreateDirectory("logs");

	this->filePath = new char[100*sizeof(char)];
	strcpy(this->filePath, "logs/");
	strcat(this->filePath, fileName);

	if(!this->fileManager.IsPathExists(this->filePath))
	{
		FRESULT result = this->fileManager.CreateFile(this->filePath, false);

		if(result == FR_OK)
			this->SuccessMonitor();
		else
			this->ErrorMonitor();
	}
}

void Logger::ErrorMonitor()
{
	this->fileManager.UnMountSD();
	FRESULT result = this->fileManager.MountSD();

	if(result != FR_OK)
	{
	  HAL_GPIO_WritePin(this->errorGPIO, this->errorPin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(this->successGPIO, this->successPin, GPIO_PIN_RESET);
	}
}

void Logger::SuccessMonitor()
{
	HAL_GPIO_WritePin(this->errorGPIO, this->errorPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(this->successGPIO, this->successPin, GPIO_PIN_SET);
}

void Logger::Info(const char* message)
{
	uint32_t bufferSize = (200 + strlen(message)) * sizeof(char);
	char *buffer = new char(bufferSize);
	sprintf(buffer, "{ \"timestamp\": \"%lu\", \"logger\": \"%s\", \"level\": \"%s\", \"message\": \"%s\" }",
			HAL_GetTick(),
			this->loggerName,
			"Info",
			message);

	int bytesWritten = this->fileManager.AppendLineToFile(this->filePath, buffer, false);

	if(bytesWritten != -1)
		this->SuccessMonitor();
	else
		this->ErrorMonitor();

	delete[] buffer;
}

void Logger::Error(const char* message)
{
	uint32_t bufferSize = (200 + strlen(message)) * sizeof(char);
	char *buffer = new char(bufferSize);
	sprintf(buffer, "{ \"timestamp\": \"%lu\", \"logger\": \"%s\", \"level\": \"%s\", \"message\": \"%s\" }",
			HAL_GetTick(),
			this->loggerName,
			"Error",
			message);

	int bytesWritten = this->fileManager.AppendLineToFile(this->filePath, buffer, false);

	if(bytesWritten != -1)
		this->SuccessMonitor();
	else
		this->ErrorMonitor();

	delete[] buffer;
}
