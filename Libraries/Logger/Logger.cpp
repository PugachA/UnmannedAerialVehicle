/*
 * Logger.cpp
 *
 *  Created on: Jul 29, 2020
 *      Author: foton
 */

#include "Logger.h"

Logger::Logger(const char* loggerName,
		SDFileManager& fileManager,
		GPIO_TypeDef* monitorGPIO,
		uint16_t monitorPin) : loggerName(loggerName), fileManager(fileManager)
{
	this->monitorGPIO = monitorGPIO;
	this->monitorPin = monitorPin;
	this->errorCount = 0;

	uint8_t counter = 0;
	char fileNameBuf[100];
	char filePathBuf[150];
	bool flag = false;

	while(!flag)
	{
		sprintf(fileNameBuf, "log-file-%s-%d.log", this->loggerName, counter);
		sprintf(filePathBuf, "%s/%s",this->folderName, fileNameBuf);

		if(!this->fileManager.IsPathExists(filePathBuf))
			flag = true;

		counter++;
	}

	CreateLogFile(fileNameBuf);
}

Logger::Logger(const char* fileName,
		const char* loggerName,
		SDFileManager& fileManager,
		GPIO_TypeDef* monitorGPIO,
		uint16_t monitorPin) : loggerName(loggerName), fileManager(fileManager)
{
	this->monitorGPIO = monitorGPIO;
	this->monitorPin = monitorPin;
	this->errorCount = 0;

	CreateLogFile(fileName);
}

Logger::~Logger()
{}

void Logger::CreateLogFile(const char* fileName)
{
	sprintf(this->filePath, "/%s", this->folderName);

	if(!this->fileManager.IsPathExists(this->filePath))
		this->fileManager.CreateDirectory(this->folderName);

	sprintf(this->filePath, "%s/%s", this->folderName, fileName);

	if(!this->fileManager.IsPathExists(this->filePath))
	{
		FRESULT result = this->fileManager.CreateFile(this->filePath, false);

		if(result == FR_OK)
			this->SuccessMonitor();
		else
			this->ErrorMonitor("");
	}
}

void Logger::ErrorMonitor(const char* message)
{
	strcat(this->errorQueue, message);
	strcat(this->errorQueue, "\n");
	this->errorCount++;

	this->fileManager.UnMountSD();
	FRESULT result = this->fileManager.MountSD();

	if(result != FR_OK)
	  HAL_GPIO_WritePin(this->monitorGPIO, this->monitorPin, GPIO_PIN_SET);
}

void Logger::SuccessMonitor()
{
	HAL_GPIO_WritePin(this->monitorGPIO, this->monitorPin, GPIO_PIN_RESET);
}

void Logger::WriteToLog(const char* message, const char* messageType)
{
	uint32_t bufferSize = 200 + strlen(message);
	char buffer[bufferSize];
	sprintf(buffer, "timestamp:%lu;message:%s", HAL_GetTick(), message);

	int bytesWritten;
	if(this->errorCount > 0)
	{
		bytesWritten = this->fileManager.AppendLineToFile(this->filePath, this->errorQueue, false);

		if(bytesWritten != -1)
		{
			memset(this->errorQueue, 0, sizeof(this->errorQueue));
			this->errorCount = 0;
		}
	}

	bytesWritten = this->fileManager.AppendLineToFile(this->filePath, buffer, false);

	if(bytesWritten != -1)
		this->SuccessMonitor();
	else
		this->ErrorMonitor(buffer);
}

void Logger::Info(const char* message)
{
	WriteToLog(message, "Info");
}

void Logger::Error(const char* message)
{
	WriteToLog(message, "Error");
}

void Logger::Warn(const char* message)
{
	WriteToLog(message, "Warn");
}

void Logger::Debug(const char* message)
{
	WriteToLog(message, "Debug");
}
