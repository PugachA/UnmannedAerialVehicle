/*
 * Logger.h
 *
 *  Created on: Jul 29, 2020
 *      Author: foton
 */

#include "SDFileManager\SDFileManager.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"

#ifndef Logger_H_
#define Logger_H_

class Logger
{
	public:
		Logger(const char* loggerName,
				SDFileManager& fileManager,
				GPIO_TypeDef* monitorGPIO,
				uint16_t monitorPin);
		Logger(const char* fileName,
				const char* loggerName,
				SDFileManager& fileManager,
				GPIO_TypeDef* monitorGPIO,
				uint16_t monitorPin);
		virtual ~Logger();
		void Info(const char* message);
		void Error(const char* message);
		void Warn(const char* message);
		void Debug(const char* message);

	private:
		const char* loggerName;
		char filePath[150];
		char errorQueue[3000] = ""; //расчитываю на 10 записей по 300 символов
		uint8_t errorCount;
		SDFileManager fileManager;
		GPIO_TypeDef* monitorGPIO;
		uint16_t monitorPin;
		const char folderName[5] = "logs";

		void WriteToLog(const char* message, const char* messageType);
		void CreateLogFile(const char* fileName);
		void ErrorMonitor(const char* message);
		void SuccessMonitor();
};

#endif /* Logger_H_ */
