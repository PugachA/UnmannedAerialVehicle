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
				GPIO_TypeDef* successGPIO,
				uint16_t successPin,
				GPIO_TypeDef* errorGPIO,
				uint16_t errorPin);
		Logger();
		virtual ~Logger();
		void Info(const char* message);
		void Error(char* message);
		void Warn(char* message);
		void Debug(char* message);

	private:
		const char* loggerName;
		char* filePath;
		SDFileManager fileManager;
		GPIO_TypeDef* successGPIO;
		uint16_t successPin;
		GPIO_TypeDef* errorGPIO;
		uint16_t errorPin;

		void CreateLogFile(const char* fileName);
		void ErrorMonitor();
		void SuccessMonitor();
};

#endif /* Logger_H_ */
