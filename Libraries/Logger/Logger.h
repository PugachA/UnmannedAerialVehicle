/*
 * Logger.h
 *
 *  Created on: Jul 29, 2020
 *      Author: foton
 */

#include "SDFileManager\SDFileManager.h"
#include "stdio.h"

#ifndef Logger_H_
#define Logger_H_

class Logger
{
	public:
		Logger(const char* loggerName,
				SDFileManager& fileManager);
		virtual ~Logger();

	private:
		const char* loggerName;
		char* filePath;
		SDFileManager fileManager;
		GPIO_TypeDef* successGPIO;
		uint16_t successPin;
		GPIO_TypeDef* errorGPIO;
		uint16_t errorPin;

		void CreateLogFile();
};

#endif /* Logger_H_ */
