/*
 * Logger.cpp
 *
 *  Created on: Jul 29, 2020
 *      Author: foton
 */

#include "Logger.h"

Logger::Logger(const char* loggerName,
		SDFileManager& fileManager) : loggerName(loggerName), fileManager(fileManager)
{
	CreateLogFile();
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

	free(buf);
}

Logger::~Logger()
{
	// TODO Auto-generated destructor stub
}

