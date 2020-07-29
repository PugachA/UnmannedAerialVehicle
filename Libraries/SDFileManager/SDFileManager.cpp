#include "SDFileManager.h"

SDFileManager::SDFileManager(const TCHAR* path)
{
	this->path = path;
}

SDFileManager::~SDFileManager()
{}

FRESULT SDFileManager::MountSD()
{
	return f_mount(&this->fileSystem, this->path, 1);
}

FRESULT SDFileManager::UnMountSD()
{
	return f_mount(NULL, this->path, 1);
}

FRESULT SDFileManager::CreateFile(const char* name, bool force = false)
{
	FILINFO fileInfo;
	FRESULT fileResult = f_stat(name, &fileInfo);

	if(fileResult == FR_OK && !force)
		return FR_EXIST;

	FIL file;
	fileResult = f_open(&file, name, FA_CREATE_ALWAYS|FA_READ|FA_WRITE);

	if (fileResult != FR_OK)
	{
		f_close(&file);
		return fileResult;
	}

	fileResult = f_close(&file);

    return fileResult;
}

//Delete exiting file
FRESULT SDFileManager::RemoveFile(const char* name)
{
	FILINFO fileInfo;
	FRESULT fileResult = f_stat(name, &fileInfo);

	if (fileResult != FR_OK)
		return FR_NO_FILE;

	return f_unlink(name);
}

//Add data to file
FRESULT SDFileManager::AppendToFile(const char* name, char* data, bool force = false)
{
	FILINFO fileInfo;
	FRESULT fileResult = f_stat(name, &fileInfo);

	if (fileResult != FR_OK)
	 fileResult = this->CreateFile(name, force);

	FIL file;
	fileResult = f_open(&file, name, FA_OPEN_APPEND | FA_WRITE);
	if (fileResult != FR_OK)
	{
		f_close(&file);
		return fileResult;
	}

	UINT bytesWritten;
	fileResult = f_write(&file, data, strlen(data), &bytesWritten);
	if (fileResult != FR_OK)
	{
		f_close(&file);
		return fileResult;
	}

	fileResult = f_close(&file);

	return fileResult;
}

FRESULT SDFileManager::AppendLineToFile(const char* name, char* data, bool force = false)
{
	char *buf = (char *)malloc((strlen(data) + 2)*sizeof(char));
	strcpy(buf, data);
	strcat(buf, "\r\n");

	FRESULT fileResult = this->AppendToFile(name, buf, force);
	free(buf);

	return fileResult;
}

FRESULT SDFileManager::CreateDirectory(const char* name)
{
	return f_mkdir(name);
}

//Get free space in KB
uint32_t SDFileManager::GetFreeSpace()
{
	FATFS *pFileSystem;
	DWORD freeClustersCount;
    f_getfree(this->path, &freeClustersCount, &pFileSystem);

    return (uint32_t)(freeClustersCount * pFileSystem->csize * 0.5);
}

bool SDFileManager::IsPathExists(const char* name)
{
	FILINFO fileInfo;
	FRESULT fileResult = f_stat(name, &fileInfo);

	if (fileResult == FR_OK)
		return true;

	return false;
}
