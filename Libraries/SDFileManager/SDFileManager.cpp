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

FRESULT SDFileManager::CreateFile(const char* name, bool force)
{
	FILINFO fileInfo;
	FRESULT fileResult = f_stat(name, &fileInfo);

	if(fileResult == FR_OK && !force)
		return FR_NO_FILE;

	FIL file;
	fileResult = f_open(&file, name, FA_CREATE_ALWAYS|FA_READ|FA_WRITE);

	if (fileResult != FR_OK)
		return fileResult;

	fileResult = f_close(&file);

    return fileResult;
}
