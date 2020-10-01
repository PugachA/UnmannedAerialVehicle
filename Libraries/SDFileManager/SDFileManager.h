#include "fatfs.h"
#include "string.h"

#ifndef SDFileManager_H_
#define SDFileManager_H_

class SDFileManager
{
	private:
		const TCHAR* path;
		FATFS fileSystem;

	public:
		SDFileManager(const TCHAR* path);
		virtual ~SDFileManager();

		FRESULT MountSD();
		FRESULT UnMountSD();
		FRESULT CreateFile(const char* name, bool force);
		FRESULT RemoveFile(const char* name);
		int AppendToFile(const char* name, char* data, bool force);
		int AppendLineToFile(const char* name, char* data, bool force);
		FRESULT CreateDirectory(const char* name);
		uint32_t GetFreeSpace();
		bool IsPathExists(const char* name);
		//TODO Добавить ReadFile с заданным объемом
};
#endif /* SDFileManager_H_ */
