#include "fatfs.h"
#include "string.h"

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
		FRESULT AppendToFile(const char* name, char* data, bool force);
		FRESULT AppendLineToFile(const char* name, char* data, bool force);
		FRESULT CreateDirectory(const char* name);
		uint32_t GetFreeSpace();
		//TODO Добавить ReadFile с заданным объемом
};
