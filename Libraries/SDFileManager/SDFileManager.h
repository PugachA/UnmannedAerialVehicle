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
		FRESULT RemoveFile(char* name);
		FRESULT AppendToFile(const char* name, char* data, bool force);
		FRESULT AppendLineToFile(const char* name, char* data, bool force);
		FRESULT CreateDirectory(char* name);
		FRESULT CheckSpace(char* name);
		//TODO Добавить ReadFile
};
