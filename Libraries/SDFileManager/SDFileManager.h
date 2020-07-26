#include "fatfs.h"

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
		FRESULT AppendToFile(char* name, char* data);
		FRESULT CreateDirectory(char* name);
		FRESULT CheckSpace(char* name);
		//TODO Добавить ReadFile
};
