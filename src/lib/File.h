/**
  * @class File
  * @author Maria Santamaria 
  * @date June 2012
  */

#ifndef FILE_H
#define FILE_H

#include <fstream>
#include <string>
#include <vector>  
#include <dirent.h>
#include <sys/types.h>

class File {
	private:
	
	protected:
	
	public:
		File();		
		~File();
		
		static std::vector<std::string> getEntryList(std::string dirPath);
};

#endif

