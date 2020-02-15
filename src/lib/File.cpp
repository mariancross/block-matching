#include "File.h"
using namespace std;

File::File() { 

}

File::~File() { 

}

vector<string> File::getEntryList(string dirPath) {
	vector<string> entryList;
	
	DIR* dir = opendir( dirPath.c_str() );
	dirent* entry;
	
	while( ( entry = readdir(dir) ) != NULL ) {
		string file = string( entry->d_name);
		
		if( file.rfind(".pgm") != string::npos ) {
			string absolutePath = string(dirPath);
			absolutePath.append(file);
			
			entryList.push_back(absolutePath);
		}
	}
	
	delete entry;
	entry = 0;
	
	return entryList;
}

