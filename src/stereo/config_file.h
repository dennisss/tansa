#ifndef CONF_FILE_H_
#define CONF_FILE_H_

#include <map>
#include <string>

using namespace std;

/**
 * Windows style .ini or .conf files with sections and properties
 */
class ConfigFile {
public:

	static Read(char *filename);

	string get(string section, string key);

private:
	ConfigFile();
	map<string, map<string, string>> data;


}


#endif
