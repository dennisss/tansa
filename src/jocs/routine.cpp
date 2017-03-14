
#include "tansa/routine.h"
#include "tansa/csv.h"
#include "tansa/jocsParser.h"

using namespace std;


namespace tansa {


bool hasExtension(string path, const char *ext) {
	return path.find(ext) != std::string::npos;
}

bool Routine::IsFile(string path) {
	return hasExtension(path, ".csv") || hasExtension(path, ".jocs");
}

Routine *Routine::Load(string path, double scale) {

	Routine *p = NULL;

	if(hasExtension(path, ".csv")) {
		p = parse_csv(path.c_str(), scale);
	}
	else if(hasExtension(path, ".jocs")) {
		p = Jocs::Parse(path, scale);
	}


	// Otherwise throw exception?
	return p;

}

double Routine::duration() {

	double maxTime = 0;
	for(int i = 0; i < actions.size(); i++) {
		if(actions[i].size() > 0) {
			double t = actions[i][actions[i].size() - 1]->GetEndTime();
			if(t > maxTime)
				maxTime = t;
		}
	}

	return maxTime;
}



}
