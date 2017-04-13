
#include "tansa/routine.h"
#include "tansa/csv.h"
#include "tansa/jocsParser.h"

using namespace std;


namespace tansa {

bool hasExtension(string path, const char *ext) {
	std::string::size_type idx = path.find(ext);
	return idx != std::string::npos && idx == (path.size() - strlen(ext));
}

bool Routine::IsFile(string path) {
	return hasExtension(path, ".csv") || hasExtension(path, ".jocs") || hasExtension(path, ".js") || hasExtension(path, ".json");
}

Routine *Routine::Load(string path, double scale) {

	Routine *p = NULL;

	if(hasExtension(path, ".csv")) {
		p = parse_csv(path.c_str(), scale);
	}
	else if(hasExtension(path, ".jocs") || hasExtension(path, ".js") || hasExtension(path, ".json")) {
		p = Jocs::Parse(path, scale);
	}

	FeasibilityChecker fc;
	if(p != NULL && !fc.check(*p)) {
		delete p;
		return NULL;
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
