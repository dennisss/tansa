#include "camera_node.h"
#include "interfaces/file.h"
#ifdef USE_RPI
#include "interfaces/raspicam.h"
#endif

#include <unistd.h>
#include <iostream>



using namespace std;
using namespace tansa;

static bool running = false;
static MocapCameraNode *node;
static MocapCameraImagingInterface *interface = NULL;


void signal_sigint(int s) {
	running = false;
}

void usage() {
	cout << "Usage: ./mocap_camera [-file image.png] [-raspicam] [-lport N] [-rport N]" << endl
		 << "       At most one of -file and -raspicam can be specified" << endl;
	exit(1);
}

// "/Users/dennis/Workspace/tansa/ext/pi-ir/verylong.jpg"
int main(int argc, char** argv) {

	// Parse cli arguments
	for(int i = 1; i < argc; i++) {
		if(strcmp(argv[i], "-file") == 0) {
			if(interface != NULL) {
				printf("To many interfaces selected\n");
				return 1;
			}
			else if(i + 1 >= argc) {
				printf("Missing filename\n");
				return 1;
			}

			interface = new FileImagingInterface(argv[i + 1]);
			i++;
		}
#ifdef USE_RPI
		else if(strcmp(argv[i], "-raspicam")) {
			if(interface != NULL) {
				printf("To many interfaces selected\n");
				return 1;
			}

			interface = new RaspicamImagingInterface();
		}
#endif
		else {
			printf("Unknown option: %s\n", argv[i]);
			return 1;
		}

	}

	if(interface == NULL) {
		printf("No physical interface selected!\n");
		return 1;
	}


	// Select and init interface
	node = new MocapCameraNode(interface);


	// Run the server
	signal(SIGINT, signal_sigint);
	running = true;
	node->connect();

	while(running) {
		usleep(1000);
	}

	node->disconnect();


	delete node;
	delete interface;

	return 0;
}
