#include "camera_node.h"
#ifdef USE_OPENCV
#include "interfaces/file.h"
#endif
#ifdef USE_RPI
#include "interfaces/raspicam.h"
#endif
#ifdef BUILD_GRAPHICS
#include "interfaces/virtual.h"
#endif

#include <unistd.h>
#include <signal.h>
#include <iostream>



using namespace std;
using namespace tansa;

static bool running = false;
static unsigned id = 0;
static Context ctx;
static MocapCameraNode *node;
static MocapCameraImagingInterface *interface = NULL;


void signal_sigint(int s) {
	running = false;
	//ctx.notify();
}

void usage() {
	cout << "Usage: ./mocap_camera [-file image.png] [-raspicam] [-lport N] [-rport N] [-ip IP_OF_INTERFACE]" << endl
		 << "       At most one of -file and -raspicam can be specified" << endl;
	exit(1);
}

// "/Users/dennis/Workspace/tansa/ext/pi-ir/verylong.jpg"
int main(int argc, char** argv) {

	// Parse cli arguments
	for(int i = 1; i < argc; i++) {
		if(false) {

		}
#ifdef USE_OPENCV
		else if(strcmp(argv[i], "-file") == 0) {
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
#endif
#ifdef USE_RPI
		else if(strcmp(argv[i], "-raspicam") == 0) {
			if(interface != NULL) {
				printf("To many interfaces selected\n");
				return 1;
			}

			interface = new RaspicamImagingInterface();
		}
#endif
#ifdef BUILD_GRAPHICS
		else if(strcmp(argv[i], "-virtual") == 0) {
			CameraModel m = CameraModel::Default(id);
			interface = new VirtualImagingInterface(m);
		}
#endif
		else if(strcmp(argv[i], "-n") == 0) {
			if(i + 1 >= argc) {
				printf("Missing number\n");
				return 1;
			}

			id = atoi(argv[i + 1]);
			i++;
		}
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
	node = new MocapCameraNode(&ctx, interface, id);


	// Run the server
	signal(SIGINT, signal_sigint);
	running = true;
	node->connect(NULL, MOCAP_CAMERA_NODE_DEFAULT_PORT + id, MOCAP_CAMERA_MASTER_DEFAULT_PORT);


	Rate r(2);
	while(running) {
		ctx.poll();

		glfwPollEvents(); // Needed in order for the virtual window to respond to events (must be in the main thread)
		r.sleep();
	}


/*
	// TODO: Something in these causes a crash

	interface->stop();
	node->disconnect();


	delete node;
	delete interface;
*/
	return 0;
}
