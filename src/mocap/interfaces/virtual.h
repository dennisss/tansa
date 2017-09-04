#ifndef TANSA_MOCAP_INTERFACES_VIRTUAL_H_
#define TANSA_MOCAP_INTERFACES_VIRTUAL_H_

#include "../camera_node.h"
#include <tansa/vision/camera_model.h>
#include <tansa/graphics.h>

#include <thread>

namespace tansa {

/**
 * This is designed to simulate a motion capture camera by rendering images of a simulation world state
 */
class VirtualImagingInterface : public MocapCameraImagingInterface {
public:

	// TODO: Should also take in a camera hardware configuration to simulate
	/**
	 * Creates a new virtual camera
	 *
	 * NOTE: It must be created in the main thread
	 */
	VirtualImagingInterface(const CameraModel &cam /* Simulation *sim */);
	virtual ~VirtualImagingInterface();

	virtual void start();
	virtual void stop();

	virtual ImageSize getSize();


private:

	friend void virtual_imaging_thread(VirtualImagingInterface *inst);

	void setup_camera();

	std::thread thread;
	Image img;


	graphics::Window *window;
	double scale = 1.0;

	CameraModel camera;
};


void virtual_imaging_thread(VirtualImagingInterface *inst);


}

#endif
