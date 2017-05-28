#ifndef TANSA_MOCAP_INTERFACES_RASPICAM_H_
#define TANSA_MOCAP_INTERFACES_RASPICAM_H_

#include "../camera_node.h"

#include <raspicam/raspicam.h>


namespace tansa {

class RaspicamImagingInterface : public MocapCameraImagingInterface {
public:

	RaspicamImagingInterface();
	virtual ~RaspicamImagingInterface();

	virtual void start();
	virtual void stop();

	virtual ImageSize getSize();

private:
	friend void raspicam_image_callback(void *arg);

	raspicam::RaspiCam camera;
};

void raspicam_image_callback(void *arg);


}


#endif
