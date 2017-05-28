#ifndef TANSA_MOCAP_INTERFACES_RASPICAM_H_
#define TANSA_MOCAP_INTERFACES_RASPICAM_H_

#include "../node.h"

#include <raspicam/raspicam.h>


namespace tansa {

class RaspicamImagingInterface : public MocapCameraImagingInterface {
public:

	virtual ~RaspicamImagingInterface();

	virtual void start();
	virtual void stop();

private:
	raspicam::Raspicam camera;
};

void raspicam_image_callback(void *arg);


}


#endif
