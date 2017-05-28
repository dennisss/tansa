#ifdef USE_RPI

#include "raspicam.h"

namespace tansa {

// TODO: Look into STC clock mode in the MMAL stuff

RaspicamImagingInterface::RaspicamImagingInterface() {
	camera.setUserCallback(raspicam_image_callback, this);
}


void RaspicamImagingInterface::start() {

	camera.setFormat(RASPICAM_FORMAT_GRAY);
	camera.setCaptureSize(1640, 1232);
	camera.setFrameRate(10);
	camera.setShutterSpeed(8000);
	// TODO: setBrightness, setSharpness, setContrast, setISO, setSaturation, setSaturation, setAWB_RB, setImageEffect
	camera.setAWB(RASPICAM_AWB_OFF);
	camera.setImageEffect(RASPICAM_IMAGE_EFFECT_NONE);
	camera.setVideoStabilization(false);

	// TODO: Set exposure compensation to 0
	// TODO: Set exposure mode to 'auto'
	// TODO: Set ISO

	camera.open();

}

void RaspicamImagingInterface::stop() {
	camera.release();
}


void raspicam_image_callback(void *arg) {

	RaspicamImagingInterface *self = (RaspicamImagingInterface *) arg;

	Image img;
	img.data = (PixelType *) camera.getImageBufferData();
	img.width = camera.getWidth();
	img.height = camera.getHeight();

	node.handle_image(&img);
}




}

#endif
