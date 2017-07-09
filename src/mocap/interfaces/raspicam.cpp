#ifdef USE_RPI

#include "raspicam.h"

namespace tansa {

// TODO: Look into STC clock mode in the MMAL stuff

RaspicamImagingInterface::RaspicamImagingInterface() {
	camera.setUserCallback(raspicam_image_callback, this);
}

RaspicamImagingInterface::~RaspicamImagingInterface() {
	this->stop();
}

/*
	Replicates this command:

	raspivid -n -ex auto -ISO 600 -ifx none -ss 8000 -w 1640 -h 1232 -o video.h264 -t 30000 -awb off -awbg 1,1 -fps 40 -ev 0 -br 50 -sa 0 -sh 0
*/
void RaspicamImagingInterface::start() {

	camera.setFormat(raspicam::RASPICAM_FORMAT_GRAY);
	camera.setCaptureSize(1640, 1232);
	camera.setFrameRate(1);
	camera.setShutterSpeed(8000);
	camera.setAWB(raspicam::RASPICAM_AWB_OFF);
	camera.setAWB_RB(1, 1); // TODO:
	camera.setImageEffect(raspicam::RASPICAM_IMAGE_EFFECT_NONE);
	camera.setVideoStabilization(false);
	camera.setBrightness(50);
	camera.setSaturation(0);
	camera.setSharpness(0);
	camera.setContrast(0);
	camera.setISO(600);
	camera.setExposureCompensation(0);
	camera.setExposure(raspicam::RASPICAM_EXPOSURE_AUTO); // auto mode from what I can tell is required for it to respect

	camera.open();

}

void RaspicamImagingInterface::stop() {
	camera.release();
}

ImageSize RaspicamImagingInterface::getSize() {
	ImageSize s;
	// TODO: The width and height don't get locked until the camera is started
	s.width = 1640; // camera.getWidth();
	s.height = 1232; // camera.getHeight();
	return s;
}

void raspicam_image_callback(void *arg) {

	RaspicamImagingInterface *self = (RaspicamImagingInterface *) arg;

	Image img;
	img.data = (PixelType *) self->camera.getImageBufferData();
	img.width = self->camera.getWidth();
	img.height = self->camera.getHeight();

	MocapCameraImage *msg = new MocapCameraImage();
	msg->image = &img;
	self->publish(msg);
}




}

#endif
