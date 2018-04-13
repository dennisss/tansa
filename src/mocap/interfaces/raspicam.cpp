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

	raspistill -n -ex auto -ISO 800 -ifx none -ss 8000 -w 1640 -h 1232 -o image.jpg -awb off -awbg 1,1 -ev 0 -br 50 -sa 0 -sh 0

	raspivid -n -ex auto -ISO 800 -ifx none -ss 8000 -w 1640 -h 1232 -o video.h264 -t 30000 -awb off -awbg 1,1 -fps 40 -ev 0 -br 50 -sa 0 -sh 0
*/
void RaspicamImagingInterface::start(const MocapCameraPacketConfig &config) {

	camera.setFormat(raspicam::RASPICAM_FORMAT_GRAY);
	camera.setCaptureSize(1640, 1232);
	camera.setFrameRate(config.framerate); // 40
	camera.setShutterSpeed(config.exposure); // 8000
	camera.setAWB(raspicam::RASPICAM_AWB_OFF);
	camera.setAWB_RB(1, 1); // TODO:
	camera.setImageEffect(raspicam::RASPICAM_IMAGE_EFFECT_NONE);
	camera.setVideoStabilization(false);
	camera.setBrightness(50);
	camera.setSaturation(0);
	camera.setSharpness(0);
	camera.setContrast(0);
	camera.setISO(800); // Can go up to 1600 in certain modes (may require using sports exposure mode)
	camera.setExposureCompensation(0);
	camera.setExposure(raspicam::RASPICAM_EXPOSURE_AUTO); // auto mode from what I can tell is required for it to respect

	// Force mode 4: up to ~40fps at 1296x972 (v1) or 1640x1232 (v2)
	camera.setSensorMode(4);
	// TODO: Also look into clock options and getting timestamps

	camera.open();

}

void RaspiCam::startVideoStream() {
	/*
	For doing the video streaming, it is probably easiest for us to spawn a child process using v4l2 and use that as
	http://www.lewisroberts.com/2015/05/15/raspberry-pi-mjpeg-at-30fps/

	Add bcm2835-v4l2 to /etc/modules
	sudo apt-get install vlc

	cvlc --no-audio v4l2:///dev/video0 --v4l2-width 1920 --v4l2-height 1080 --v4l2-chroma MJPG --v4l2-hflip 1 --v4l2-vflip 1 --sout '#standard{access=http{mime=multipart/x-mixed-replace;boundary=--7b3cc56e5f51db803f790dad720ed50a},mux=mpjpeg,dst=:8554/}' -I dummy

	cvlc --no-audio v4l2:///dev/video0 --v4l2-width 1920 --v4l2-height 1080 --v4l2-chroma h264 --v4l2-fps 30 --v4l2-hflip 1 --v4l2-vflip 1 --sout '#standard{access=http,mux=ts,dst=:8554}' -I dummy

	v4l2-ctl -c sharpness=30,compression_quality=100,video_bitrate_mode=1,video_bitrate=25000000
	*/


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

	// TODO: This is not thread safe
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
