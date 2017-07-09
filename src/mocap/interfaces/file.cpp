#ifdef USE_OPENCV

#include "file.h"
#include <tansa/time.h>

using namespace std;


namespace tansa {

FileImagingInterface::FileImagingInterface(const char *path) {
	cv::Mat img;
	img = cv::imread(path, 1);

	if(img.data != NULL) {
		cv::cvtColor(img, gray, CV_BGR2GRAY);
		isImage = true;
		return;
	}


	video.open(path);
	if(video.isOpened()) {
		isImage = false;
		return;
	}


	printf("No data could be read from file: %s\n", path);
}

void FileImagingInterface::start() {

	if(!isImage) {
		mocap_file_imaging_thread(this);
		//if(pthread_create(&thread, NULL, mocap_file_imaging_thread, (void *) this) != 0) {
		//	// TODO: Alert that is failed
		//}
		return;
	}
	else {
		publish_image();
	}
}


void FileImagingInterface::publish_image() {
	cv::Mat g = gray.clone();

	Image input;
	input.data = &g.at<uint8_t>(0,0);
	input.width = g.cols;
	input.height = g.rows;

	// TODO: Copy the image
	MocapCameraImage *msg = new MocapCameraImage();
	msg->image = &input;

	this->publish(msg);
}

void FileImagingInterface::stop() {


}

ImageSize FileImagingInterface::getSize() {
	ImageSize s;
	if(isImage) {
		s.width = gray.cols;
		s.height = gray.rows;
	}
	else {
		s.width = video.get(CV_CAP_PROP_FRAME_WIDTH);
		s.height = video.get(CV_CAP_PROP_FRAME_HEIGHT);
	}
	return s;
}


void *mocap_file_imaging_thread(void *arg) {

	FileImagingInterface *self = (FileImagingInterface *) arg;

	int fps = self->video.get(CV_CAP_PROP_FPS);

	cv::Mat img;

	Rate r(fps);
	while(self->video.grab()) {
		self->video.retrieve(img);
		cv::cvtColor(img, self->gray, CV_BGR2GRAY);
		self->publish_image();

		r.sleep();
	}



}


}


#endif
