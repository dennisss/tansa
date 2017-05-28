#ifdef USE_OPENCV

#include "file.h"

using namespace std;


namespace tansa {

FileImagingInterface::FileImagingInterface(const char *path) {
	cv::Mat img;
	img = cv::imread(path, 1);

	if(!img.data) {
		printf("No image data \n");
		return;
	}

	cv::cvtColor(img, gray, CV_BGR2GRAY);
}

void FileImagingInterface::start() {
	Image input;
	input.data = &gray.at<uint8_t>(0,0);
	input.width = gray.cols;
	input.height = gray.rows;

	MocapCameraImage msg;
	msg.image = &input;

	this->publish(msg);
}


void FileImagingInterface::stop() {


}

ImageSize FileImagingInterface::getSize() {
	ImageSize s;
	s.width = gray.cols;
	s.height = gray.rows;
	return s;
}


}


#endif
