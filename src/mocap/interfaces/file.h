#ifndef TANSA_MOCAP_INTERFACES_FILE_H_
#define TANSA_MOCAP_INTERFACES_FILE_H_

#include "../camera_node.h"
#include <pthread.h>
#include <opencv2/opencv.hpp>


namespace tansa {

/**
 * A simple interface to test using a static image
 */
class FileImagingInterface : public MocapCameraImagingInterface {
public:

	FileImagingInterface(const char *path);

	virtual ~FileImagingInterface() {}


	virtual void start();
	virtual void stop();

	virtual ImageSize getSize();

	virtual int getModel() {
		return 0;
	}
	
	virtual std::string getSerial() {
		return "";
	}

private:

	friend void *mocap_file_imaging_thread(void *);

	void publish_image();

	bool isImage;

	cv::Mat gray;

	cv::VideoCapture video;
	pthread_t thread;
};


void *mocap_file_imaging_thread(void *);


}


#endif
