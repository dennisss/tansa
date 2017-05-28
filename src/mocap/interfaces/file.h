#ifndef TANSA_MOCAP_INTERFACES_FILE_H_
#define TANSA_MOCAP_INTERFACES_FILE_H_

#include "../camera_node.h"
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

private:

	cv::Mat gray;

};


}


#endif
