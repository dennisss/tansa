#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ctime>

#ifdef RASPBERRY_PI
#include <raspicam/raspicam.h>
#endif


using namespace std;
using namespace cv;


raspicam::Raspicam camera;

void raspicam_image_callback(void *) {

	Image img;
	img.data = (PixelType *) camera.getImageBufferData();
	img.width = camera.getWidth();
	img.height = camera.getHeight();

	node.handle_image(&img);
}

void start_processing() {

	camera.setFormat(RASPICAM_FORMAT_GRAY);
	camera.setCaptureSize(1920, 1080);
	camera.setFrameRate(10);
	camera.setShutterSpeed(8000);
	// TODO: setBrightness, setSharpness, setContrast, setISO, setSaturation, setSaturation, setAWB_RB, setImageEffect
	camera.setAWB(RASPICAM_AWB_OFF);
	camera.setImageEffect(RASPICAM_IMAGE_EFFECT_NONE);
	camera.setVideoStabilization(false);
	camera.setUserCallback(raspicam_image_callback, NULL);
	camera.open();
}

void stop_processing() {
	camera.release();
}




int main(int argc, char** argv) {

	// Start server for

	//if(argc != 2) {
	//	printf("usage: DisplayImage.out <Image_Path>\n");
	//	return -1;
	//}

	Mat img;
	img = imread("/Users/dennis/Workspace/tansa/ext/pi-ir/verylong.jpg", 1);

	if(!img.data) {
		printf("No image data \n");
		return -1;
	}

	Mat gray(img.rows, img.cols, CV_8UC1);
	cvtColor(img, gray, CV_BGR2GRAY);



	Image input;
	input.data = &gray.at<uint8_t>(0,0);
	input.width = img.cols;
	input.height = img.rows;

	Image mask;
	mask.width = input.width;
	mask.height = input.height;
	mask.data = (uint8_t *) malloc(mask.width * mask.height);
	memset(mask.data, 0xff, mask.width * mask.height);


	vector<ImageBlob> blobs;

	DisjointSets forest(mask.width * mask.height);






	cout << "Found: " << blobs.size() << endl;


	for(size_t i = 0; i < blobs.size(); i++) {
		cout << blobs[i].cx << " " << blobs[i].cy << ": " << blobs[i].area << endl;

		Point center(cvRound(blobs[i].cx), cvRound(blobs[i].cy));
		int radius = cvRound(blobs[i].radius);
		// circle center
		circle(img, center, 3, Scalar(0,255,0), -1, 8, 0);
		// circle outline
		circle(img, center, radius, Scalar(0,0,255), 3, 8, 0);
	}



	//img = gray;

	Mat imageSmall;
	resize(img, imageSmall, cv::Size(img.cols * 0.3, img.rows * 0.3), 0, 0, CV_INTER_LINEAR);

	namedWindow("Blobs", WINDOW_AUTOSIZE);
	imshow("Blobs", imageSmall);
	waitKey(0);

	return 0;
}
