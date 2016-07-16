#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//using namespace std;
using namespace cv;

#include "config_file.h"

/// Cached camera parameters
Mat rmap[2][2]; // Maps for undistorting/rectifying images




// Get the single camera instrinsics from the configuration file
void build_camera_matrix(ConfigFile &cfg, string name, Mat &cam, Mat &dist){

	cam = Mat::eye(3, 3, CV_32F);
	cam.at<float>(0,0) = cfg.get(name + ":fx");
	cam.at<float>(1,1) = cfg.get(name + ":fy");
	cam.at<float>(0,2) = cfg.get(name + ":cx");
	cam.at<float>(1,2) = cfg.get(name + ":cy");

	dist = Mat::zeros(4, CV_32F);
	dist.at<float>(0) = cfg.get(name, "k1");
	dist.at<float>(1) = cfg.get(name, "k2");
}

// Prepare for doing stereo matching
void init_mappings(ConfigFile &cfg){


	cfg.get("left_cam_2k");

	cfg.get("right_cam_2k");
}


void compute_disparity(Mat img, Mat &out){
	Mat gray, left, right;

	int width = img.cols / 2;

	// Change full image to grayscale
	cvtColor(img, gray, CV_BGR2GRAY);

	// Split the zed image into two halves
	Mat::copyTo(left, gray.colRange(0, width));
	Mat::copyTo(right, gray.colRange(width, 2*width));

	// Rectify
	for( k = 0; k < 2; k++ ){
		Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;
		remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
		cvtColor(rimg, cimg, COLOR_GRAY2BGR);
		Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
		resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
		if( useCalibrated )
		{
			Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
					  cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
			rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
		}
	}


	// Do stereo matching


}

void compute_cloud(Mat disp){


}


int main(int argc, char *argv[]){

	ConfigFile cfg = ConfigFile::Read("SN3166.conf");


	Mat img = imread("full.jpg"), disp;
	compute_disparity(img, disp);



}
