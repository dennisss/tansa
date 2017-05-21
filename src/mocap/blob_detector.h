#ifndef TANSA_MOCAP_BLOB_DETECTOR
#define TANSA_MOCAP_BLOB_DETECTOR


namespace tansa {



typedef uint8_t PixelType;

// This allows reuse of the image memory for storing labels but restricted a single camera to track up to 254 blobs
typedef PixelType LabelType;

typedef unsigned PixelIndexType;


// A grayscale image
struct Image {
	unsigned width;
	unsigned height;
	PixelType *data;
};


struct ImageBlob {
	vector<PixelIndexType> indices; /**< Indices of all pixels included in the blob */

	unsigned area = 0;
	unsigned perimeter;
	unsigned cx = 0;
	unsigned cy = 0;
	unsigned radius = 0;
};

/**
 * Finds blobs : connected regions of similar pixels
 */
class BlobDetector {
public:


	/**
	 * Given a grayscale image, finds blobs in it
	 * NOTE: This modifies the image in place
	 */
	void detect(Image *img, vector<ImageBlob> *blobs);



private:



	// Detection pipeline
	void thresholdAndMask();


	// Options
	PixelType threshold;
	Image *mask;


	// Variables for the current detect() execution
	Image *img;



};




}



#endif
