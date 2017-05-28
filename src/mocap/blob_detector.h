#ifndef TANSA_MOCAP_BLOB_DETECTOR
#define TANSA_MOCAP_BLOB_DETECTOR

#include <tansa/algorithm.h>

#include <string.h>
#include <vector>


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

	void copyTo(Image *other) {
		other->data = (PixelType *) malloc(width*height);
		memcpy(other->data, this->data, width*height);
		other->width = width;
		other->height = height;
	}

};

struct ImageSize {
	unsigned width;
	unsigned height;
};


struct ImageBlob {
	std::vector<PixelIndexType> indices; /**< Indices of all pixels included in the blob */

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

	BlobDetector(unsigned width, unsigned height, unsigned nthreads = 1);
	~BlobDetector();

	/**
	 * Given a grayscale image, finds blobs in it
	 * NOTE: This modifies the image in place
	 */
	void detect(Image *img, std::vector<ImageBlob> *blobs);

	// Around 200 is recommended
	void set_threshold(PixelType thresh) { this->threshold = thresh; }
	PixelType get_threshold() { return this->threshold; }

	/**
	 * While no important blobs are in the frame, call this for a few sequential frames to block out all noisy regions
	 *
	 * NOTE: This is threshold dependent
	 */
	void auto_mask(Image *next); // TODO: This should have a start command, a few frames of capture, and then a mask set

	void set_mask(Image *mask); // TODO: This should copy it
	Image *get_mask() { return &mask; };
	void clear_mask();


private:

	// Detection pipeline
	void threshold_n_mask(Image *img);
	void connected_components(Image *img, std::vector<ImageBlob> *blobs);
	void postprocess_components(Image *img, std::vector<ImageBlob> *blobs);

	// Options
	PixelType threshold;
	Image mask;

	DisjointSets forest;

};




}



#endif
