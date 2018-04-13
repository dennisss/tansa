#ifndef TANSA_MOCAP_BLOB_DETECTOR
#define TANSA_MOCAP_BLOB_DETECTOR

#include <tansa/vision/image.h>
#include <tansa/algorithm.h>

#include <string.h>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>


namespace tansa {

#define MAX_BLOBS 255
#define MAX_PIXELS_PER_BLOB 2000


typedef uint8_t PixelType;

// This allows reuse of the image memory for storing labels but restricted a single camera to track up to 254 blobs
typedef PixelType LabelType;

typedef unsigned PixelIndexType;



struct ImageBlob {
	unsigned id; /**< Unique to this blob of is the index of the very first pixel found that is in this blob */
	std::vector<PixelIndexType> indices; /**< Indices of all pixels included in the blob */

	unsigned area = 0;
	unsigned perimeter;
	unsigned cx = 0;
	unsigned cy = 0;
	unsigned radius = 0;
};

// TODO: Move this code to the vision library
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

	friend void blob_detector_thread(BlobDetector *d, int num);

	void notifyWorker(int i);
	void waitForWorker(int i);

	// Process a single part of the image
	void process_region(Image *img, ImageSubRegion *region, std::vector<ImageBlob> *blobs);

	// Detection pipeline

	/**
	 * Step 1: Inplace thresholding of an image and masking such a pixel is zeroed if:
	 * - it's value is less than or equal to the threshold
	 * - or the corresponding entry in the map is zero
	 * NOTE: Parallelized across threads
	 */
	void threshold_n_mask(Image *img, ImageSubRegion *region);

	/**
	 * Step 2: Pass 1: Combining pixels based on connectivity
	 * NOTE: Parallelized across all threads
	 *
	 * @param img
	 * @param region
	 * @param init whether or not this should regenerate sets for each next pixel (what should happen with a new image)
	 */
	void connected_components_combine(Image *img, ImageSubRegion *region, bool init = true);

	/**
	 * Step 2: Pass 2: Generate a list of blobs from the connected component data
	 * NOTE: This will modify the image inplace
	 * NOTE: This is parallelized across all threads
	 */
	void connected_components_extract(Image *img, ImageSubRegion *region, std::vector<ImageBlob> *blobs);

	/**
	 * Step 2: Part 3: Not another pass but instead this just combines all the results gathered by the workers in the main thread
	 */
	void combine_all_blobs(Image *img, std::vector<ImageBlob> *blobs);

	/**
	 * Step 3: Generate useful metrics about the blobs
	 */
	void postprocess_components(Image *img, std::vector<ImageBlob> *blobs);

	// Options
	PixelType threshold;
	Image mask;

	DisjointSets forest;


	struct Worker {
		Worker() {}
		Worker(const Worker &w) : ready(w.ready), done(w.done) {}

		std::thread thread;
		std::mutex mtx;
		std::condition_variable cvar;
		std::vector<ImageBlob> blobs;

		bool ready = false; /**< Whether or not there is data ready for this thread to process */
		bool done = false; /**< Whether or not it is done processing the data */
	};

	bool running = false; /**< Whether or not the threads should be running */
	Image *activeImage; /**< Reference to the current image being processed. To be shared across all worker threads */
	std::vector<ImageSubRegion> regions; /**< All parts of the image that need to be processed. This is the division for each thread */
	std::vector<ImageSubRegion> seams; /**< Regions between the regions which need to be re-processed to combine the components */
	std::vector<Worker> workers;

};

void blob_detector_thread(BlobDetector *d, int num);



}



#endif
