#include "blob_detector.h"

#ifdef __linux__
#include <sched.h>
#endif
#include <algorithm>

using namespace std;


namespace tansa {

BlobDetector::BlobDetector(unsigned width, unsigned height, unsigned nthreads) : forest(width*height) {
	mask.width = width;
	mask.height = height;
	mask.data = (PixelType *) malloc(width * height);
	this->clear_mask();


	unsigned linesPerThread = height / nthreads;
	if((height % nthreads) != 0)
		linesPerThread++;

	running = true;
	workers.resize(nthreads);

	for(unsigned i = 0; i < nthreads; i++) {
		ImageSubRegion r;
		unsigned line = i*linesPerThread;
		r.offset = line * width;
		r.start = 0;
		r.size = min(linesPerThread, height - line) * width;
		regions.push_back(r);

		if(i > 0) {
			ImageSubRegion s;
			s.offset = (line - 1) * width;
			s.start = width;
			s.size = 2 * width;
			seams.push_back(s);
		}

		workers[i].blobs.reserve(MAX_BLOBS);

		workers[i].thread = move(std::thread(blob_detector_thread, this, i));
	}
}

BlobDetector::~BlobDetector() {
	running = false;
	for(int i = 0; i < workers.size(); i++) {
		workers[i].cvar.notify_one();
		workers[i].thread.join();
	}

	free(mask.data);
}


void blob_detector_thread(BlobDetector *d, int num) {

#ifdef __linux__
	// Assigning this thread to run on a unique core
	cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(num, &set);
	sched_setaffinity(0, sizeof(cpu_set_t), &set);
#endif

	BlobDetector::Worker *w = &d->workers[num];
	ImageSubRegion *reg = &d->regions[num];
	std::vector<ImageBlob> *blobs = &w->blobs;

	while(true) {
		// Wait for a notice from the master
		std::unique_lock<std::mutex> lock(w->mtx);
		w->cvar.wait(lock, [d, w](){ return !d->running ||  w->ready; });

		if(!d->running) {
			break;
		}

		// Otherwise, must be ready

		// Do work
		d->process_region(d->activeImage, reg, blobs);


		// Notify master that we are done
		w->ready = false;
		w->done = true;
		w->cvar.notify_one();
	}

}




void BlobDetector::detect(Image *img, vector<ImageBlob> *blobs) {


	// TODO: This may fail miserably if there is a blob that starts at pixel 0

	unsigned i = 0;

	this->activeImage = img;

	// Notify all threads to get to work
	for(i = 0; i < workers.size(); i++) {
		notifyWorker(i);
	}

	// Do our part
	//process_region(img, &regions[0]);

	// Wait for all threads
	// TODO: If >1 seam is available, then it can be done in parallel by the second thread in that set
	for(i = 0; i < workers.size(); i++) {
		waitForWorker(i);
	}

	// Combine along seams
	for(i = 0; i < seams.size(); i++) {
		connected_components_combine(img, &seams[i], false);
	}

	// Lastly combine the blobs that were found by all the child thread
	combine_all_blobs(img, blobs);

	postprocess_components(img, blobs);

	//component_filter(img, blobs);
}

inline void BlobDetector::process_region(Image *img, ImageSubRegion *region, std::vector<ImageBlob> *blobs) {
	threshold_n_mask(img, region);
	connected_components_combine(img, region);
	connected_components_extract(img, region, blobs);
}


inline void BlobDetector::notifyWorker(int i) {
	Worker *w = &workers[i];

	std::unique_lock<std::mutex> lock(w->mtx);
	w->ready = true;
	w->done = false;
	w->cvar.notify_one();
}

inline void BlobDetector::waitForWorker(int i) {
	Worker *w = &workers[i];

	std::unique_lock<std::mutex> lock(w->mtx);
	if(!w->done) {
		w->cvar.wait(lock, [w]() { return w->done; });
	}
}



void BlobDetector::auto_mask(Image *next) {
	ImageSubRegion r = next->all();
	threshold_n_mask(next, &r);

	for(unsigned i = 0; i < next->width * next->height; i++) {
		if(next->data[i] != 0) {
			mask.data[i] = 0;
		}
	}
}


void BlobDetector::clear_mask() {
	memset(mask.data, 0xff, mask.width * mask.height);
}

void BlobDetector::threshold_n_mask(Image *img, ImageSubRegion *region) {
	unsigned i;
	unsigned N = region->size - region->start; //img->height * img->width;
	PixelType thresh = this->threshold;

	PixelType *p = img->data + (region->offset + region->start);
	PixelType *mp = mask.data + (region->offset + region->start);

	for(i = 0; i < N; ++i) {
		if(*p <= thresh || *mp == 0) {
			*p = 0;
		}
		p++;
		mp++;
	}
}


// Should support 16bit number of components
// We should reject components with only one pixel
// Input: a binary image (8bit)
// Output: a labeling (16bit)
void BlobDetector::connected_components_combine(Image *img, ImageSubRegion *region, bool init) {

	unsigned N = region->offset + region->size; //img->width * img->height;

	// Previous neighbors in 8-connectivity
	// These are ordered to go from lowest to highest index
	unsigned negativeOffsets[] = {
		img->width + 1, // up-left
		img->width, // up
		img->width - 1, // up-right
		1 // left
	};

	// First pass
	// TODO: This can be parallelized easily by splitting up the image vertically and then doing a combine along all the edges once
	for(unsigned i = (region->offset + region->start); i < N; i++) {
		uint8_t *p = img->data + i;

		// Ignore background
		if(*p == 0)
			continue;


		bool hasNeighbors = false;
		unsigned firstLabel = 0;
		for(unsigned j = 0; j < 4; j++) {
			uint8_t *pj = p - negativeOffsets[j];

			// Skip invalid (must stay inside of the region) or background neighbors
			if(!(negativeOffsets[j] <= (i - region->offset) && *pj != 0))
				continue;

			unsigned nIdx = i - negativeOffsets[j];

			// TODO: Because the DisjointSets keeps tracks of mins, we don't need to do that here
			if(!hasNeighbors) { // This is the first neighbor
				firstLabel = forest.findSet(nIdx);
				hasNeighbors = true;
			}
			else { // Merge the other
				forest.unionSets(firstLabel, nIdx);
			}

		}

		if(init) {
			forest.makeSet(i);
		}

		if(hasNeighbors) { // Merge with min neighbor
			forest.unionSets(firstLabel, i);
		}

	}

}

void BlobDetector::connected_components_extract(Image *img, ImageSubRegion *region, std::vector<ImageBlob> *blobs) {

	unsigned N = region->offset + region->size; //img->width * img->height;

	blobs->resize(0);

	// Second pass
	// Assigning ordered labels to each number
	// This converts the image into a labeling from 1-M where M is the number of components
	LabelType nextLabel = 1;
	LabelType *labels = img->data;
	for(unsigned i = (region->offset + region->start); i < N; i++) {
		if(img->data[i] == 0) // Skip background pixels
			continue;

		unsigned s = forest.findSetMin(i);

		if(s == i) { // We've encountered the first element of the set

			// More labels than allowed
			if(nextLabel == 0xff) {
				labels[i] = 0;
				continue;
			}

			labels[i] = nextLabel++;

			// TODO: Keep blobs at an allocated size of 255 at all times
			blobs->push_back(ImageBlob());
			(*blobs)[labels[i] - 1].id = s;
			(*blobs)[labels[i] - 1].indices.push_back(i);
		}
		else { // Otherwise, choose the label of the root

			labels[i] = labels[s]; // TODO: This will likely do a lot of cache invalidation

			// If part of an overflowed segment, stop
			if(labels[i] == 0 || (*blobs)[labels[i] - 1].indices.size() > MAX_PIXELS_PER_BLOB) {
				continue;
			}

			(*blobs)[labels[i] - 1].indices.push_back(i);
		}
	}

}

inline void BlobDetector::combine_all_blobs(Image *img, std::vector<ImageBlob> *blobs) {
	// This works because the blob extractor sets the values of the image pixels equal to the index of the blob in the the worker's blob collection

	// TODO: This function does not enforce our limit on the number of blobs and indices

	*blobs = workers[0].blobs;
	for(unsigned i = 1; i < workers.size(); i++) {

		std::vector<ImageBlob> &subblobs = workers[i].blobs;

		for(unsigned j = 0; j < subblobs.size(); j++) {
			unsigned setId = forest.findSetMin(subblobs[j].id);
			if(setId != subblobs[j].id) { // This blob was merged with an earlier blob
				unsigned idx = img->data[setId] - 1;

				img->data[subblobs[j].id] = idx; // TODO: This should not be needed
				(*blobs)[idx].indices.insert( (*blobs)[idx].indices.end(), subblobs[j].indices.begin(), subblobs[j].indices.end() );
			}
			else { // Brand new blob
				img->data[setId] = blobs->size();
				blobs->push_back(subblobs[j]);
			}
		}
	}
}

void BlobDetector::postprocess_components(Image *img, vector<ImageBlob> *blobs) {


	for(unsigned i = 0; i < blobs->size(); i++) {
		ImageBlob &b = (*blobs)[i];

		b.area = b.indices.size();

		// First pass find centroid
		for(unsigned j = 0; j < b.indices.size(); j++) {
			unsigned idx = b.indices[j];
			unsigned x = idx % img->width,
					 y = idx / img->width;

			b.cx += x;
			b.cy += y;
		}

		b.cx /= b.area;
		b.cy /= b.area;


		// Second pass find the radius
		for(unsigned j = 0; j < b.indices.size(); j++) {
			unsigned idx = b.indices[j];
			unsigned x = idx % img->width,
					 y = idx / img->width;

			int dx = x - b.cx,
			 	dy = y - b.cy;

			unsigned r = (dx*dx) + (dy*dy);

			if(r > b.radius) {
				b.radius = r;
			}
		}

		b.radius = sqrt(b.radius);

	}

}


// TODO: Circularity filter, area filter, filter circles that are very close to each other, filter circles enclosed in other circles?

void component_filter(Image *img, vector<ImageBlob> *blobs) {

	for(unsigned i = 0; i < blobs->size(); i++) {
		//if(blobs[i])


	}


}


}
