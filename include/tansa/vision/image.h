#ifndef TANSA_VISION_IMAGE_H_
#define TANSA_VISION_IMAGE_H_

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

namespace tansa {


enum ImageFormat {
	ImageGrayscale,
	ImageRGB
};

enum ImageOutputFormat {
	ImageJPEG
};

struct ImageSubRegion {
	unsigned offset; /**< Start pixel offset of very beginning of this region */
	unsigned start; /**< Start pixel index relative to 'offset' of the first pixel which should be processed */
	unsigned size; /**< Relative to offset, this is the total number of pixels in this part of the image */
};


// A grayscale image
struct Image {
	unsigned width;
	unsigned height;
	uint8_t depth; /**< Number of bytes per pixel */
	uint8_t *data;

	void copyTo(Image *other) {
		other->data = (uint8_t *) malloc(width*height);
		memcpy(other->data, this->data, width*height);
		other->width = width;
		other->height = height;
	}

	ImageSubRegion all() {
		ImageSubRegion r;
		r.offset = 0;
		r.start = 0;
		r.size = width * height;
		return r;
	}

	inline void flipY() {
		for(int i = 0; i < height / 2; i++) {
			for(int j = 0; j < width; j++) {
				int a = i*width + j,
					b = (height - i - 1)*width + j;

				char tmp = this->data[a];
				this->data[a] = this->data[b];
				this->data[b] = tmp;
			}
		}
	}

	/**
	 * Encodes the image in JPEG format and puts the result into the supplied buffer
	 * TODO: This currently only supports grayscale images
	 */
	void encodeJPEG(std::vector<char> *buffer)





};

struct ImageSize {
	unsigned width;
	unsigned height;
};



}


#endif
