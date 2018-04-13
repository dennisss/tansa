#include <tansa/vision/image.h>

#include <jpeglib.h>


namespace tansa {



void Image::encodeJPEG(std::vector<char> *buffer) {



}



std::vector<JOCTET> my_buffer;
#define BLOCK_SIZE 16384


/*
	Grab from &my_buffer[0], my_buffer.size())
*/


void my_init_destination(j_compress_ptr cinfo) {
    my_buffer.resize(BLOCK_SIZE);
    cinfo->dest->next_output_byte = &my_buffer[0];
    cinfo->dest->free_in_buffer = my_buffer.size();
}

boolean my_empty_output_buffer(j_compress_ptr cinfo) {
    size_t oldsize = my_buffer.size();
    my_buffer.resize(oldsize + BLOCK_SIZE);
    cinfo->dest->next_output_byte = &my_buffer[oldsize];
    cinfo->dest->free_in_buffer = my_buffer.size() - oldsize;
    return true;
}

void my_term_destination(j_compress_ptr cinfo) {
    my_buffer.resize(my_buffer.size() - cinfo->dest->free_in_buffer);
}



static void take_image() {

	shrink(buffer_raw, buffer, width, height, 3, 4);
	width /= 4;
	height /= 4;


	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;

	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);

	//FILE* outfile = fopen("test.jpeg", "wb");
	//jpeg_stdio_dest(&cinfo, outfile);

	// Using
	cinfo.dest = (struct jpeg_destination_mgr *) malloc(sizeof(struct jpeg_destination_mgr));
	cinfo.dest->init_destination = &my_init_destination;
	cinfo.dest->empty_output_buffer = &my_empty_output_buffer;
	cinfo.dest->term_destination = &my_term_destination;

	cinfo.image_width = width;
	cinfo.image_height = height;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_RGB;

	jpeg_set_defaults(&cinfo);
	/*set the quality [0..100]  */
	jpeg_set_quality(&cinfo, 75, true);
	jpeg_start_compress(&cinfo, true);

	JSAMPROW row_pointer;/* pointer to a single row */

	while(cinfo.next_scanline < cinfo.image_height) {
		row_pointer = (JSAMPROW) &buffer[cinfo.next_scanline*3*width];
		jpeg_write_scanlines(&cinfo, &row_pointer, 1);
	}

	jpeg_finish_compress(&cinfo);

	free(buffer_raw);
	free(buffer);
}


}
