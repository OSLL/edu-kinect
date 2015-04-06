#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "libfreenect.h"
#include <vector>
#include <stdint.h>

freenect_resolution cur_resolution_depth = FREENECT_RESOLUTION_MEDIUM;
freenect_depth_format cur_format_depth = FREENECT_DEPTH_REGISTERED;

freenect_resolution cur_resolution_video = FREENECT_RESOLUTION_MEDIUM;
freenect_video_format cur_format_video = FREENECT_VIDEO_RGB;


freenect_context *f_ctx;
freenect_device *f_dev;

uint16_t* depth;
uint8_t* rgb;

struct frame_sequence{
	std::vector<uint16_t*> depth_frame;
	std::vector<uint8_t*> video_frame;	
};

frame_sequence kinect_data; 

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	assert (depth == v_depth);
	
	uint16_t* new_depth = (uint16_t*)malloc(freenect_find_depth_mode(cur_resolution_depth, cur_format_depth).bytes);

	kinect_data.depth_frame.push_back((uint16_t*)depth);
	depth = new_depth;
	freenect_set_depth_buffer(dev, depth);
}

void video_cb(freenect_device *dev, void *v_rgb, uint32_t timestamp)
{
	assert(rgb == v_rgb);

	uint8_t* new_rgb = (uint8_t*)malloc(freenect_find_video_mode(cur_resolution_video, cur_format_video).bytes);

	kinect_data.video_frame.push_back((uint8_t*) rgb);
	rgb = new_rgb;
	freenect_set_video_buffer(dev, rgb);
}

int main(int argc, char **argv)
{
	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}

	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_CAMERA)); 

	int nr_devices = freenect_num_devices (f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);

	int user_device_number = 0;
	if (argc > 1)
		user_device_number = atoi(argv[1]);

	if (nr_devices < 1) {
		freenect_shutdown(f_ctx);
		return 1;
	}

	if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
		freenect_shutdown(f_ctx);
		return 1;
	}

	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_video_callback(f_dev, video_cb);
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(cur_resolution_depth, cur_format_depth));
	freenect_set_video_mode(f_dev, freenect_find_video_mode(cur_resolution_video, cur_format_video));

	uint16_t* depth = (uint16_t*)malloc(freenect_find_depth_mode(cur_resolution_depth, cur_format_depth).bytes);
	uint8_t* rgb = (uint8_t*)malloc(freenect_find_video_mode(cur_resolution_video, cur_format_video).bytes);

	freenect_set_depth_buffer(f_dev, depth);
	freenect_set_video_buffer(f_dev, rgb);

	freenect_start_depth(f_dev);
	freenect_start_video(f_dev);

	return 0;
}
