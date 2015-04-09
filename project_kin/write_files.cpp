#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "libfreenect.h"
#include <vector>
#include <string>
#include <iostream>
#include <stdint.h>

freenect_resolution cur_resolution_depth = FREENECT_RESOLUTION_MEDIUM;
freenect_depth_format cur_format_depth = FREENECT_DEPTH_REGISTERED;

freenect_resolution cur_resolution_video = FREENECT_RESOLUTION_MEDIUM;
freenect_video_format cur_format_video = FREENECT_VIDEO_RGB;


struct Data
{
	freenect_context *f_ctx;
	freenect_device *f_dev;

	uint16_t* depth;
	uint8_t* rgb;
	
	std::vector<FILE*> files_depth;
	std::vector<FILE*> files_video;		
};

struct Singleton
{
	public:
		Data data;
		static Singleton* GiveSingleton()
		{
			if (!self)
			{
				self = new Singleton();
			}			
			return self;
		}
	private:        
		static Singleton* self;
	        Singleton(){}
	        Singleton(const Singleton& root);
	        Singleton operator=(const Singleton&);
};

Singleton* Singleton::self = NULL;

struct frame_sequence{
	void add_depth(uint16_t* depth)
	{
		if ((int)depth_frame.size() < Singleton::GiveSingleton()->data.files_video.size())
			depth_frame.push_back(depth);
	}
	void add_video(uint8_t* video)
	{
		if ((int)video_frame.size() < Singleton::GiveSingleton()->data.files_video.size())
			video_frame.push_back(video);
	}
	int size()
	{
		return std::min((int)depth_frame.size(), (int)video_frame.size());
	}
	void write_to_files(std::vector<FILE*>& files_depth, std::vector<FILE*>& files_video)
	{
		int width_depth = freenect_find_depth_mode(cur_resolution_depth, cur_format_depth).width;
		int height_depth = freenect_find_depth_mode(cur_resolution_depth, cur_format_depth).height;
		for (int i = 0; i < (int)files_depth.size(); i++)
		{
			for (int j = 0; j < height_depth; j++)
			{
				for (int l = 0; l < width_depth; l++)
					fprintf(files_depth[i], "%d ", *depth_frame[j*width_depth + l]);
				fprintf(files_depth[i], "\n");
			}
		}
		int width_video = freenect_find_video_mode(cur_resolution_video, cur_format_video).width;
		int height_video = freenect_find_video_mode(cur_resolution_video, cur_format_video).height;
		for (int i = 0; i < (int)files_video.size(); i++)
		{
			for (int j = 0; j < height_video; j++)
			{
				for (int l = 0; l < width_video; l++)
					fprintf(files_video[i], "%x ", *video_frame[j*width_video + l]);
				fprintf(files_video[i], "\n");
			}
		}
	}
	private:
		std::vector<uint16_t*> depth_frame;
		std::vector<uint8_t*> video_frame;
};

frame_sequence kinect_data; 

void write_help()
{
	std::cout << "use <run_file> <number_of_device>(default 0) <count_of_files(default 1)>" << std::endl;
}

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	assert (Singleton::GiveSingleton()->data.depth == v_depth);
	
	uint16_t* new_depth = (uint16_t*)malloc(freenect_find_depth_mode(cur_resolution_depth, cur_format_depth).bytes);

	kinect_data.add_depth((uint16_t*)Singleton::GiveSingleton()->data.depth);
	Singleton::GiveSingleton()->data.depth = new_depth;
	freenect_set_depth_buffer(dev, Singleton::GiveSingleton()->data.depth);
}

void video_cb(freenect_device *dev, void *v_rgb, uint32_t timestamp)
{
	assert(Singleton::GiveSingleton()->data.rgb == v_rgb);

	uint8_t* new_rgb = (uint8_t*)malloc(freenect_find_video_mode(cur_resolution_video, cur_format_video).bytes);

	kinect_data.add_video((uint8_t*) Singleton::GiveSingleton()->data.rgb);
	Singleton::GiveSingleton()->data.rgb = new_rgb;
	freenect_set_video_buffer(dev, Singleton::GiveSingleton()->data.rgb);
}


bool init_device(int argc, char** argv)
{
	if (freenect_init(&(Singleton::GiveSingleton()->data.f_ctx), NULL) < 0) {
		printf("freenect_init() failed\n");
		return false;
	}

	freenect_set_log_level(Singleton::GiveSingleton()->data.f_ctx, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(Singleton::GiveSingleton()->data.f_ctx, (freenect_device_flags)(FREENECT_DEVICE_CAMERA)); 

	int nr_devices = freenect_num_devices (Singleton::GiveSingleton()->data.f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);

	int user_device_number = 0;
	if (argc > 1)
		user_device_number = atoi(argv[1]);
	if (nr_devices < 1) {
		freenect_shutdown(Singleton::GiveSingleton()->data.f_ctx);
		printf("Devices do not found\n");
		return false;
	}

	if (freenect_open_device(Singleton::GiveSingleton()->data.f_ctx, &(Singleton::GiveSingleton()->data.f_dev), user_device_number) < 0) {
		printf("Could not open device\n");
		freenect_shutdown(Singleton::GiveSingleton()->data.f_ctx);
		return false;
	}

	freenect_set_depth_callback(Singleton::GiveSingleton()->data.f_dev, depth_cb);
	freenect_set_video_callback(Singleton::GiveSingleton()->data.f_dev, video_cb);
	freenect_set_depth_mode(Singleton::GiveSingleton()->data.f_dev, freenect_find_depth_mode(cur_resolution_depth, cur_format_depth));
	freenect_set_video_mode(Singleton::GiveSingleton()->data.f_dev, freenect_find_video_mode(cur_resolution_video, cur_format_video));

	uint16_t* depth = (uint16_t*)malloc(freenect_find_depth_mode(cur_resolution_depth, cur_format_depth).bytes);
	uint8_t* rgb = (uint8_t*)malloc(freenect_find_video_mode(cur_resolution_video, cur_format_video).bytes);

	freenect_set_depth_buffer(Singleton::GiveSingleton()->data.f_dev, depth);
	freenect_set_video_buffer(Singleton::GiveSingleton()->data.f_dev, rgb);

	return true;
}


std::string name_of_file_depth(int t)
{
	if (t < 10)
	{
		return ("depth_file_0" + std::to_string(t));
	}
	else
	{
		return ("depth_file_" + std::to_string(t));
	}
}

std::string name_of_file_video(int t)
{
	if (t < 10)
	{
		return ("video_file_0" + std::to_string(t));
	}
	else
	{
		return ("video_file_" + std::to_string(t));
	}
}


bool init_files(int argc, char** argv, std::vector<FILE*>& files_depth, std::vector<FILE*>& files_video)
{
	if (argc < 2)
	{
		files_depth.resize(1);
		files_video.resize(1);
	}
	else
	{	
		if ((int)atoi(argv[2]) > 50)
		{
			printf("Too much files set\n");
			freenect_shutdown(Singleton::GiveSingleton()->data.f_ctx);
			write_help();
			return false;
		}
		files_depth.resize((int)atoi(argv[2]));
		files_video.resize((int)atoi(argv[2]));		
	}
	for (int i = 0; i < (int)files_depth.size(); i++)
	{
		files_depth[i] = fopen(name_of_file_depth(i).c_str(), "w");
	}
	for (int j = 0; j < (int)files_video.size(); j++)
	{
		files_video[j] = fopen(name_of_file_video(j).c_str(), "w");
	}
}

int main(int argc, char **argv)
{

	if (!init_device(argc, argv))
		return 1;
	
	if (!init_files(argc, argv, Singleton::GiveSingleton()->data.files_depth, Singleton::GiveSingleton()->data.files_video))
		return 1;

	freenect_start_depth(Singleton::GiveSingleton()->data.f_dev);
	freenect_start_video(Singleton::GiveSingleton()->data.f_dev);

	while (1)
	{
		if (kinect_data.size() == (int)Singleton::GiveSingleton()->data.files_depth.size())
		{
			freenect_stop_depth(Singleton::GiveSingleton()->data.f_dev);
			freenect_stop_video(Singleton::GiveSingleton()->data.f_dev);
			kinect_data.write_to_files(Singleton::GiveSingleton()->data.files_depth, Singleton::GiveSingleton()->data.files_video);
			break;
		}
	}

	return 0;
}
