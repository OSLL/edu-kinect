#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "libfreenect.h"
#include <vector>
#include <string>
#include <iostream>
#include <stdint.h>

const freenect_resolution cur_resolution_depth = FREENECT_RESOLUTION_MEDIUM;
const freenect_depth_format cur_format_depth = FREENECT_DEPTH_REGISTERED;

const freenect_resolution cur_resolution_video = FREENECT_RESOLUTION_MEDIUM;
const freenect_video_format cur_format_video = FREENECT_VIDEO_RGB;

const int MAX_FILES = 50;

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp);
void video_cb(freenect_device *dev, void *v_rgb, uint32_t timestamp);

struct Files
{
	std::vector<FILE*> depth_files;
	std::vector<FILE*> video_files;
	void init(int t);
};

struct frame_sequence{
	void add_depth(uint16_t* depth);
	void add_video(uint8_t* video);
	int size();
	void write_to_files(Files& kinect_files);
	private:
		std::vector<uint16_t*> depth_frame;
		std::vector<uint8_t*> video_frame;
};

struct Data
{
	int count_of_files;
	frame_sequence kinect_data; 
	uint16_t* depth;
	uint8_t* rgb;
};

struct Singleton
{
	public:
		Data data;
		static Singleton* GetSingleton();
	private:        
		static Singleton self;
	        Singleton(){}
	        Singleton(const Singleton& root);
	        Singleton operator=(const Singleton&);
};

struct My_Error
{
	std::string s;
	My_Error(std::string _s);
};

struct Device
{
	freenect_context *f_ctx;
	freenect_device *f_dev;
	int nr_devices;
	Device();
	void InitDevice(int user_device_number);
	void startDepth();
	void startVideo();
	void writeFiles(Files& kinect_files);
};


void frame_sequence::add_depth(uint16_t* depth)
{
	if ((int)depth_frame.size() < (int)Singleton::GetSingleton()->data.count_of_files)
		depth_frame.push_back(depth);
}
void frame_sequence::add_video(uint8_t* video)
{
	if ((int)video_frame.size() < Singleton::GetSingleton()->data.count_of_files)
		video_frame.push_back(video);
}
int frame_sequence::size()
{
	return std::min((int)depth_frame.size(), (int)video_frame.size());
}

void frame_sequence::write_to_files(Files& kinect_files)
{
	int width_depth = freenect_find_depth_mode(cur_resolution_depth, cur_format_depth).width;
	int height_depth = freenect_find_depth_mode(cur_resolution_depth, cur_format_depth).height;
	for (int i = 0; i < (int)kinect_files.depth_files.size(); i++)
	{
		for (int j = 0; j < height_depth; j++)
		{
			for (int l = 0; l < width_depth; l++)
				fprintf(kinect_files.depth_files[i], "%d ", *depth_frame[j*width_depth + l]);
			fprintf(kinect_files.depth_files[i], "\n");
		}
	}
	int width_video = freenect_find_video_mode(cur_resolution_video, cur_format_video).width;
	int height_video = freenect_find_video_mode(cur_resolution_video, cur_format_video).height;
	for (int i = 0; i < (int)kinect_files.video_files.size(); i++)
	{
		for (int j = 0; j < height_video; j++)
		{
			for (int l = 0; l < width_video; l++)
				fprintf(kinect_files.video_files[i], "%x ", *video_frame[j*width_video + l]);
			fprintf(kinect_files.video_files[i], "\n");
		}
	}
}

Singleton* Singleton::GetSingleton()
{
	return &self;
}

Singleton Singleton::self;

My_Error::My_Error(std::string _s): s(_s) {}

Device::Device()
{
	if (freenect_init(&(f_ctx), NULL) < 0) {
		std::cerr << "freenect_init() failed\n";
		throw My_Error("init");
	}

	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_CAMERA)); 

	nr_devices = freenect_num_devices (f_ctx);
	std::cout << "Number of devices found:" << nr_devices << "\n";
}

void Device::InitDevice(int user_device_number = 0)
{
	if (nr_devices < 1) {
		freenect_shutdown(f_ctx);
		std::cerr << "Devices do not found\n";
		throw My_Error("not_found");
	}

	if (freenect_open_device(f_ctx, &(f_dev), user_device_number) < 0) {
		std::cerr << "Could not open device\n";
		freenect_shutdown(f_ctx);
		throw My_Error("can_not_open");
	}

	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_video_callback(f_dev, video_cb);
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(cur_resolution_depth, cur_format_depth));
	freenect_set_video_mode(f_dev, freenect_find_video_mode(cur_resolution_video, cur_format_video));

	uint16_t* depth = (uint16_t*) new uint8_t [freenect_find_depth_mode(cur_resolution_depth, cur_format_depth).bytes];
	uint8_t* rgb = new uint8_t [freenect_find_video_mode(cur_resolution_video, cur_format_video).bytes];

	freenect_set_depth_buffer(f_dev, depth);
	freenect_set_video_buffer(f_dev, rgb);

}
void Device::startDepth()
{
	freenect_start_depth(f_dev);
}
void Device::startVideo()
{
	freenect_start_video(f_dev);
}

void Device::writeFiles(Files& kinect_files)
{
	while (1)
	{
		if (Singleton::GetSingleton()->data.kinect_data.size() >= Singleton::GetSingleton()->data.count_of_files)
		{
			freenect_stop_depth(f_dev);
			freenect_stop_video(f_dev);
			Singleton::GetSingleton()->data.kinect_data.write_to_files(kinect_files);
			break;
		}
	}
}

void write_help()
{
	std::cout << "use <run_file> <number_of_device> <count_of_files>" << std::endl;
	std::cout << "	number_of_device: number of device that you want to open, default = 0\n";
	std::cout << "	count_of_files: number of files that you want to get in the end, 1 <= count_of_files <= 50, default 1\n"; 
}

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	assert (Singleton::GetSingleton()->data.depth == v_depth);
	
	uint16_t* new_depth = (uint16_t*) new uint8_t [freenect_find_depth_mode(cur_resolution_depth, cur_format_depth).bytes];

	Singleton::GetSingleton()->data.kinect_data.add_depth((uint16_t*)Singleton::GetSingleton()->data.depth);
	Singleton::GetSingleton()->data.depth = new_depth;
	freenect_set_depth_buffer(dev, Singleton::GetSingleton()->data.depth);
}

void video_cb(freenect_device *dev, void *v_rgb, uint32_t timestamp)
{
	assert(Singleton::GetSingleton()->data.rgb == v_rgb);

	uint8_t* new_rgb = new uint8_t [freenect_find_video_mode(cur_resolution_video, cur_format_video).bytes];

	Singleton::GetSingleton()->data.kinect_data.add_video((uint8_t*) Singleton::GetSingleton()->data.rgb);
	Singleton::GetSingleton()->data.rgb = new_rgb;
	freenect_set_video_buffer(dev, Singleton::GetSingleton()->data.rgb);
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

void Files::init(int count_of_files)
{
	depth_files.resize(count_of_files);
	video_files.resize(count_of_files);

	for (int i = 0; i < count_of_files; i++)
	{
		depth_files[i] = fopen(name_of_file_depth(i).c_str(), "w");
	}
	for (int j = 0; j < (int)video_files.size(); j++)
	{
		video_files[j] = fopen(name_of_file_video(j).c_str(), "w");
	}
}

void CheckValidation(int argc, char** argv, int& user_device_number, int& count_of_files)
{
	count_of_files = 1;
	if (argc > 1)
	{
		char *pEnd;
		user_device_number = strtol(argv[1], &pEnd, 10);
		if (*pEnd != 0)
		{
			std::cerr << "wrong arguments(user_device_number)" << std::endl;
			write_help();
			throw My_Error("wrong_args");
		}
	}
	if (argc > 2)
	{
		char *pEnd;
		count_of_files = strtol(argv[2], &pEnd, 10);
		if (*pEnd != 0)
		{
			std::cerr << "wrong arguments(count_of_files), argument is not number" << std::endl;
			write_help();
			throw My_Error("wrong_args");			
		}
		if (count_of_files < 1 || count_of_files > MAX_FILES)
		{
			std::cerr << "wrong arguments(count_of_files), number is out of bounds " << std::endl;
			write_help();
			throw My_Error("wrong_args");
		}
	}
}

int main(int argc, char **argv)
{
	int user_device_number = 0;

	CheckValidation(argc, argv, user_device_number, Singleton::GetSingleton()->data.count_of_files);	

	Device dev;

	dev.InitDevice(user_device_number);	

	Files kinect_files;
	kinect_files.init(Singleton::GetSingleton()->data.count_of_files);

	dev.startDepth();
	dev.startVideo();

	dev.writeFiles(kinect_files);
	
	return 0;
}
