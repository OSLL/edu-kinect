#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "libfreenect.h"
#include <vector>
#include <string>
#include <iostream>
#include <stdint.h>
#include <pthread.h>
#include <unistd.h>

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

	freenect_resolution cur_depth_resolution = FREENECT_RESOLUTION_MEDIUM;
	freenect_depth_format cur_depth_format = FREENECT_DEPTH_REGISTERED;

	freenect_resolution cur_video_resolution = FREENECT_RESOLUTION_MEDIUM;
	freenect_video_format cur_video_format = FREENECT_VIDEO_RGB;

	const int MAX_FILES = 50;
	const int MAX_DEPTH_RESOLUTION = 0;
	const int MAX_DEPTH_FORMAT = 0;
	const int MAX_VIDEO_RESOLUTION = 0;
	const int MAX_VIDEO_FORMAT = 0;

	static const freenect_resolution types_of_resolutions_depth[];
	static const freenect_depth_format types_of_formats_depth[];
	static const freenect_resolution types_of_resolutions_video[];
	static const freenect_video_format types_of_formats_video[];
};

const freenect_resolution Data::types_of_resolutions_depth[] = {FREENECT_RESOLUTION_MEDIUM};
const freenect_depth_format Data::types_of_formats_depth[] = {FREENECT_DEPTH_REGISTERED};
const freenect_resolution Data::types_of_resolutions_video[] = {FREENECT_RESOLUTION_MEDIUM};
const freenect_video_format Data::types_of_formats_video[] = {FREENECT_VIDEO_RGB};


//const freenect_video_format 

//Data::types_of_formats_depth

//const freenect_resolution Data::types_of_resolutions_depth[1] = {FREENECT_RESOLUTION_MEDIUM};
//const freenect_depth_format Data::types_of_formats_depth[1] = {FREENECT_DEPTH_REGISTERED};
//const freenect_resolution Data::types_of_resolutions_video[1] = {FREENECT_RESOLUTION_MEDIUM};
//const freenect_video_format Data::types_of_formats_video[1] = {FREENECT_VIDEO_RGB};

struct Singleton
{
	public:
		Data data;
		static Singleton* GetInstance();
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
	if ((int)depth_frame.size() < (int)Singleton::GetInstance()->data.count_of_files)
	{
		depth_frame.push_back(depth);
	}
}
void frame_sequence::add_video(uint8_t* video)
{
	if ((int)video_frame.size() < Singleton::GetInstance()->data.count_of_files)
	{
		video_frame.push_back(video);
	}
}
int frame_sequence::size()
{
	return std::min((int)depth_frame.size(), (int)video_frame.size());
}

void frame_sequence::write_to_files(Files& kinect_files)
{
	int width_depth = freenect_find_depth_mode(Singleton::GetInstance()->data.cur_depth_resolution, Singleton::GetInstance()->data.cur_depth_format).width;
	int height_depth = freenect_find_depth_mode(Singleton::GetInstance()->data.cur_depth_resolution, Singleton::GetInstance()->data.cur_depth_format).height;
	
	for (int i = 0; i < (int)kinect_files.depth_files.size(); i++)
	{
		for (int j = 0; j < height_depth; j++)
		{
			for (int l = 0; l < width_depth; l++)
				fprintf(kinect_files.depth_files[i], "%d ", depth_frame[i][j*width_depth + l]);
			fprintf(kinect_files.depth_files[i], "\n");
		}
	}

	int width_video = freenect_find_video_mode(Singleton::GetInstance()->data.cur_video_resolution, 
								Singleton::GetInstance()->data.cur_video_format).width;
	int height_video = freenect_find_video_mode(Singleton::GetInstance()->data.cur_video_resolution, 
								Singleton::GetInstance()->data.cur_video_format).height;
	for (int i = 0; i < (int)kinect_files.video_files.size(); i++)
	{
		for (int j = 0; j < height_video; j++)
		{
			for (int l = 0; l < width_video; l++)
			{	for (int t = 0; t < 3; t++)
				{
					fprintf(kinect_files.video_files[i], "%x", video_frame[i][(j*width_video + l)*3 + t]);
				}
				fprintf(kinect_files.video_files[i], " ");
			}
			fprintf(kinect_files.video_files[i], "\n");
		}
	}
}

Singleton* Singleton::GetInstance()
{
	return &self;
}

Singleton Singleton::self;

My_Error::My_Error(std::string _s): s(_s) {}

Device::Device()
{
	if (freenect_init(&(f_ctx), NULL) < 0) {
		throw My_Error("freenect_init() failed\n");
	}
	
	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(f_ctx, FREENECT_DEVICE_CAMERA); 

	nr_devices = freenect_num_devices (f_ctx);
	std::cout << "Number of devices found:" << nr_devices << "\n";
}

void Device::InitDevice(int user_device_number = 0)
{
	if (nr_devices < 1) {
		freenect_shutdown(f_ctx);
		throw My_Error("Devices did not find\n");
	}

	if (freenect_open_device(f_ctx, &(f_dev), user_device_number) < 0) {
		freenect_shutdown(f_ctx);
		throw My_Error("Could not open device\n");
	}

	uint16_t* depth = (uint16_t*) new uint8_t [freenect_find_depth_mode(Singleton::GetInstance()->data.cur_depth_resolution, Singleton::GetInstance()->data.cur_depth_format).bytes];
	uint8_t* rgb = new uint8_t [freenect_find_video_mode(Singleton::GetInstance()->data.cur_video_resolution, Singleton::GetInstance()->data.cur_video_format).bytes];


 	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_video_callback(f_dev, video_cb);
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(Singleton::GetInstance()->data.cur_depth_resolution, Singleton::GetInstance()->data.cur_depth_format));
	freenect_set_video_mode(f_dev, freenect_find_video_mode(Singleton::GetInstance()->data.cur_video_resolution, Singleton::GetInstance()->data.cur_video_format));

	Singleton::GetInstance()->data.depth = depth;
	Singleton::GetInstance()->data.rgb = rgb;

	freenect_set_depth_buffer(f_dev, Singleton::GetInstance()->data.depth);
	freenect_set_video_buffer(f_dev, Singleton::GetInstance()->data.rgb);

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
		usleep(10000);
		if (Singleton::GetInstance()->data.kinect_data.size() >= Singleton::GetInstance()->data.count_of_files)
		{
			freenect_stop_depth(f_dev);
			freenect_stop_video(f_dev);
			Singleton::GetInstance()->data.kinect_data.write_to_files(kinect_files);
			break;
		}	
		int res = freenect_process_events(f_ctx);
		if (res < 0 && res != -10) {
			throw My_Error("\nError received from libusb - aborting.");
			break;
		}
	}
}

void write_help()
{
	std::cout << "use <run_file> <number_of_device> <count_of_files> <resolution_depth> <format_depth> <resolution_format> <resolution_depth>" << std::endl;
	std::cout << "	number_of_device: number of device that you want to open, [0..], default = 0\n";
	std::cout << "	count_of_files: number of files that you want to get in the end, [1..50], default 1\n"; 
	std::cout << "	resolution_depth: code of depth resolution, [0.." << Singleton::GetInstance()->data.MAX_DEPTH_RESOLUTION << "]\n";
	std::cout << "		0 - (default) FREENECT_RESOLUTION_MEDIUM\n";
	std::cout << "	format_depth: code of depth format, [0.." << Singleton::GetInstance()->data.MAX_DEPTH_FORMAT << "]\n";
	std::cout << "		0 - (default) FREENECT_DEPTH_REGISTERED\n";
	std::cout << "	video_resolution: code of video resolution, [0.." << Singleton::GetInstance()->data.MAX_VIDEO_RESOLUTION << "]\n";
	std::cout << "		0 - (default) FREENECT_RESOLUTION_MEDIUM\n";
	std::cout << " video_format: code of video format, [0.." << Singleton::GetInstance()->data.MAX_VIDEO_FORMAT << "]\n";
	std::cout << "		0 - (default) FREENECT_VIDEO_RGB\n"; 	 
}

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
//	std::cerr << "depth\n"; 
	assert (Singleton::GetInstance()->data.depth == v_depth);
	
	uint16_t* new_depth = (uint16_t*) new uint8_t [freenect_find_depth_mode(Singleton::GetInstance()->data.cur_depth_resolution, Singleton::GetInstance()->data.cur_depth_format).bytes];

	Singleton::GetInstance()->data.kinect_data.add_depth((uint16_t*)Singleton::GetInstance()->data.depth);
	Singleton::GetInstance()->data.depth = new_depth;
	freenect_set_depth_buffer(dev, Singleton::GetInstance()->data.depth);
}

void video_cb(freenect_device *dev, void *v_rgb, uint32_t timestamp)
{
//	std::cerr << "video\n";
	assert(Singleton::GetInstance()->data.rgb == v_rgb);

	uint8_t* new_rgb = new uint8_t [freenect_find_video_mode(Singleton::GetInstance()->data.cur_video_resolution, Singleton::GetInstance()->data.cur_video_format).bytes];

	Singleton::GetInstance()->data.kinect_data.add_video((uint8_t*) Singleton::GetInstance()->data.rgb);
	Singleton::GetInstance()->data.rgb = new_rgb;
	freenect_set_video_buffer(dev, Singleton::GetInstance()->data.rgb);
}


std::string name_of_file_depth(int t, int max_t)
{
	int size = std::to_string(max_t).size();
	char s[size];
	std::sprintf(s, "%0*d", size, t);
	return (std::string("depth_file_") + std::string(s));		
}

std::string name_of_file_video(int t, int max_t)
{
	int size = std::to_string(max_t).size();
	char s[size];
	std::sprintf(s, "%0*d", size, t);
	return (std::string("video_file_") + std::string(s));		
}

void Files::init(int count_of_files)
{
	depth_files.resize(count_of_files);
	video_files.resize(count_of_files);

	for (int i = 0; i < count_of_files; i++)
	{
		depth_files[i] = fopen(name_of_file_depth(i, count_of_files - 1).c_str(), "w");
	}
	for (int j = 0; j < (int)video_files.size(); j++)
	{
		video_files[j] = fopen(name_of_file_video(j, count_of_files - 1).c_str(), "w");
	}
}

int CheckValidation(int argc, char** argv, int& user_device_number, int& count_of_files, Data& data)
{
	if (argc > 1 && strcmp(argv[1], "--help") == 0)
	{
		write_help();
		return 0;
	}
	count_of_files = 1;

	data.cur_depth_resolution = FREENECT_RESOLUTION_MEDIUM;
	data.cur_depth_format = FREENECT_DEPTH_REGISTERED;

	data.cur_video_resolution = FREENECT_RESOLUTION_MEDIUM;
	data.cur_video_format = FREENECT_VIDEO_RGB;

	if (argc > 1)
	{
		char *pEnd;
		user_device_number = strtol(argv[1], &pEnd, 10);
		if (*pEnd != 0)
		{
			std::cerr << "wrong arguments(count_of_files), argument is not number\n" << std::endl;
			write_help();
			throw My_Error("wrong_args");
		}
		if (user_device_number < 0)
		{
			std::cerr << "wrong arguments(count_of_files), number is out of bounds\n" << std::endl;
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
			std::cerr << "wrong arguments(count_of_files), argument is not number\n" << std::endl;
			write_help();
			throw My_Error("wrong_args");			
		}
		if (count_of_files < 1 || count_of_files > data.MAX_FILES)
		{
			std::cerr << "wrong arguments(count_of_files), number is out of bounds\n" << std::endl;
			write_help();
			throw My_Error("wrong_args");
		}
	}
	if (argc > 3)
	{
		char* pEnd;
		int num_resolution_depth;
		num_resolution_depth = strtol(argv[3], &pEnd, 10);
		if (*pEnd != 0)
		{
			std::cerr << "wrong arguments(resolution_depth), argument is not number\n" << std::endl;
			write_help();
			throw My_Error("wrong_args");			
		}
		if (num_resolution_depth < 0 || num_resolution_depth > data.MAX_DEPTH_RESOLUTION)
		{
			std::cerr << "wrong arguments(resolution_depth), number is out of bounds\n" << std::endl;
			write_help();
			throw My_Error("wrong_args");
		}
		data.cur_depth_resolution = Data::types_of_resolutions_depth[num_resolution_depth];
	}
	if (argc > 4)
	{
		char* pEnd;
		int num_format_depth;
		num_format_depth = strtol(argv[4], &pEnd, 10);
		if (*pEnd != 0)
		{
			std::cerr << "wrong arguments(format_depth), argument is not number\n" << std::endl;
			write_help();
			throw My_Error("wrong_args");			
		}
		if (num_format_depth < 0 || num_format_depth > data.MAX_DEPTH_FORMAT)
		{
			std::cerr << "wrong arguments(format_depth), number is out of bounds\n" << std::endl;
			write_help();
			throw My_Error("wrong_args");
		}	
		data.cur_depth_format = Data::types_of_formats_depth[num_format_depth];	
	}
	if (argc > 5)
	{
		char* pEnd;
		int num_video_resolution;
		num_video_resolution = strtol(argv[5], &pEnd, 10);
		if (*pEnd != 0)
		{
			std::cerr << "wrong arguments(video_resolution), argument is not number\n" << std::endl;
			write_help();
			throw My_Error("wrong_args");			
		}
		if (num_video_resolution < 0 || num_video_resolution > data.MAX_VIDEO_RESOLUTION)
		{
			std::cerr << "wrong arguments(video_resolution), number is out of bounds\n" << std::endl;
			write_help();
			throw My_Error("wrong_args");
		}
		data.cur_video_resolution = Data::types_of_resolutions_video[num_video_resolution];
	}
	if (argc > 6)
	{
		char* pEnd;
		int num_video_format;
		num_video_format = strtol(argv[6], &pEnd, 10);
		if (*pEnd != 0)
		{
			std::cerr << "wrong arguments(video_format), argument is not number" << std::endl;
			write_help();
			throw My_Error("wrong_args");			
		}
		if (num_video_format < 0 || num_video_format > data.MAX_VIDEO_FORMAT)
		{
			std::cerr << "wrong arguments(video_format), number is out of bounds " << std::endl;
			write_help();
			throw My_Error("wrong_args");
		}
		data.cur_video_format = Data::types_of_formats_video[num_video_format];		
	}
	return 1;
}

int main(int argc, char **argv)
{
	try{

		int user_device_number = 0;
	
		if (!CheckValidation(argc, argv, user_device_number, Singleton::GetInstance()->data.count_of_files, 
											Singleton::GetInstance()->data))
		{
			return 0;
		}
		Device dev;

		dev.InitDevice(user_device_number);	

		Files kinect_files;
		kinect_files.init(Singleton::GetInstance()->data.count_of_files);
	
		dev.startDepth();
		dev.startVideo();

		dev.writeFiles(kinect_files);
	}
	catch(My_Error& me)
	{
		std::cerr << me.s << std::endl;
	}
	catch(...)
	{
		std::cerr << "Error" << std::endl;	
	}	
	return 0;
}
