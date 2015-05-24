#include<cstdio>
#include "libfreenect.h"
#include<algorithm>
#include<iostream>
#include<string>
#include<cstring>
#include<unistd.h>
#include<GL/glut.h>
#include<vector>
#include<dirent.h>

freenect_resolution CUR_DEPTH_RESOLUTION = FREENECT_RESOLUTION_MEDIUM;
freenect_depth_format CUR_DEPTH_FORMAT = FREENECT_DEPTH_REGISTERED;

struct My_Error;
struct video_frame;
class depth_frame;
struct work_files;
struct Manager;

struct My_Error
{
	std::string mes;
	My_Error(std::string _mes);
};

struct video_frame
{
	int count_of_bytes;
	char* frame;
		
	video_frame(freenect_video_format cur_freenect_video_format, freenect_resolution cur_video_resolution);  	
};

class depth_frame
{
	friend Manager;
	int count_of_bytes;
	freenect_frame_mode cur_depth_mode;
	uint16_t* frame;

	void read_frame(FILE* f);

	public:
		depth_frame(freenect_depth_format cur_freenect_depth_format, freenect_resolution cur_depth_resolution);

		~depth_frame();
}; 

struct work_files
{
	std::vector<std::string> depth;
	std::vector<std::string> video;
	DIR* dir;

	void find_depth_files(const char path_dir[] = std::string("./").c_str());
};

struct Manager
{
	private:
		freenect_depth_format cur_freenect_depth_format;
		freenect_resolution cur_depth_resolution;
		freenect_frame_mode cur_depth_mode;

		std::vector<std::vector<std::vector<uint16_t>>> depth_film;
		std::vector<std::vector<std::vector<uint16_t>>> video_film;
		int cnt_files = 0;
		int cut_files = 0;

		void add_depth_files(std::vector<std::string>& v);

		uint16_t* count_depth_sum();
	public:
		Manager(freenect_depth_format _cur_freenect_depth_format, freenect_resolution _cur_depth_resolution);

		void set_cut_files(int n);
		
		uint16_t* count_average();

		uint16_t* count_dispersion(uint16_t* average);

};

class Drawer
{	
	static Drawer* currentInstance;

	uint16_t* array;
	int height;
	int width;
	static void rerouting_draw(); 

	public:	

		Drawer(uint16_t* _array, int h, int w);
		void init(int &argc, char* argv[]);
		void display();
		void draw();
};
Drawer* Drawer::currentInstance;


My_Error::My_Error(std::string _mes): mes(_mes) {} 


video_frame::video_frame(freenect_video_format cur_freenect_video_format, freenect_resolution cur_video_resolution) {}  


void depth_frame::read_frame(FILE* f)
{	
	for (int i = 0; i < cur_depth_mode.height; i++)
	{
		for (int j = 0; j < cur_depth_mode.width; j++)
		{
			fscanf(f, "%hd", &frame[j + cur_depth_mode.width*i]);
		}
	}
}

depth_frame::depth_frame(freenect_depth_format cur_freenect_depth_format, freenect_resolution cur_depth_resolution)
{
	cur_depth_mode = freenect_find_depth_mode(cur_depth_resolution, cur_freenect_depth_format);
	count_of_bytes = cur_depth_mode.bytes;
	frame = (uint16_t*) new char [count_of_bytes];
	memset(frame, 0, sizeof(frame));
	
}

depth_frame::~depth_frame()
{
	delete frame;
} 


void work_files::find_depth_files(const char path_dir[])
{
	dirent* content;
	if ((dir = opendir(path_dir)) == NULL)
	{
		throw My_Error("can not open directory for find files");		
	}
	while ((content = readdir(dir)) != NULL)
	{
		if (std::strncmp("depth_file_", content->d_name, 11) == 0)
			depth.push_back(std::string(content->d_name));
	}	
	if (closedir(dir) != 0)
		throw My_Error("can not close dir");
}


void Manager::add_depth_files(std::vector<std::string>& v)
{
	FILE* f;
	depth_frame cur_frame(cur_freenect_depth_format, cur_depth_resolution); 
	for (int i = 0; i < (int)v.size(); i++)
	{
		f = fopen(v[i].c_str(), "r");
		if (f == NULL)
		{
			throw My_Error(std::string("can not open file ") + v[i]); 
		}
		cur_frame.read_frame(f);
		for (int j = 0; j < cur_depth_mode.height; j++)
			for (int k = 0; k < cur_depth_mode.width; k++)
			{
				depth_film[j][k][i] = cur_frame.frame[j*cur_depth_mode.width + k];
			}
		fclose(f);
	}
	for (auto i = depth_film.begin(); i != depth_film.end(); i++)
		for (auto j = i->begin(); j != i->end(); j++)
			std::sort(j->begin(), j->end());
}

uint16_t* Manager::count_depth_sum()
{		
	uint16_t* ans_frame = (uint16_t*) new char[cur_depth_mode.bytes];
	for (int i = cut_files; i < cnt_files - cut_files; i++)
	{
		for (int j = 0; j < cur_depth_mode.height; j++)
		{
			for (int k = 0; k < cur_depth_mode.width; k++)
			{
				ans_frame[j*cur_depth_mode.width+k] += depth_film[j][k][i]; 
			}
		}
	}
	return ans_frame;
}

Manager::Manager(freenect_depth_format _cur_freenect_depth_format, freenect_resolution _cur_depth_resolution)
{
	cur_freenect_depth_format = _cur_freenect_depth_format;
	cur_depth_resolution = _cur_depth_resolution;
	cur_depth_mode = freenect_find_depth_mode(cur_depth_resolution, cur_freenect_depth_format);
	work_files wf;
	wf.find_depth_files(std::string("./").c_str());
	depth_film.resize(cur_depth_mode.height);
	for (auto i = depth_film.begin(); i != depth_film.end(); i++)
	{
		i->resize(cur_depth_mode.width);
		for (auto j = i->begin(); j != i->end(); j++)
			j->resize((int)wf.depth.size());
	}
	cnt_files = (int)wf.depth.size();
	add_depth_files(wf.depth);
}

void Manager::set_cut_files(int n)
{
	if (2*n >= cnt_files)
	{
		throw My_Error("You want too much values to cut. Method count_average(int cut)");
	}
	cut_files = n;
}

uint16_t* Manager::count_average()
{
	if (cnt_files == 0)
	{
		throw My_Error("Not find files. Files must have the following form: depth_file_*");
	}
	
	uint16_t* sum_frame = count_depth_sum();

	for (int i = 0; i < cur_depth_mode.height; i++)
	{
		for (int j = 0; j < cur_depth_mode.width; j++)
		{
			sum_frame[i*cur_depth_mode.width+j] /= cnt_files - 2*(cut_files); 							
		}
	}
	return sum_frame;
} 

uint16_t* Manager::count_dispersion(uint16_t* average)
{
	if ((int)depth_film.size() == 0)
	{
		throw My_Error("Not find files. Files must have the following form: depth_file_*");
	}

	uint16_t* ans = new uint16_t [cur_depth_mode.height*cur_depth_mode.width];
	for (int i = 0; i < cur_depth_mode.height; i++)
	{
		for (int j = 0; j < cur_depth_mode.width; j++)	
		{
			double temp_ans = 0;
			for (int k = cut_files; k < cnt_files - cut_files; k++)
			{

				temp_ans += pow((depth_film[i][j][k] - 
						average[i*cur_depth_mode.width + j]), 2)/(cnt_files - 2*cut_files);	
			}
			ans[i*cur_depth_mode.width + j] = (uint16_t)sqrt(temp_ans);
		}
	}
	return ans;
}


Drawer::Drawer(uint16_t* _array, int h, int w)
{	
	array = _array;	
	height = h;
	width = w;
}

void Drawer::display()
{
	glClear(GL_COLOR_BUFFER_BIT);
	glPointSize(1.0);
	glBegin(GL_POINTS);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			double color = (double)array[i*width + j]/FREENECT_DEPTH_MM_MAX_VALUE;
			glColor3f(color, color, color);
//			if (i == 100) std::cerr << color << " ";
			glVertex2f((double)j/width, (double)i/height);
		}
	}
	glEnd();
	glFlush();

}

void Drawer::init(int& argc, char* argv[])
{
	Drawer::currentInstance = this;	

	glutInitWindowSize(640, 480); 
	glutInitWindowPosition(0, 0);  

	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);

	glutCreateWindow("My OpenGL Application");
	::glutDisplayFunc(Drawer::rerouting_draw);	
	
	glClearColor(0, 0, 0, 0);

}

void Drawer::rerouting_draw()
{
	currentInstance->display();
} 

void Drawer::draw()
{
	glutMainLoop();  

}

int main(int argc, char* argv[])
{
	try{
		Manager manager(CUR_DEPTH_FORMAT, CUR_DEPTH_RESOLUTION);
		manager.set_cut_files(1);
		uint16_t* aver = manager.count_average();
		uint16_t* disp = manager.count_dispersion(aver);
		Drawer drawer(aver, 480, 640);
		drawer.init(argc, argv);
		drawer.draw();
	}
	catch(My_Error& me)
	{
		std::cerr << me.mes << std::endl;
	}	

	return 0;
}

