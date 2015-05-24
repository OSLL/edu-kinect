#include <GL/glut.h>
#include <GL/glut.h>
#include<cstdio>
#include<iostream>

class Drawer
{	
	static Drawer* currentInstance;

	int* array_video;
	uint16_t* array_depth;
	int height;
	int width;
	static void rerouting_draw(); 

	public:	

		Drawer(int* _array_video, uint16_t* _array_depth, int h, int w);
		void init(int &argc, char* argv[]);
		void display();
		void draw();
};
Drawer* Drawer::currentInstance;

Drawer::Drawer(int* _array_video, uint16_t* _array_depth, int h, int w)
{	
	array_video = _array_video;
	array_depth = _array_depth;	
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
			double color1 = (double)(array_video[i*width + j] >> 16)/255.0;
			double color2 = (double)((array_video[i*width + j] >> 8) % 256)/255.0;
			double color3 = (double)(array_video[i*width + j] % 256)/255.0;
//			if (i == 100) printf("%x", array[i*width + j];
			glColor3f(color1, color2, color3);
//			if (i == 100) std::cerr << color << " ";
			glVertex2f((double)j/width, (double)i/height);
		}
	}
	for (int i = -height + 1; i < 0; i++)
	{
		for (int j = -width + 1; j < 0; j++)
		{
			double color = (double)(array_depth[(-i)*width + j]/1e4);
			glColor3f(color, color, color);
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
	FILE* f_video = fopen("video_file_01", "r");
	FILE* f_depth = fopen("depth_file_01", "r");

	int* arr_video = new int [480*640];
	uint16_t* arr_depth = new uint16_t [480*640];

	for (int i = 0; i < 480; i++)
	{
		for (int j = 0; j < 640; j++)
		{
			int k;
			fscanf(f_video, "%x", &arr_video[i*640 + j]);
			fscanf(f_depth, "%d", &k);
			arr_depth[i*640 + j] = k;
		}
	}
	Drawer drawer(arr_video, arr_depth, 480, 640);
	drawer.init(argc, argv);
	drawer.draw();

	return 0;
}
