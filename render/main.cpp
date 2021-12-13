#include <iostream>
#include "Window.h"

#include <GL/glew.h>
#include <GL/glut.h>

int main(int argc,char** argv)
{
	Window* window;
	window = new Window();
	if(argc==2)
		window->initNN(argv[1]);
	else if(argc == 3){
		window->initNN(argv[1]);
		window->loadNN(argv[2]);
	}
		
	glutInit(&argc,argv);
	
	window->initWindow(1920,1080,"kinematics");
	glutMainLoop();
	return 0;
}