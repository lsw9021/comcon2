#include "GLUTWindow3D.h"
#include "Camera.h"
#include <GL/glut.h>

GLUTWindow3D::
GLUTWindow3D()
	:GLUTWindow(),mCamera(new Camera()),mDrag(false),mMouse(0),mPrevX(0),mPrevY(0)
{

}
void
GLUTWindow3D::
display()
{
	glClearColor(1.0,1.0,1.0,1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	this->initLights();
	mCamera->apply();
	this->render();
	glutSwapBuffers();
}
void
GLUTWindow3D::
keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
		case 27: exit(0); break;
		default: break;
	}
	glutPostRedisplay();
}
void
GLUTWindow3D::
special(int key, int x, int y)
{
	glutPostRedisplay();
}
#include <iostream>
void
GLUTWindow3D::
mouse(int button, int state, int x, int y)
{
	if(state == GLUT_DOWN){
		mDrag = true;
		mPrevX = x;
		mPrevY = y;
	}
	else
		mDrag = false;
	mMouse = button;
	switch(mMouse)
	{
		case 3: mCamera->pan(0,0,0,10);break;//Up
		case 4: mCamera->pan(0,0,0,-10);break;//Down
		default: break;	
	}
	glutPostRedisplay();
}
void
GLUTWindow3D::
motion(int x, int y)
{
	if(mDrag == false)
		return;
	switch(mMouse)
	{
		case 0: mCamera->rotate(x,y,mPrevX,mPrevY);break;//Left
		case 1: mCamera->translate(x,y,mPrevX,mPrevY);break;//Mid
		default: break;
	}
	mPrevX = x;
	mPrevY = y;
	glutPostRedisplay();
}
void
GLUTWindow3D::
reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	mCamera->apply();
}
void
GLUTWindow3D::
timer(int tic)
{
	glutTimerFunc(30,timerEvent,1);
	glutPostRedisplay();
}
void
GLUTWindow3D::
initLights()
{
	static float ambient[]             = {0.2, 0.2, 0.2, 1.0};
	static float diffuse[]             = {0.2, 0.2, 0.2, 1.0};
	static float front_mat_shininess[] = {60.0};
	static float front_mat_specular[]  = {0.2, 0.2,  0.2,  1.0};
	static float front_mat_diffuse[]   = {0.2, 0.2, 0.2, 1.0};
	static float lmodel_ambient[]      = {0.2, 0.2,  0.2,  1.0};
	static float lmodel_twoside[]      = {GL_TRUE};

	GLfloat position[] = {0.0, 100.0, 100.0, 0.0};
	GLfloat position1[] = {0.0, 100.0, -100.0, 0.0};

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,  lmodel_ambient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position1);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);

	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, front_mat_shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  front_mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   front_mat_diffuse);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDisable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);
	glEnable(GL_FOG);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	GLfloat fogColor[] = {1,1,1,1};
	glFogfv(GL_FOG_COLOR,fogColor);
	glFogi(GL_FOG_MODE,GL_LINEAR);
	glFogf(GL_FOG_START, 10.0);
	glFogf(GL_FOG_END,20.0);
}