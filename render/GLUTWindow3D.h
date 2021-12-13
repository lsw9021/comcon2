#ifndef __GLUT_WINDOW_3D_H__
#define __GLUT_WINDOW_3D_H__
#include "GLUTWindow.h"

class GLUTWindow3D : public GLUTWindow
{
public:
	GLUTWindow3D();

	virtual void initLights();
	virtual void render() = 0;
protected:
	virtual void display();
	virtual void keyboard(unsigned char key, int x, int y);
	virtual void special(int key, int x, int y);
	virtual void mouse(int button, int state, int x, int y);
	virtual void motion(int x, int y);
	virtual void reshape(int w, int h);
	virtual void timer(int tic);
protected:
	Camera* mCamera;

	bool mDrag;
	int mMouse;
	int mPrevX,mPrevY;
};
#endif