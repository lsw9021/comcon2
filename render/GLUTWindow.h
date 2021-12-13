#ifndef __GLUT_WINDOW_H__
#define __GLUT_WINDOW_H__
#include <string>
#include <vector>
class Camera;
class GLUTWindow
{
public:
	GLUTWindow();

	void initWindow(int w,int h,const std::string& name);

	static GLUTWindow* current();
	static void displayEvent();
	static void keyboardEvent(unsigned char key, int x, int y);
	static void specialEvent(int key,int x,int y);
	static void mouseEvent(int button, int state, int x, int y);
	static void motionEvent(int x,int y);
	static void reshapeEvent(int w,int h);
	static void timerEvent(int tic);
	static std::vector<GLUTWindow*> windows;
	static std::vector<int> winIDs;
protected:
	virtual void display() = 0;
	virtual void keyboard(unsigned char key, int x, int y) = 0;
	virtual void special(int key, int x, int y) = 0;
	virtual void mouse(int button, int state, int x, int y) = 0;
	virtual void motion(int x, int y) = 0;
	virtual void reshape(int w, int h) = 0;
	virtual void timer(int tic) = 0;
};
#endif