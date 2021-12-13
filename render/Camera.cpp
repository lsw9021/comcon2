#include "Camera.h"
#include <GL/glut.h>
Camera::
Camera(const Eigen::Vector3d& look_at,const Eigen::Vector3d& eye,const Eigen::Vector3d& up,float fovy)
	:mLookAt(look_at),mEye(eye),mUp(up),mFovy(fovy),mZNear(0.1), mZFar(10000.0)
{
}
void
Camera::
apply()
{
	GLint w = glutGet(GLUT_WINDOW_WIDTH);
	GLint h = glutGet(GLUT_WINDOW_HEIGHT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(mFovy,(GLfloat)w/(GLfloat)h, mZNear, mZFar);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(mEye[0],mEye[1],mEye[2],mLookAt[0],mLookAt[1],mLookAt[2],mUp[0],mUp[1],mUp[2]);
}
void
Camera::
pan(int x,int y,int px,int py)
{
	float delta = (float)(py - y);
	delta = 1.0f - delta/200.0f;
	mEye = mLookAt - (mLookAt - mEye)*delta;
}
void
Camera::
zoom(int x,int y,int px,int py)
{
	float delta = (float)(py - y);
	mFovy += delta/20.0;
}
void
Camera::
rotate(int x,int y,int px,int py)
{
	Eigen::Vector3d prev_point = this->getTrackballPoint(px,py);
	Eigen::Vector3d curr_point = this->getTrackballPoint(x,y);
	Eigen::Vector3d axis = (curr_point.cross(prev_point)).normalized();

	axis = this->getCameraCoordinate()*axis;

	float cosT = curr_point.dot(prev_point)/(curr_point.norm()*prev_point.norm());
	float sinT = curr_point.cross(prev_point).norm()/(curr_point.norm()*prev_point.norm());

	double angle = std::atan2(sinT,cosT);

	Eigen::Vector3d n = mLookAt - mEye;
	Eigen::AngleAxisd aa(angle,axis);

	n = aa*n;
	mUp = aa*mUp;
	mEye = mLookAt - n;
}
void
Camera::
translate(int x,int y,int px,int py)
{
	Eigen::Vector3d delta((double)(x - px),(double)(y - py),0.0);
	delta = this->getCameraCoordinate()*(delta/200.0);
	mLookAt += delta;
	mEye += delta;
}
Eigen::Matrix3d
Camera::
getCameraCoordinate()
{
	Eigen::Vector3d n = (mLookAt - mEye).normalized();
	Eigen::Vector3d u = (mUp.cross(n)).normalized();
	Eigen::Vector3d v = (n.cross(u)).normalized();

	Eigen::Matrix3d uvn;
	uvn.col(0) = u;
	uvn.col(1) = v;
	uvn.col(2) = n;

	return uvn;
}
Eigen::Matrix4d
Camera::
getProjectionMatrix()
{
	GLint w = glutGet(GLUT_WINDOW_WIDTH);
	GLint h = glutGet(GLUT_WINDOW_HEIGHT);

	double aspect = (double)w/(double)h;

	double f = std::tan(M_PI*0.5*(1.0 - mFovy/180.0));
	double range_inv = 1.0 / (mZNear - mZFar);

	Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
	P(0,0) = f/aspect;
	P(1,1) = f;
	P(2,2) = (mZNear + mZFar)*range_inv;
	P(2,3) = 2.0*mZNear*mZFar*range_inv;
	P(3,2) = -1.0;

	return P;
}
Eigen::Matrix4d
Camera::
getViewMatrix()
{
	Eigen::Vector3d u,v,n;
	v = mUp;
	v.normalize();
	n = mEye - mLookAt;
	n.normalize();
	u = v.cross(n);
	u.normalize();
	v = n.cross(u);

	Eigen::Matrix3d R;
	R.col(0) = u;
	R.col(1) = v;
	R.col(2) = n;
	
	R.transposeInPlace();
	Eigen::Vector3d t = -R*mEye;

	Eigen::Matrix4d V = Eigen::Matrix4d::Identity();
	V.block<3,3>(0,0) = R;
	V.block<3,1>(0,3) = t;

	return V;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
Camera::
getRay(int x, int y)
{
	GLint w = glutGet(GLUT_WINDOW_WIDTH);
	GLint h = glutGet(GLUT_WINDOW_HEIGHT);

	Eigen::Matrix4d P = this->getProjectionMatrix();
	Eigen::Matrix4d V = this->getViewMatrix();
	Eigen::Matrix4d PV_inv = (P*V).inverse();

	Eigen::Vector4d screen_pos;
	screen_pos[0] = (double)x/(double)w*2.0 - 1.0;
	screen_pos[1] = 1.0 - (double)y/(double)h*2.0;
	screen_pos[2] = -1.0;
	screen_pos[3] = 1.0;

	Eigen::Vector4d p0 = PV_inv*screen_pos;
	screen_pos[2] = 1.0;
	Eigen::Vector4d p1 = PV_inv*screen_pos;
	p0 /= p0[3];
	p1 /= p1[3];

	return std::make_pair(p0.segment<3>(0), p1.segment<3>(0));
}

double
Camera::
getPointDepth(const Eigen::Vector3d& point)
{
	Eigen::Matrix4d P = this->getProjectionMatrix();
	Eigen::Matrix4d V = this->getViewMatrix();
	Eigen::Vector4d screen_pos;
	screen_pos.segment<3>(0) = point;
	screen_pos[3] = 1.0;
	screen_pos = P*V*screen_pos;
	screen_pos /= screen_pos[3];

	return screen_pos[2];
}
Eigen::Vector3d
Camera::
getWorldPosition(int x,int y,double z)
{
	GLint w = glutGet(GLUT_WINDOW_WIDTH);
	GLint h = glutGet(GLUT_WINDOW_HEIGHT);

	Eigen::Matrix4d P = this->getProjectionMatrix();
	Eigen::Matrix4d V = this->getViewMatrix();
	Eigen::Matrix4d PV_inv = (P*V).inverse();

	Eigen::Vector4d screen_pos;
	screen_pos[0] = (double)x/(double)w*2.0 - 1.0;
	screen_pos[1] = 1.0 - (double)y/(double)h*2.0;
	screen_pos[2] = z;
	screen_pos[3] = 1.0;

	Eigen::Vector4d p = PV_inv*screen_pos;
	p /= p[3];

	return p.segment<3>(0);
}
Eigen::Vector3d
Camera::
getTrackballPoint(int x,int y)
{
	GLint w = glutGet(GLUT_WINDOW_WIDTH);
	GLint h = glutGet(GLUT_WINDOW_HEIGHT);

	float rad = std::sqrt((float)w*w + h*h)/2.0f;
	float dx = (float) (x - w)/2.0f;
	float dy = (float) (y - h)/2.0f;
	float dxdx_dydy = dx*dx + dy*dy;

	if(rad*rad > dxdx_dydy)
		return Eigen::Vector3d(dx/rad,dy/rad,std::sqrt(rad*rad - dxdx_dydy)/rad);
	else
		return Eigen::Vector3d(dx/rad,dy/rad,0.0);
}