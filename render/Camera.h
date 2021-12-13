#ifndef __CAMERA_H__
#define __CAMERA_H__
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utility>
class Camera
{
public:
	Camera(const Eigen::Vector3d& look_at = Eigen::Vector3d(0.0,0.0,0.0),const Eigen::Vector3d& eye = Eigen::Vector3d(0.0,0.0,2.0),const Eigen::Vector3d& up = Eigen::Vector3d::UnitY(),float fovy = 60.0);
	void apply();

	void pan(int x,int y,int px,int py);
	void zoom(int x,int y,int px,int py);
	void rotate(int x,int y,int px,int py);
	void translate(int x,int y,int px,int py);
	Eigen::Matrix3d getCameraCoordinate();
	Eigen::Matrix4d getProjectionMatrix();
	Eigen::Matrix4d getViewMatrix();
	std::pair<Eigen::Vector3d, Eigen::Vector3d> getRay(int x, int y);
	double getPointDepth(const Eigen::Vector3d& point);
	Eigen::Vector3d getWorldPosition(int x,int y,double z);

	const Eigen::Vector3d& getLookAt(){return mLookAt;}
	const Eigen::Vector3d& getEye(){return mEye;}
	const Eigen::Vector3d& getUp(){return mUp;}

	void setLookAt(const Eigen::Vector3d& v){mLookAt = v;}
	void setEye(const Eigen::Vector3d& v){mEye = v;}
	void setUp(const Eigen::Vector3d& v){mUp = v;}

private:
	Eigen::Vector3d getTrackballPoint(int x,int y);

	Eigen::Vector3d mLookAt;
	Eigen::Vector3d mEye;
	Eigen::Vector3d mUp;

	float mFovy;
	float mZNear, mZFar;

};
#endif