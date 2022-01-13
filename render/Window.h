#ifndef __WINDOW_H__
#define __WINDOW_H__
#include "GLUTWindow3D.h"
#include <utility>
#include <chrono>
#include <Eigen/Core>
#include "Environment.h"
#include "DARTRendering.h"
#include "OBJ.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/embed.h>
namespace py = pybind11;

class Window : public GLUTWindow3D
{
public:
	Window();

	Environment* mEnvironment;
	void render() override;

	void reset();
	void step();

	void initNN(const std::string& config);
	void loadNN(const std::string& checkpoint);
protected:
	void keyboard(unsigned char key, int x, int y) override;
	void special(int key, int x, int y) override;
	void mouse(int button, int state, int x, int y) override;
	void motion(int x, int y) override;
	void reshape(int w, int h) override;
	void timer(int tic) override;

	std::vector<unsigned char> mScreenshotTemp;
	std::vector<unsigned char> mScreenshotTemp2;
	bool mCapture;
	void capture_screen();
	void save_pose();

	bool mPlay;
	
	DARTRendering::Option mSimRenderOption;
	DARTRendering::Option mKinRenderOption;
	DARTRendering::Option mTargetRenderOption;

	bool mFocus;
	bool mUseNN;

	bool mRenderContactForce;
	bool mRenderTargetPosition;
	py::scoped_interpreter guard;
	py::object mm,mns,sys_module;
	py::module trainer_md;
	py::object runner;
	std::string mCheckPoint;

	dart::dynamics::BodyNode* mTargetBodyNode;
	Eigen::Vector3d mTargetLocalPosition, mTargetPosition;
	double mTargetDepth;
	std::vector<double> mDhats;
	std::vector<std::vector<Eigen::Vector3d>> mCOMTrajectories;

	OBJ* mHat;
	OBJ* mGlasses;

	Eigen::VectorXd mAction;
	std::vector<std::vector<Eigen::Vector3d>> mHandTrajectories;
	Eigen::Vector3d mLocalBallJointPos;

	std::vector<Eigen::Vector3d> mConstraintForces;
};


#endif