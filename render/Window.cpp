#include "Window.h"
#include "Camera.h"
#include <fstream>
#include <time.h>
#include <iostream>
#include "DrawUtils.h"
#include "Collision.h"
#include "DARTRendering.h"
#include <Eigen/Core>
#include <Eigen/SVD>
#include <dart/dart.hpp>

using namespace py::literals;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

Window::
Window()
	:GLUTWindow3D(),
	mPlay(false),
	mFocus(true),
	mCapture(false),
	mRenderContactForce(false),
	mRenderTargetPosition(true)
{
	mEnvironment = new Environment();
	
	mCamera->setLookAt(Eigen::Vector3d(0.0,1.3,0.8));
	mCamera->setEye( Eigen::Vector3d(-2.0,1.3,-0.8));

	this->reset();
	Eigen::Vector3d com = mEnvironment->getSimCharacter()->getSkeleton()->getBodyNode(0)->getCOM();
	mCamera->setLookAt(com);
	mCamera->setEye( com + Eigen::Vector3d(2.0,0.0,0.0));

	mHat = new OBJ(std::string(ROOT_DIR)+"/data/obj/hat.obj");
	mGlasses = new OBJ(std::string(ROOT_DIR)+"/data/obj/glasses.obj");
}
void
Window::
initNN(const std::string& config)
{
	mUseNN = true;

	mm = py::module::import("__main__");
	mns = mm.attr("__dict__");
	sys_module = py::module::import("sys");
	py::str module_dir = (std::string(ROOT_DIR)+"/python").c_str();
	sys_module.attr("path").attr("insert")(1, module_dir);

	trainer_md = py::module::import("trainer");
	runner = trainer_md.attr("build_runner")(config);
	runner.attr("set_random_force_from_function")();
}
void
Window::
loadNN(const std::string& checkpoint)
{
	mCheckPoint = checkpoint;
	std::cout<<"load "<<mCheckPoint<<std::endl;
	trainer_md.attr("load_runner")(runner, checkpoint);
	runner.attr("set_random_force_from_function")();
	mEnvironment->setForceTargetPosition(runner.attr("force").cast<Eigen::Vector3d>());
	// Eigen::VectorXd ff = runner.attr("get_force_function")().cast<Eigen::VectorXd>();
	// mEnvironment->setForceFunction(ff);
}
void
Window::
render()
{
	initLights();
	glEnable(GL_FOG);
GLfloat fogColor[] = {1,1,1,1};
  glFogfv(GL_FOG_COLOR,fogColor);
  glFogi(GL_FOG_MODE,GL_LINEAR);
  glFogf(GL_FOG_START, 5.0);
  glFogf(GL_FOG_END, 20.0);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_MULTISAMPLE);
	if(DrawUtils::initialized == false){
		DrawUtils::buildMeshes();
		DARTRendering::gRenderOption.texture_id = MeshUtils::buildTexture((std::string(ROOT_DIR)+"/data/textures/ground.png").c_str());
		mSimRenderOption.texture_id = MeshUtils::buildTexture((std::string(ROOT_DIR)+"/data/textures/simchar.png").c_str());
		mKinRenderOption.texture_id = MeshUtils::buildTexture((std::string(ROOT_DIR)+"/data/textures/kinchar.png").c_str());
		mTargetRenderOption.texture_id = MeshUtils::buildTexture((std::string(ROOT_DIR)+"/data/textures/targetchar.png").c_str());
	}
	glColor4f(0.4,0.4,1.2,0.2);
	float y = mEnvironment->getGround()->getBodyNode(0)->getTransform().translation()[1] +
	 		dynamic_cast<const BoxShape*>(mEnvironment->getGround()->getBodyNode(0)->getShapeNodesWith<VisualAspect>()[0]->getShape().get())->getSize()[1]*0.5;
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glDisable(GL_LIGHTING);
	double width = 0.005;
	int count = 0;
	glBegin(GL_QUADS);
	for(double x = -100.0;x<100.01;x+=1.0)
	{
		for(double z = -100.0;z<100.01;z+=1.0)
		{
			if(count%2==0)
				glColor3f(216.0/255.0,211.0/255.0,204.0/255.0);			
			else
				glColor3f(216.0/255.0-0.1,211.0/255.0-0.1,204.0/255.0-0.1);
			count++;
			glVertex3f(x,y,z);
			glVertex3f(x+1.0,y,z);
			glVertex3f(x+1.0,y,z+1.0);
			glVertex3f(x,y,z+1.0);
		}
	}
	glEnd();
	glEnable(GL_LIGHTING);

	DARTRendering::drawSkeleton(mEnvironment->getSimCharacter()->getSkeleton(),mSimRenderOption);
	
	if(mEnvironment->getObstacle()->getJoint(1)->getDampingCoefficient(0)<100.0)
		glColor3f(0.0,1.5,0.0);
	else
		glColor3f(1.5,0.0,0.0);
	DARTRendering::drawSkeleton(mEnvironment->getObstacle(),mKinRenderOption);
	glColor3f(0,0,0);
	for(int i =0;i<mHandTrajectories.size();i++)
	{
		glBegin(GL_LINE_STRIP);
		for(int j=0;j<mHandTrajectories[i].size();j++)
			glVertex3f(mHandTrajectories[i][j][0],mHandTrajectories[i][j][1],mHandTrajectories[i][j][2]);
		glEnd();
	}
	
	if(mRenderTargetPosition)
	{
		mEnvironment->getSimCharacter()->pushState();
		Eigen::VectorXd pu = mEnvironment->getSimCharacter()->getPositions();
		Eigen::VectorXd p = mEnvironment->getSimCharacter()->computeDisplacedPositions(pu);
		
		mEnvironment->getSimCharacter()->getSkeleton()->setPositions(p);

		DARTRendering::drawSkeleton(mEnvironment->getSimCharacter()->getSkeleton(),mKinRenderOption);
		mEnvironment->getSimCharacter()->popState();
	}
	
	glPushMatrix();
	Eigen::Vector3d ps = mEnvironment->getObstacle()->getBodyNode(1)->getTransform()*mLocalBallJointPos;
	glTranslatef(ps[0],ps[1],ps[2]);
	DrawUtils::drawSphere(0.05);
	glPopMatrix();
	
	
	if(mCapture)
		this->capture_screen();
}

void
Window::
reset()
{
	mEnvironment->reset();
	mCOMTrajectories.emplace_back(std::vector<Eigen::Vector3d>());
	mDhats.emplace_back(mEnvironment->getSimCharacter()->getRootDHat().norm());
	if(mUseNN)
	{
		runner.attr("set_random_force_from_function")();
		mEnvironment->setForceTargetPosition(runner.attr("force").cast<Eigen::Vector3d>());
		std::cout<<runner.attr("force_index").cast<double>()<<std::endl;
	}
	mAction = Eigen::VectorXd::Zero(mEnvironment->getDimAction());
	mHandTrajectories.emplace_back(std::vector<Eigen::Vector3d>());
	mLocalBallJointPos = mEnvironment->getObstacle()->getBodyNode(1)->getTransform().inverse()*mEnvironment->mBallJointPos;
}
void
Window::
step()
{
	Eigen::VectorXd action = Eigen::VectorXd::Zero(mEnvironment->getDimAction());
	Eigen::VectorXd obs = mEnvironment->getState();
	if(mUseNN)
	{
		action = runner.attr("compute_action")(obs).cast<Eigen::VectorXd>();
	}
	mAction = action;
	mCOMTrajectories.back().emplace_back(mEnvironment->getSimCharacter()->getReferenceTransform().translation());

	if(mTargetBodyNode!=nullptr)
	{
		Eigen::Vector3d force = 20.0*(mTargetPosition - mTargetBodyNode->getTransform()*mTargetLocalPosition);
		mEnvironment->getSimCharacter()->addExternalForce(mTargetBodyNode,mTargetLocalPosition,force);
	}
	mEnvironment->step(action);

	if(mEnvironment->mFrame>=17 && mEnvironment->mFrame<=51)
	{
		mHandTrajectories.back().emplace_back(mEnvironment->getSimCharacter()->getSkeleton()->getBodyNode("RightHand")->getCOM());
	}

	if(mFocus)
	{
		Eigen::Vector3d com = mEnvironment->getSimCharacter()->getSkeleton()->getBodyNode(0)->getCOM();	
		com[1] = 1.0;
		mCamera->setLookAt(com);
		mCamera->setEye( com + Eigen::Vector3d(3.0,0.0,-3.0));
	}
	
	

	// Eigen::VectorXd s_amp = mEnvironment->getStateAMP();
	// if(mEnvironment->eoe())
	// 	this->reset();
}

void
Window::
keyboard(unsigned char key, int x, int y)
{
	Eigen::Vector3d d_hat = mEnvironment->getSimCharacter()->getDHat();
	switch(key)
	{
		case 's':this->step();break;
		case 'r':this->reset();break;
		case 'f':mFocus = !mFocus;break;
		case 'C':mCapture=true;break;
		case 'x':mEnvironment->forceCreateObstacle();break;
		case 'l':this->loadNN(mCheckPoint);break;
		case '1':mRenderTargetPosition = !mRenderTargetPosition;break;
		case '2':d_hat[0] /=0.1;break;
		case '3':d_hat[1] *=0.1;break;
		case '4':d_hat[1] /=0.1;break;
		case '5':d_hat[2] *=0.1;break;
		case '6':d_hat[2] /=0.1;break;
		case '7':d_hat = Eigen::Vector3d::Constant(1e-3);break;
		case '8':d_hat *= 0.5;break;
		case '9':d_hat *= 2.0;break;
		case 'z':mEnvironment->getSimCharacter()->toggleLight();break;

		case ' ':mPlay = !mPlay; break;
		default:GLUTWindow3D::keyboard(key,x,y);break;
	}
	Eigen::Vector3d lo = Eigen::Vector3d::Zero();
	Eigen::Vector3d up = Eigen::Vector3d::Ones();
	d_hat = dart::math::clip<Eigen::Vector3d, Eigen::Vector3d>(d_hat, lo, up);
	mEnvironment->getSimCharacter()->setDHat(d_hat);
}
void
Window::
special(int key, int x, int y)
{
	switch(key)
	{
		case 100: break;//Left
		case 101: break;//Up
		case 102: break;//Right
		case 103: break;//bottom
		default:GLUTWindow3D::special(key,x,y);break;
	}

}
void
Window::
mouse(int button, int state, int x, int y)
{
	GLUTWindow3D::mouse(button,state,x,y);
	if(mMouse != 2) // Right
		return;
	if(state==0)
	{
		auto ray = mCamera->getRay(x, y);
		auto skel = mEnvironment->getSimCharacter()->getSkeleton();

		double min_t = 1e6;
		BodyNode* min_bn = nullptr;
		Eigen::Vector3d min_hit;		
		for(int i=0;i<skel->getNumBodyNodes();i++)
		{
			auto bn = skel->getBodyNode(i);
			auto sns = bn->getShapeNodesWith<VisualAspect>();
			auto shape = sns.back()->getShape().get();
			Eigen::Isometry3d T = sns.back()->getTransform();
			if(shape->is<SphereShape>()){
				const auto* sphere = dynamic_cast<const SphereShape*>(shape);
				Eigen::Vector3d hit;
				double collide = Collision::checkLineSphere(ray.first, ray.second, T.translation(), sphere->getRadius(), hit);
				if(collide>=0.0 && collide<min_t)
				{
					min_t = collide;
					min_bn = bn;
					min_hit = hit;
				}
			}
			else if(shape->is<BoxShape>()){
				const auto* box = dynamic_cast<const BoxShape*>(shape);
				Eigen::Vector3d hit;
				Eigen::Vector3d b_min,b_max;
				b_min = -0.5*box->getSize();
				b_max = 0.5*box->getSize();
				double collide = Collision::checkLineBox(T, b_min, b_max, ray.first, ray.second, hit);
				if(collide>=0.0 && collide<min_t)
				{
					min_t = collide;
					min_bn = bn;
					min_hit = hit;
				}
			}
			if(min_bn!=nullptr)
			{
				mTargetBodyNode = min_bn;
				mTargetLocalPosition = mTargetBodyNode->getTransform().inverse()*min_hit;
				mTargetPosition = min_hit;
				mTargetDepth = min_t;
			}
			else
			{
				mTargetBodyNode = nullptr;
				mTargetLocalPosition.setZero();
				mTargetPosition.setZero();
				mTargetDepth = -1.0;
			}
		}
	}
	else{
		mTargetBodyNode = nullptr;
		mTargetLocalPosition.setZero();
		mTargetPosition.setZero();
		mTargetDepth = -1.0;
	}
	
}
void
Window::
motion(int x, int y)
{
	GLUTWindow3D::motion(x,y);
	if(mMouse == 2 && mDrag)
	{
		auto ray = mCamera->getRay(x,y);
		mTargetPosition = ray.first + mTargetDepth*(ray.second - ray.first);
	}
}
void
Window::
reshape(int w, int h)
{
	mScreenshotTemp.resize(4*w*h);
	mScreenshotTemp2.resize(4*w*h);
	GLUTWindow3D::reshape(w,h);
}
void
Window::
timer(int tic)
{
	if(mPlay)
		this->step();
	GLUTWindow3D::timer(tic);
}
#include <chrono>
#include <ctime>
#include "lodepng.h"

std::string timepoint_to_string(const std::chrono::system_clock::time_point& p_tpTime,
                                           const std::string& p_sFormat)
{
    auto converted_timep = std::chrono::system_clock::to_time_t(p_tpTime);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&converted_timep), p_sFormat.c_str());
 
    return oss.str();
}

void
Window::
capture_screen()
{
	static int count = 0;
	static std::string path = timepoint_to_string(std::chrono::system_clock::now(), "../data/png/%Y_%m_%d:%H_%M_%S");
	if(count == 0){
		std::string command = "mkdir " + path;
		system(command.c_str());	
	}
	
	
	char file_name[256];
	std::string file_base = "Capture";

	std::snprintf(file_name, sizeof(file_name), "%s%s%s%.4d.png",
				path.c_str(), "/", file_base.c_str(), count++);

	int tw = glutGet(GLUT_WINDOW_WIDTH);
	int th = glutGet(GLUT_WINDOW_HEIGHT);

	glReadPixels(0, 0,  tw, th, GL_RGBA, GL_UNSIGNED_BYTE, &mScreenshotTemp[0]);

	// reverse temp2 temp1
	for (int row = 0; row < th; row++) {
	memcpy(&mScreenshotTemp2[row * tw * 4],
		   &mScreenshotTemp[(th - row - 1) * tw * 4], tw * 4);
	}
	
	unsigned result = lodepng::encode(file_name, mScreenshotTemp2, tw, th);

	// if there's an error, display it
	if (result) {
	std::cout << "lodepng error " << result << ": "
			<< lodepng_error_text(result) << std::endl;
	} else {
		std::cout << "wrote screenshot " << file_name << "\n";
	}
}
void
Window::
save_pose()
{
	// static std::string path = timepoint_to_string(std::chrono::system_clock::now(), "../data/png/%Y_%m_%d:%H_%M_%S");
	// if(count == 0){
	// 	std::string command = "mkdir " + path;
	// 	system(command.c_str());	
	// }
	
	
	// char file_name[256];
	// std::string file_base = "Capture";

	// std::snprintf(file_name, sizeof(file_name), "%s%s%s%.4d.png",
	// 			path.c_str(), "/", file_base.c_str(), count++);

	// int tw = glutGet(GLUT_WINDOW_WIDTH);
	// int th = glutGet(GLUT_WINDOW_HEIGHT);

	// glReadPixels(0, 0,  tw, th, GL_RGBA, GL_UNSIGNED_BYTE, &mScreenshotTemp[0]);

	// // reverse temp2 temp1
	// for (int row = 0; row < th; row++) {
	// memcpy(&mScreenshotTemp2[row * tw * 4],
	// 	   &mScreenshotTemp[(th - row - 1) * tw * 4], tw * 4);
	// }
	
	// unsigned result = lodepng::encode(file_name, mScreenshotTemp2, tw, th);

	// // if there's an error, display it
	// if (result) {
	// std::cout << "lodepng error " << result << ": "
	// 		<< lodepng_error_text(result) << std::endl;
	// } else {
	// 	std::cout << "wrote screenshot " << file_name << "\n";
	// }
}