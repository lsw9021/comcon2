#include "Environment.h"
#include "Character.h"
#include "DARTUtils.h"
#include "MathUtils.h"
#include "dart/collision/bullet/bullet.hpp"
#include "dart/collision/fcl/fcl.hpp"
using namespace dart;
using namespace dart::simulation;
using namespace dart::dynamics;
Environment::
Environment()
	:mWorld(std::make_shared<World>()),
	mControlHz(30),
	mSimulationHz(600),
	mMaxFrame(300)
{
	dart::math::Random::generateSeed(true);

	std::map<std::string, std::string> bvh_map;
	std::map<std::string, double> kp_map;
	std::map<std::string, double> mf_map;
	auto simskel = DARTUtils::buildFromFile(std::string(ROOT_DIR)+"/data/skel.xml",bvh_map, kp_map, mf_map);
	mSimCharacter = new Character(simskel);

	for(auto kp : kp_map)
		mSimCharacter->setPDGain(kp.first, kp.second);

	for(auto mf : mf_map)
		mSimCharacter->setMaxForces(mf.first, mf.second);

	std::string base_bvh_file = std::string(ROOT_DIR)+"/data/bvh/walk_long.bvh";
	std::string walk_file = std::string(ROOT_DIR)+"/data/bvh/walk.bvh";
	auto kinskel = kin::Skeleton::create(base_bvh_file);
	mKinCharacter = new kin::Character(kinskel);
	
	for(auto bvh : bvh_map)
	{
		int sid = mSimCharacter->getSkeleton()->getBodyNode(bvh.first)->getIndexInSkeleton();
		int kid = mKinCharacter->getIndexInSkeleton(bvh.second);
		
		mSimCharacter->addKinematicMap(sid, kid);		
	}

	// mKinCharacter->addMotion(base_bvh_file, 82, 85);
	// mKinCharacter->addMotion(base_bvh_file, "0:24:04 1:47:23");
	mKinCharacter->addMotion(base_bvh_file, 172, 206);
	mKinCharacter->getMotion(0)->rotate(M_PI*1.02);
	mKinCharacter->getMotion(0)->translate(-Eigen::Vector3d::UnitX()*2.3);
	mKinCharacter->addMotion(base_bvh_file, "0:24:04 1:47:23");
	// mKinCharacter->addMotion(base_bvh_file, "1:44:04 1:47:23");


	double ground_height = mKinCharacter->computeMinFootHeight() - 0.15;
	mGround = DARTUtils::createGround(ground_height);
	mWorld->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
	mWorld->addSkeleton(mSimCharacter->getSkeleton());
	mWorld->addSkeleton(mGround);
	mWorld->setTimeStep(1.0/(double)mSimulationHz);
	mWorld->setGravity(Eigen::Vector3d(0,-9.81,0.0));

	mSimCharacter->getSkeleton()->setSelfCollisionCheck(false);
	mSimCharacter->getSkeleton()->setAdjacentBodyCheck(false);

	int dim_action = getDimAction();
	mActionSpace = Eigen::VectorXd::Constant(dim_action, M_PI*2);
	mTask = false;
	this->reset();

}

int
Environment::
getDimState()
{
	return this->getState().rows();
}
int
Environment::
getDimStateAMP()
{
	return this->getStateAMP().rows();
}

int
Environment::
getDimAction()
{
	int n = mSimCharacter->getSkeleton()->getNumDofs();
	return n-6;
}

void
Environment::
reset()
{
	mContactEOE = false;
	mEOE = false;
	mFrame = 0;

	Eigen::Vector3d position, position_prev;
	Eigen::MatrixXd rotation, rotation_prev;
	Eigen::Vector3d linear_velocity;
	Eigen::MatrixXd angular_velocity;

	mKinCharacter->samplePoseFromMotion(0,
		position,rotation,linear_velocity,angular_velocity,position_prev,rotation_prev);

	Eigen::VectorXd p,v,p_prev;

	mSimCharacter->computeSimPoseAndVel(position,rotation,linear_velocity,angular_velocity,p,v);
	mSimCharacter->computeSimPose(position_prev,rotation_prev,p_prev);

	mSimCharacter->reset(p_prev, v);
	mPrevPositions2 = mSimCharacter->getPositions();
	mSimCharacter->reset(p, v);
	mPrevPositions = mSimCharacter->getPositions();
	mPrevCOM = mSimCharacter->getSkeleton()->getCOM();

	mTargetBodyNode = mSimCharacter->getSkeleton()->getBodyNode("RightArm");
	mBodyCenter = mSimCharacter->getReferenceTransform().inverse()*mTargetBodyNode->getCOM();



	mRewardPosition = 1.0;
	mRewardTask = 1.0;

	mForceTimeCount = 270;
	mForceTime = 30;
	mForceIdleTime= 300;

	this->updateForceTargetPosition();
	this->recordState();
}
void
Environment::
step(const Eigen::VectorXd& action)
{
	this->updateForceTargetPosition();
	int num_sub_steps = mSimulationHz/mControlHz;

	//#1
	Eigen::Vector3d com = mTargetBodyNode->getCOM();
	Eigen::Vector3d force = 60.0*mForceTargetPosition;
	if(mForceTimeCount<mForceTime)
		mSimCharacter->addExternalForce(mTargetBodyNode, Eigen::Vector3d::Zero(), force);
	mSimCharacter->step();


	//#2
	// std::string bn_name;
	// Eigen::Vector3d offset, force;
	// mSimCharacter->getExternalForce(bn_name, offset, force);

	// mSimCharacter->step();

	for(int i=0;i<num_sub_steps;i++)
	{
		mSimCharacter->actuate(action);
		//#1
		if(mForceTimeCount<mForceTime)
			mTargetBodyNode->addExtForce(force, Eigen::Vector3d::Zero());

		//#2
		// if(bn_name.size()!=0){
		// 	mSimCharacter->getSkeleton()->getBodyNode(bn_name)->addExtForce(force, offset);
		// }
		mWorld->step();
		// Check EOE
		auto cr = mWorld->getConstraintSolver()->getLastCollisionResult();
		for(int j=0;j<cr.getNumContacts();j++)
		{
			auto contact = cr.getContact(j);
			auto shapeFrame1 = const_cast<dart::dynamics::ShapeFrame*>(contact.collisionObject1->getShapeFrame());
			auto shapeFrame2 = const_cast<dart::dynamics::ShapeFrame*>(contact.collisionObject2->getShapeFrame());

			auto bn1 = shapeFrame1->asShapeNode()->getBodyNodePtr();
			auto bn2 = shapeFrame2->asShapeNode()->getBodyNodePtr();

			auto skel1 = bn1->getSkeleton();
			auto skel2 = bn2->getSkeleton();

			if(bn1->getName().find("Foot") != std::string::npos)
				continue;
			else if(bn2->getName().find("Foot") != std::string::npos)
				continue;

			if(skel1->getName() == "humanoid" && skel2->getName() == "ground"){
				mContactEOE = true;
				break;
			}

			if(skel1->getName() == "ground" && skel2->getName() == "humanoid"){
				mContactEOE = true;
				break;
			}

		}
	}
	this->recordState();
	mPrevPositions2 = mPrevPositions;
	mPrevPositions = mSimCharacter->getPositions();
	mPrevCOM = mSimCharacter->getSkeleton()->getCOM();
	mFrame++;

}
void
Environment::
verbose()
{
	std::cout<<mSimCharacter->getPositions().transpose()<<std::endl;
}
const bool&
Environment::
eoe()
{

	if(mContactEOE)
		mEOE = true;
	else if(mFrame>=mMaxFrame)
		mEOE = true;
	else
		mEOE = false;
	return mEOE;
}
const Eigen::MatrixXd&
Environment::
getStateAMPExperts()
{
	std::vector<Eigen::VectorXd> state_expert;
	Eigen::VectorXd u = Eigen::VectorXd::Zero(mSimCharacter->getSkeleton()->getNumDofs());
	for(auto motion : mKinCharacter->getMotions()){
		int num_frames = motion->getNumFrames();

		std::vector<Eigen::VectorXd> state_amp;
		for(int i=1;i<num_frames;i++)
		{
			Eigen::VectorXd p,p_prev;
			mSimCharacter->computeSimPose(motion->getPosition(i),motion->getRotation(i),p);
			mSimCharacter->computeSimPose(motion->getPosition(i-1),motion->getRotation(i-1),p_prev);

			Eigen::VectorXd s = mSimCharacter->getStateAMP(MathUtils::ravel(p, u), MathUtils::ravel(p_prev, u));
			state_amp.emplace_back(s);
		}
		int n = state_amp[0].rows();
		for(int i=0;i<state_amp.size()-1;i++)
		{
			Eigen::VectorXd ss1(n*2);
			ss1<<state_amp[i], state_amp[i+1];
			state_expert.emplace_back(ss1);
		}
	}

	mStateAMPExperts = MathUtils::toEigenMatrix(state_expert);
	return mStateAMPExperts;
}
void
Environment::
recordState()
{
	mState = mSimCharacter->getState();

	if(mTask){}

	double tar_speed = 1.0;
	double pos_err_scale = 0.5;
	double vel_err_scale = 4 / (tar_speed * tar_speed);

	Eigen::Vector3d tar_pos = mSimCharacter->getU().head<3>();
	Eigen::Vector3d pos = mSimCharacter->getSkeleton()->getBodyNode(0)->getCOM();
	Eigen::Vector3d root_tar_delta = tar_pos - pos;
	root_tar_delta[1] = 0.0;
	root_tar_delta[2] = 0.0;

	double root_tar_dist_sq = root_tar_delta.squaredNorm();
	double pos_reward = std::exp(-pos_err_scale * root_tar_dist_sq);

	double vel_reward = 0;
	double dist_threshold = 0.5;

	if (root_tar_dist_sq < dist_threshold * dist_threshold)
		vel_reward = 1.0;
	else
	{
		double step_dur = 1.0/30.0;

		Eigen::Vector3d com = mSimCharacter->getSkeleton()->getCOM();
		Eigen::Vector3d com_tar_delta = tar_pos - com;
		com_tar_delta[2] = 0.0;

		com_tar_delta[1] = 0.0;
		double com_tar_dist = com_tar_delta.norm();
		Eigen::Vector3d com_tar_dir = Eigen::Vector3d::Zero();
		if (com_tar_dist > 0.0001)
			com_tar_dir = com_tar_delta / com_tar_dist;

		Eigen::Vector3d com_dir = (com - mPrevCOM) / step_dur;

		com_dir[2] = 0.0;
		com_tar_dir[2] = 0.0;

		double avg_vel = com_tar_dir.dot(com_dir);
		double vel_err = tar_speed - avg_vel;

		if (avg_vel < 0)
			vel_reward = 0.0;
		else
		{
			vel_err = std::max(vel_err, 0.0);
			vel_reward = std::exp(-vel_err_scale * vel_err * vel_err);
		}
		
	}
	mRewardPosition = 0.6 * pos_reward + 0.4 * vel_reward;
	mRewardPosition = 2.0*mRewardPosition - 1.0;
	// mRewardPosition = 1.0;

	Eigen::VectorXd s = mSimCharacter->getStateAMP(mPrevPositions,
												mPrevPositions2);
	Eigen::VectorXd s1 = mSimCharacter->getStateAMP(mSimCharacter->getPositions(),
													mPrevPositions);

	mStateAMP.resize(s.rows()*2);
	mStateAMP<<s,s1;

	
}
Eigen::VectorXd
Environment::
convertToRealActionSpace(const Eigen::VectorXd& a_normalized)
{
	Eigen::VectorXd a_real;
	Eigen::VectorXd up,lo;
	up = mActionSpace;
	lo = -mActionSpace;
	a_real = dart::math::clip<Eigen::VectorXd, Eigen::VectorXd>(a_normalized, lo, up);
	return a_real;
}
void
Environment::
updateForceTargetPosition()
{
	mForceTimeCount++;
	if(mForceTimeCount<mForceIdleTime)
	{
		return;
	}
	mForceTimeCount = 0;
	double r = 0.3;
	double phi = dart::math::Random::uniform<double>(0, 2*M_PI);
	// double theta =  dart::math::Random::uniform<double>(0, M_PI);
	// mForceTargetPosition = Eigen::Vector3d(std::cos(phi),0.0, std::sin(phi));
	mForceTargetPosition = Eigen::Vector3d(1.0,0.0,0.0);
}