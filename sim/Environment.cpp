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
	mSimulationHz(300),
	mMaxFrame(200)
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
	//#0 balance
	mKinCharacter->addMotion(base_bvh_file, 82, 85);
	mKinCharacter->getMotion(0)->repeat(0,500);
	
	// mKinCharacter->addMotion(base_bvh_file, "0:24:04 1:47:23");
	//#1 Push Recovery
	// mKinCharacter->addMotion(base_bvh_file, 172, 206);
	// mKinCharacter->getMotion(0)->rotate(M_PI*1.02);
	// mKinCharacter->getMotion(0)->translate(-Eigen::Vector3d::UnitX()*2.3);
	// mKinCharacter->addMotion(base_bvh_file, "0:24:04 1:47:23");
	//#2 walking
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
	// int stride = 32;
	// mdTheta = 2*M_PI/(double)stride;
	// this->setForceFunction(Eigen::VectorXd::Constant(stride, 3.0)+Eigen::VectorXd::Random(stride));
	// mForceFunction = Eigen::VectorXd::Constant(stride, 10.0);
	// this->setForceFunction(Eigen::VectorXd::Constant(stride, 10.0));

	this->reset();
}

// double
// Environment::
// getMaxForce(double phi)
// {
// 	int i0 = (int)std::floor(phi/mdTheta);
// 	int i1 = i0+1;
// 	i0 %= mForceFunction.rows();
// 	i1 %= mForceFunction.rows();
// 	double r = phi/mdTheta - std::floor(phi/mdTheta);

// 	return (1-r)*mForceFunction[i0] + r*mForceFunction[i1];
// }
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

	//#0 balance
	mKinCharacter->samplePoseFromMotion(0,
		position,rotation,linear_velocity,angular_velocity,position_prev,rotation_prev);
	//#1 push recovery
	// mKinCharacter->samplePoseFromMotion(0,
	// 	position,rotation,linear_velocity,angular_velocity,position_prev,rotation_prev,7);

	Eigen::VectorXd p,v,p_prev;

	mSimCharacter->computeSimPoseAndVel(position,rotation,linear_velocity,angular_velocity,p,v);
	mSimCharacter->computeSimPose(position_prev,rotation_prev,p_prev);

	mSimCharacter->reset(p_prev, v);
	mPrevPositions2 = mSimCharacter->getPositions();
	mSimCharacter->reset(p, v);
	mPrevPositions = mSimCharacter->getPositions();
	mPrevCOM = mSimCharacter->getSkeleton()->getCOM();
	mPrevOrientation = mSimCharacter->getReferenceOrientation();

	mTargetBodyNode = mSimCharacter->getSkeleton()->getBodyNode("Head");
	mBodyCenter = mSimCharacter->getReferenceTransform().inverse()*mTargetBodyNode->getCOM();

	// double dhat = dart::math::Random::uniform<double>(0.0,1.0);
	// double dhat = 0.5;
	double dhat = 1.0;
	// std::cout<<dhat<<std::endl;
	mSimCharacter->setRootDHat(Eigen::Vector3d::Constant(dhat));

	mRewardPosition = 1.0;
	mRewardTask = 1.0;

	// mForceTimeCount = 270;
	mForceTime = 30;
	mForceIdleTime = mMaxFrame;
	mForceTimeCount = mMaxFrame - 30;



	// mForceTimeCount = 30;
	// mForceTime = 15;
	// mForceIdleTime= 600;
	mForceTargetPosition.setZero();
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
	Eigen::Vector3d force = mForceTargetPosition*( std::min(1.0, mForceTimeCount/(double)mForceTime + 0.3) );
	if(mForceTimeCount<mForceTime)
		mSimCharacter->addExternalForce(mTargetBodyNode, Eigen::Vector3d::Zero(), force);
	mSimCharacter->step();


	//#2
	std::string bn_name;
	Eigen::Vector3d offset, force2;
	mSimCharacter->getExternalForce(bn_name, offset, force2);

	// mSimCharacter->step();
	mSimCharacter->clearCummulatedForces();
	bool contactRF = false;
	bool contactLF = false;
	for(int i=0;i<num_sub_steps;i++)
	{
		mSimCharacter->actuate(action);
		//#1
		if(mForceTimeCount<mForceTime)
			mTargetBodyNode->addExtForce(force, Eigen::Vector3d::Zero());

		//#2
		if(bn_name.size()!=0){
			mSimCharacter->getSkeleton()->getBodyNode(bn_name)->addExtForce(force2, offset);
		}

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

			if((bn1->getName().find("RightFoot") != std::string::npos) || (bn2->getName().find("RightFoot") != std::string::npos))
				contactRF = true;
			if((bn1->getName().find("LeftFoot") != std::string::npos) || (bn2->getName().find("LeftFoot") != std::string::npos))
				contactLF = true;

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
	if(mSimCharacter->getSkeleton()->getBodyNode(0)->getCOM()[1]<0.75)
		mContactEOE = true;
	if(contactRF ==false && contactLF == false && mFrame > 30)
		mContactEOE = true;
	this->recordState();
	mPrevPositions2 = mPrevPositions;
	mPrevPositions = mSimCharacter->getPositions();
	mPrevCOM = mSimCharacter->getSkeleton()->getCOM();
	mPrevOrientation = mSimCharacter->getReferenceOrientation();
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
	if(mContactEOE){
		mEOE = true;
	}
	else if(mFrame>=mMaxFrame){
		mEOE = true;
	}
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
	Eigen::VectorXd state = mSimCharacter->getState();

	if(mTask){}

	{
		Eigen::Isometry3d T_ref = mSimCharacter->getReferenceTransform();
		Eigen::Matrix3d R_ref = T_ref.linear();
		Eigen::Matrix3d R_ref_inv = R_ref.transpose();

		Eigen::Vector3d com = mSimCharacter->getSkeleton()->getCOM();
		Eigen::Vector3d target_com_vel_local = R_ref_inv*(mSimCharacter->getU().head<3>() - com);
		target_com_vel_local[1] = 0.0;
		double target_vel_norm = target_com_vel_local.norm();
		double max_target_vel = 3.0;
		if(target_vel_norm>max_target_vel){
			target_com_vel_local /= target_vel_norm*max_target_vel;

		}
		Eigen::Vector3d com_vel = (com - mPrevCOM)*mControlHz;
		com_vel[1] = 0.0;

		Eigen::Vector3d com_vel_local = R_ref.transpose()*com_vel;
		Eigen::Vector3d state_position = com_vel_local - target_com_vel_local;
		state_position[1] = 0.0;
		double ori = mSimCharacter->getReferenceOrientation();
		double com_ang_vel = kin::Utils::computeAngleDiff(mPrevOrientation, ori)*mControlHz;

		
		mState.resize(state.rows() + 3 + 1);
		mState<<state, state_position, com_ang_vel;
		// mState.resize(state.rows() + 3);
		// mState<<state, state_position;
		double angvel = std::abs(com_ang_vel);

		mRewardPosition = 1.0;
		if(target_com_vel_local.norm()>0.2)
		{
			double vel = (target_com_vel_local - MathUtils::projectOnVector(com_vel_local, target_com_vel_local)).norm();
			mRewardPosition = std::exp(-1.5*vel*vel)*std::exp(-0.2*angvel);
			// std::cout<<"1"<<" "<<mRewardPosition<<std::endl;
		}
		else
		{
			double vel = com_vel_local.norm();

			// mRewardPosition = std::exp(-5.0*vel*vel)*std::exp(-0.5*angvel);
			// mRewardPosition = std::exp(-3.0*vel*vel);
			mRewardPosition = 1.0;
			// std::cout<<"2"<<" "<<mRewardPosition<<std::endl;
		}
		mRewardPosition = 1.0;
	}
	

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
	double phi = dart::math::Random::uniform<double>(0, 2*M_PI);
	// mForceTargetPosition = this->getMaxForce(phi)*Eigen::Vector3d(std::sin(phi),0.0, std::cos(phi));

	// double theta =  dart::math::Random::uniform<double>(0, M_PI);
	// if(phi<M_PI)
	// 	mForceTargetPosition = Eigen::Vector3d(1.0,0.0,0.0);	
	// else
	// 	mForceTargetPosition = Eigen::Vector3d(0.0,0.0,1.0);	

	// mForceTargetPosition = Eigen::Vector3d(std::cos(phi),0.0, std::sin(phi));
	// mForceTargetPosition = Eigen::Vector3d(-1.0,0.0,0.0);
}

void
Environment::
setForceTargetPosition(const Eigen::Vector3d& f)
{
	mForceTargetPosition = f;
}