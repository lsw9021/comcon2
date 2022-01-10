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
	mObstacle(nullptr),
	mControlHz(30),
	mSimulationHz(450),
	mMaxFrame(300)
{
	dart::math::Random::generateSeed(true);

	std::map<std::string, std::string> bvh_map;
	std::map<std::string, double> kp_map;
	std::map<std::string, double> mf_map;
	// auto simskel = DARTUtils::buildFromFile(std::string(ROOT_DIR)+"/data/skel_heavy.xml",bvh_map, kp_map, mf_map);
	auto simskel = DARTUtils::buildFromFile(std::string(ROOT_DIR)+"/data/skel.xml",bvh_map, kp_map, mf_map);
	mSimCharacter = new Character(simskel);

	for(auto kp : kp_map)
		mSimCharacter->setPDGain(kp.first, kp.second);

	for(auto mf : mf_map)
		mSimCharacter->setMaxForces(mf.first, mf.second);

	std::string base_bvh_file = std::string(ROOT_DIR)+"/data/bvh/walk_long.bvh";
	std::string walk_file = std::string(ROOT_DIR)+"/data/bvh/walk.bvh";

	std::string locofile1 = std::string(ROOT_DIR)+"/data/bvh/LocomotionFlat01_000.bvh";
	std::string locofile6 = std::string(ROOT_DIR)+"/data/bvh/LocomotionFlat06_000.bvh";
	std::string locofile7 = std::string(ROOT_DIR)+"/data/bvh/LocomotionFlat07_000.bvh";
	std::string locofile8 = std::string(ROOT_DIR)+"/data/bvh/LocomotionFlat08_000.bvh";
	std::string locofile_mirror1 = std::string(ROOT_DIR)+"/data/bvh/LocomotionFlat01_000_mirror.bvh";
	std::string locofile_mirror6 = std::string(ROOT_DIR)+"/data/bvh/LocomotionFlat06_000_mirror.bvh";
	std::string locofile_mirror7 = std::string(ROOT_DIR)+"/data/bvh/LocomotionFlat07_000_mirror.bvh";
	std::string locofile_mirror8 = std::string(ROOT_DIR)+"/data/bvh/LocomotionFlat08_000_mirror.bvh";
	std::string chicken_file = std::string(ROOT_DIR)+"/data/bvh/chicken.bvh";
	std::string chicken_heavy_file = std::string(ROOT_DIR)+"/data/bvh/chicken_heavy.bvh";
	std::string manipulation_ball_file = std::string(ROOT_DIR)+"/data/bvh/manipulation_ball.bvh";


	auto kinskel = kin::Skeleton::create(base_bvh_file);
	mKinCharacter = new kin::Character(kinskel);
	
	for(auto bvh : bvh_map)
	{
		int sid = mSimCharacter->getSkeleton()->getBodyNode(bvh.first)->getIndexInSkeleton();
		int kid = mKinCharacter->getIndexInSkeleton(bvh.second);
		
		mSimCharacter->addKinematicMap(sid, kid);		
	}
	//#0 balance
	// mKinCharacter->addMotion(chicken_file, 100, 630);
	
	
	mKinCharacter->addMotion(base_bvh_file, 82, 85);
	// mKinCharacter->addMotion(manipulation_ball_file, 1, 4);
	mKinCharacter->getMotion(0)->repeat(0,500);
	// mKinCharacter->addMotion(manipulation_ball_file, "0:01:00 1:30:00");

	// mKinCharacter->addMotion(base_bvh_file, "0:24:04 1:47:23");
	

	// mKinCharacter->addMotion(locofile2, 60, 300);
	// mKinCharacter->addMotion(base_bvh_file, 82, 85);
	// mKinCharacter->addMotion(base_bvh_file, "0:24:04 1:47:23");
	// //walk
	// mKinCharacter->addMotion(locofile6, 1500, 1950);
	// mKinCharacter->addMotion(locofile_mirror6, 1500, 1950);
	// mKinCharacter->addMotion(locofile8, 30, 2700);
	// mKinCharacter->addMotion(locofile_mirror8, 30, 2700);
	// //run
	// mKinCharacter->addMotion(locofile6, 240, 700);
	// mKinCharacter->addMotion(locofile_mirror6, 240, 700);
	// mKinCharacter->addMotion(locofile1, 120, 270);
	// mKinCharacter->addMotion(locofile_mirror1, 120, 270);
	// mKinCharacter->addMotion(locofile7, 60, 700);
	// mKinCharacter->addMotion(locofile_mirror7, 60, 700);
	
	
	//redundant
	// mKinCharacter->addMotion(base_bvh_file, "0:24:04 1:47:23");
	




	double dy = dynamic_cast<const BoxShape*>(mSimCharacter->getSkeleton()->getBodyNode("LeftFoot")->getShapeNodesWith<dart::dynamics::VisualAspect>()[0]->getShape().get())->getSize()[1]*0.5;
	double ground_height = mKinCharacter->computeMinFootHeight() - 4*dy;
	mGround = DARTUtils::createGround(ground_height);


	mRod = DARTUtils::createRod(0.6, 0.05, 0.5);
	mWeldConstraint = nullptr;

	

	mWorld->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
	mWorld->addSkeleton(mSimCharacter->getSkeleton());
	mWorld->addSkeleton(mRod);
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
	return n-6 + 3 + 3;
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
	// mKinCharacter->samplePoseFromMotion(0,
	// 	position,rotation,linear_velocity,angular_velocity,position_prev,rotation_prev);
	//#1 push recovery
	// mKinCharacter->samplePoseFromMotion(0,
	// 	position,rotation,linear_velocity,angular_velocity,position_prev,rotation_prev,42);
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
	mPrevOrientation = mSimCharacter->getReferenceOrientation();

	mTargetBodyNode = mSimCharacter->getSkeleton()->getBodyNode("Head");
	mBodyCenter = mSimCharacter->getReferenceTransform().inverse()*mTargetBodyNode->getCOM();

	Eigen::Vector3d head_com = mSimCharacter->getSkeleton()->getBodyNode("Head")->getCOM();
	head_com[2] += 0.02;
	head_com[1] += 0.17;
	Eigen::Vector6d rod_pos, rod_vel;
	rod_pos.setZero();
	rod_vel.setZero();
	mRodRelativeTransform = mSimCharacter->getSkeleton()->getBodyNode("Head")->getTransform().inverse()*head_com;
	rod_pos.segment<3>(3) = head_com + 0.3*Eigen::Vector3d::UnitY();
	mRod->setPositions(rod_pos);
	mRod->setVelocities(rod_vel);
	if(mWeldConstraint == nullptr)
	{
		
		mWeldConstraint = std::make_shared<dart::constraint::BallJointConstraint>(mSimCharacter->getSkeleton()->getBodyNode("Head"),
																				mRod->getBodyNode(0), head_com);
		mWorld->getConstraintSolver()->addConstraint(mWeldConstraint);
	}


	if(mObstacle == nullptr)
	{
		mObstacle = DARTUtils::createBall(1000.0, 0.07, "Free");
		// mObstacle->getBodyNode(0)->setFrictionCoeff(0.5);
		mObstacle->getJoint(0)->setDampingCoefficient(0, 0.001);
		mObstacle->getJoint(0)->setDampingCoefficient(1, 0.001);
		mObstacle->getJoint(0)->setDampingCoefficient(2, 0.001);
		mWorld->addSkeleton(mObstacle);
	}
	Eigen::Vector6d ball_pos, ball_vel;
	ball_pos.setZero();
	ball_vel.setZero();
	Eigen::Vector3d ball_rand_pos = 0.05*Eigen::Vector3d::Random();
	ball_rand_pos[1] = 0.0;
	ball_pos.segment<3>(3) = head_com + 0.7*Eigen::Vector3d::UnitY() + ball_rand_pos;
	mObstacle->setPositions(ball_pos);
	mObstacle->setVelocities(ball_vel);

	// double dhat = dart::math::Random::uniform<double>(0.0,1.0);
	// double dhat = 0.5;
	double dhat = 1.0;
	Eigen::Vector3d dhat_vec = Eigen::Vector3d::Constant(dhat);
	dhat_vec[1] = 0.0;
	// std::cout<<dhat<<std::endl;
	mSimCharacter->setRootDHat(dhat_vec);

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

	mObstacleCount = 0;
	// mObstacleCount = 1e6;
	mObstacleDuration = 10;
	mObstacleIdleTime = 10;
	mContactObstacle = false;
	mObstacleBodyNode = nullptr;


	mCreateObstacle = true;
	mToggleCount = 0;
	// mToggleDuration = dart::math::Random::uniform<int>(1, 30);
	mToggleDuration = 30000;
	// if(dart::math::Random::uniform<double>(0.0,1.0)<0.5)
	// 	mSimCharacter->toggleLight();
	// if(dart::math::Random::uniform<double>(0.0, 1.0)<0.5)
	// 	mCreateObstacle = true;
	// else
	// 	mCreateObstacle = false;

	// // mCreateObstacle = true;
	// if(mCreateObstacle == false)
	// {
		
	
	// }
	
	

	mObstacleForce = Eigen::Vector3d::Zero();
	this->updateForceTargetPosition();
	// if(mObstacle!=nullptr){
	// 	// mWorld->getConstraintSolver()->removeConstraint(mWeldConstraint);
	// 	mWorld->removeSkeleton(mObstacle);
	// 	mObstacle=nullptr;
	// }
	// if(mCreateObstacle)
	// 	this->updateObstacle();
	this->recordState();
}
void
Environment::
step(const Eigen::VectorXd& action)
{
	this->updateForceTargetPosition();
	int num_sub_steps = mSimulationHz/mControlHz;

	bool train = false;

	if(mToggleCount>mToggleDuration)
	{
		mSimCharacter->toggleLight();
		if(mSimCharacter->getLight() == 1)
			mToggleDuration = 1e6;
		else
			mToggleDuration = dart::math::Random::uniform<int>(60, 90);
		mToggleCount = 0;

	}
	
	if(mSimCharacter->getLight() == 1 && mToggleCount == 0)
		forceCreateObstacle();
	mToggleCount++;
	this->updateObstacle();
	//#1
	

	//#2
	
	std::string bn_name;
	Eigen::Vector3d offset, force2;
	mSimCharacter->getExternalForce(bn_name, offset, force2);
	//#3

	// auto cr = mWorld->getConstraintSolver()->getLastCollisionResult();
	// if(mObstacleCount != 0)
	// for(int j=0;j<cr.getNumContacts();j++)
	// {
	// 	auto contact = cr.getContact(j);
	// 	auto shapeFrame1 = const_cast<dart::dynamics::ShapeFrame*>(contact.collisionObject1->getShapeFrame());
	// 	auto shapeFrame2 = const_cast<dart::dynamics::ShapeFrame*>(contact.collisionObject2->getShapeFrame());

	// 	auto bn1 = shapeFrame1->asShapeNode()->getBodyNodePtr();
	// 	auto bn2 = shapeFrame2->asShapeNode()->getBodyNodePtr();

	// 	auto skel1 = bn1->getSkeleton();
	// 	auto skel2 = bn2->getSkeleton();

		

	// }
	if(mContactObstacle){
		mSimCharacter->addExternalForce(mObstacleBodyNode, Eigen::Vector3d::Zero(), mObstacleForce);
		if(mSimCharacter->getLight() == 1){
			mSimCharacter->toggleLight();
			mToggleDuration = dart::math::Random::uniform<int>(60, 90);
			mToggleCount = 0;
		}
	}
	// if(mCreateObstacle)

	// mSimCharacter->step()
	mSimCharacter->clearCummulatedForces();
	bool contactRF = true;
	bool contactLF = false;
	mContactObstacle = false;
	// mObstacleForce = Eigen::Vector3d::Zero();

	int n = mSimCharacter->getSkeleton()->getNumDofs();
	Eigen::VectorXd action_pos = action.head(n-6);
	Eigen::Vector6d ghost_force = action.tail<6>();

	for(int i=0;i<num_sub_steps;i++)
	{
		mSimCharacter->actuate(action);

		//#2
		if(bn_name.size()!=0){

			mSimCharacter->getSkeleton()->getBodyNode(bn_name)->addExtForce(force2, offset);
			if(i>0)
				mSimCharacter->addExternalForce(mSimCharacter->getSkeleton()->getBodyNode(bn_name), offset, force2);
		}

		if(train)
		{
			Eigen::Vector3d com = mTargetBodyNode->getCOM();
			Eigen::Vector3d force = mForceTargetPosition;
			if(mForceTimeCount<mForceTime && mForceTargetPosition.norm()>1e-6){
				mSimCharacter->addExternalForce(mTargetBodyNode, Eigen::Vector3d::Zero(), force);
				mTargetBodyNode->addExtForce(mForceTargetPosition, Eigen::Vector3d::Zero());
			}
		}
		if(mObstacleCount<mObstacleDuration)
			mObstacle->getBodyNode(0)->addExtForce(mObstacleForce, Eigen::Vector3d::Zero());
		mSimCharacter->addGhostForce(mSimCharacter->getSkeleton()->getBodyNode("Head"), ghost_force);
		mSimCharacter->step();
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
			if((bn1->getName().find("LeftFoot") != std::string::npos && skel2->getName() == "ground") || (skel1->getName() == "ground" && bn2->getName().find("LeftFoot") != std::string::npos))
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
			if(skel1->getName() == "humanoid" && skel2->getName() == "Box"){
				mContactObstacle = true;
				mObstacleBodyNode = bn1;
				mObstacleForce += contact.force;
			}
			else if(skel1->getName() == "Box" && skel2->getName() == "humanoid"){
				mContactObstacle = true;
				mObstacleBodyNode = bn2;
				mObstacleForce -= contact.force;
			}

		}
	}
	Eigen::Matrix3d Rr = BallJoint::convertToRotation(mRod->getPositions());

	double cos_angle = std::max(-1.0,std::min(1.0,Rr.col(1)[1]));
	double rod_angle = std::acos(cos_angle)*180.0/M_PI;

	if(mSimCharacter->getSkeleton()->getBodyNode(0)->getCOM()[1]<0.75)
		mContactEOE = true;
	if(rod_angle>90.0)
		mContactEOE = true;
	if(mObstacle->getCOM()[1]<1.5)
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
	int count = 0;
	// int importance = 1;
	
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
		// Eigen::VectorXd s_idx;
		// if(count == 0)
		// 	s_idx  = Eigen::VectorXd::Zero(importance);
		// else
		// 	s_idx  = Eigen::VectorXd::Ones(importance);
		
		for(int i=0;i<state_amp.size()-1;i++)
		{
			// Eigen::VectorXd ss1(n*2 + importance);
			Eigen::VectorXd ss1(n*2);
			// ss1<<state_amp[i], state_amp[i+1], s_idx;
			ss1<<state_amp[i], state_amp[i+1];
			state_expert.emplace_back(ss1);
		}
		count++;
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

	Eigen::Isometry3d T_ref = mSimCharacter->getReferenceTransform();
	Eigen::Matrix3d R_ref = T_ref.linear();
	Eigen::Matrix3d R_ref_inv = R_ref.transpose();

	Eigen::Vector3d com = mSimCharacter->getSkeleton()->getCOM();
	Eigen::Vector3d target_com_vel_local = R_ref_inv*mSimCharacter->getUroot();
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

	///Compute Task state and reward
	Eigen::Vector3d ball_pos, ball_vel;
	ball_pos = T_ref.inverse()*mObstacle->getBodyNode(0)->getCOM();
	ball_vel = R_ref_inv*(mObstacle->getBodyNode(0)->getCOMLinearVelocity());
	ball_pos[1] = 0.0;
	ball_vel[1] = 0.0;

	Eigen::Vector3d rod_pos, rod_vel;
	rod_pos = T_ref.inverse()*mRod->getBodyNode(1)->getCOM();
	rod_vel = R_ref_inv*mRod->getBodyNode(1)->getCOMLinearVelocity();
	
	rod_pos[1] = 0.0;
	rod_vel[1] = 0.0;

	Eigen::Vector3d ball_target_pos_diff;

	ball_target_pos_diff = T_ref.inverse()*com - ball_pos;
	ball_target_pos_diff[1] = 0.0;


	mState.resize(state.rows() + 3 + 1 + 5*3);
	mState<<state, state_position, com_ang_vel,ball_pos,ball_vel,rod_pos,rod_vel,ball_target_pos_diff;

	double angvel = std::abs(com_ang_vel);

	mRewardPosition = 1.0;
	// if(false)
	if(mObstacleCount<45)
	{
		double vel = (target_com_vel_local - MathUtils::projectOnVector(com_vel_local, target_com_vel_local)).norm();
		double projection = com_vel_local.dot(target_com_vel_local)/target_com_vel_local.dot(target_com_vel_local);
		projection = std::min(1.0,projection);
		double vel_diff = 1.0 - projection;
		mRewardPosition = std::exp(-1.0*vel_diff*vel_diff);
		// mRewardPosition = std::exp(-1.5*vel*vel)*std::exp(-0.2*angvel);
	}
	else
	{
		double vel = com_vel_local.norm();

		// mRewardPosition = std::exp(-5.0*vel*vel)*std::exp(-0.5*angvel);
		mRewardPosition = std::exp(-2.0*vel*vel);
		// std::cout<<mRewardPosition<<std::endl;
	}
	// mRewardPosition = 1.0;
	mRewardTask = std::exp(-5.0*ball_target_pos_diff.norm());
	mRewardPosition = mRewardPosition*mRewardTask;
	// mRewardPosition = 2.0*mRewardPosition - 1.0;
	Eigen::VectorXd s = mSimCharacter->getStateAMP(mPrevPositions,
												mPrevPositions2);
	Eigen::VectorXd s1 = mSimCharacter->getStateAMP(mSimCharacter->getPositions(),
													mPrevPositions);
	int importance = 1;
	mStateAMP.resize(s.rows() + s1.rows());
	mStateAMP<<s,s1;
	// std::cout<<mRewardPosition<<std::endl;
	// int balance = mSimCharacter->getCurrentBalanceType();
	// if(balance <2)
	// {
	// 	mRewardPosition = 1.0;
	// 	Eigen::VectorXd s_idx = Eigen::VectorXd::Zero(importance);
	// 	mStateAMP.resize(s.rows() + s1.rows() + s_idx.rows());
	// 	mStateAMP<<s,s1,s_idx;
	// }
	// else
	// {
	// 	Eigen::VectorXd s_idx = Eigen::VectorXd::Ones(importance);
	// 	mStateAMP.resize(s.rows() + s1.rows() + s_idx.rows());
	// 	mStateAMP<<s,s1,s_idx;
	// }
	// std::cout<<balance<<std::endl;
	
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
void
Environment::
updateObstacle()
{
	mObstacleCount++;
	if(mObstacleCount<mObstacleIdleTime)
		return;

	// double mag = 0.7;
	// double mag = 0.5;
	// if(mFrame<150)
	// 	mag += 0.1;
	// else if(mFrame<600)
	// 	mag += 0.3;
	// else
	// 	mag += 2.0;
	double mag = 0.0;
	double theta = dart::math::Random::uniform<double>(0.0, 2*M_PI);

	mObstacleForce = mag*Eigen::Vector3d(std::cos(theta), 0.0, std::sin(theta));
	mObstacleCount = 0;
	mObstacleDuration = 10;
	mObstacleIdleTime = 120;
	// mObstacleIdleTime = dart::math::Random::uniform<int>(60,90);
	return;
	
	// if(mObstacleCount<mObstacleDuration)
	// 	return;
	// if(mObstacle!=nullptr){
	// 	// mWorld->getConstraintSolver()->removeConstraint(mWeldConstraint);
	// 	mWorld->removeSkeleton(mObstacle);
		
	// 	mObstacle=nullptr;
	// }
	
	// // Eigen::Vector3d size = Eigen::Vector3d::Constant(0.15) + 0.05*Eigen::Vector3d::Random();
	// Eigen::Vector3d com = mSimCharacter->getSkeleton()->getBodyNode(0)->getCOM();
	// Eigen::Vector3d size(2.0, 2.0, 0.4);


	// Eigen::Vector3d dir = mSimCharacter->getTargetVelocity();
	// double r = dart::math::Random::uniform<double>(1.5, 6.0);
	// dir.normalize();
	// double theta = std::atan2(dir[0],dir[2]);

	// Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	// T.translation() = com + r*dir;
	// T.linear() = Eigen::AngleAxisd(theta,Eigen::Vector3d::UnitY()).toRotationMatrix();
	// dynamic_cast<dart::dynamics::WeldJoint*>(mObstacle->getJoint(0))->setTransformFromParentBodyNode(T);
	

	// Eigen::VectorXd pos=Eigen::VectorXd::Zero(6);
	// Eigen::VectorXd vel=Eigen::VectorXd::Zero(6);









	// double r = 2.0;
	// double theta = dart::math::Random::uniform<double>(-0.2*M_PI, 0.2*M_PI);
	// Eigen::Vector3d dir(r*std::sin(theta), 0.0, r*std::cos(theta));
	// com += dir;

	// double speed = dart::math::Random::uniform<double>(3.0, 4.0);
	// Eigen::Vector3d linvel = -speed*dir;

	// Eigen::VectorXd pos=Eigen::VectorXd::Zero(6);
	// Eigen::VectorXd vel=Eigen::VectorXd::Zero(6);

	// linvel[1] += 1.5;
	// pos.tail<3>() = com;
	// vel.tail<3>() = linvel;
	// mObstacle->setPositions(pos);
	// mObstacle->setVelocities(vel);

	mWorld->addSkeleton(mObstacle);
	// mWeldConstraint = std::make_shared<dart::constraint::WeldJointConstraint>(mObstacle->getBodyNode(0));
	// mWorld->getConstraintSolver()->addConstraint(mWeldConstraint);
	// mObstacleCount = 0;
	// mObstacleDuration = 300;
	// mObstacleDuration = 1e6;
	// mObstacleDuration = dart::math::Random::uniform<int>(60,90);
	// mObstacleDuration = 300;
	// mObstacleDuration = dart::math::Random::uniform<int>(60,90);

	// mObstacleDuration = 3000;
	// double v = dart::math::Random::uniform<double>(0.0, 1.0);
	// if(v<0.5)
		
	// else{
	// 	mObstacleDuration = 300;
	// 	vel.setZero();
	// 	mObstacle->setVelocities(vel);
	// }
}