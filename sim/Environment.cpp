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
	std::string door_file = std::string(ROOT_DIR)+"/data/bvh/door.bvh";

	auto kinskel = kin::Skeleton::create(base_bvh_file);
	mKinCharacter = new kin::Character(kinskel);
	
	for(auto bvh : bvh_map)
	{
		int sid = mSimCharacter->getSkeleton()->getBodyNode(bvh.first)->getIndexInSkeleton();
		int kid = mKinCharacter->getIndexInSkeleton(bvh.second);
		
		mSimCharacter->addKinematicMap(sid, kid);		
	}
	
	mKinCharacter->addMotion(door_file, 0, 120);
	mKinCharacter->getMotion(0)->repeatLastFrame(180);
	mPhaseDenom = mKinCharacter->getMotion(0)->getNumFrames(); 

	double dy = dynamic_cast<const BoxShape*>(mSimCharacter->getSkeleton()->getBodyNode("LeftFoot")->getShapeNodesWith<dart::dynamics::VisualAspect>()[0]->getShape().get())->getSize()[1]*0.5;
	double ground_height = mKinCharacter->computeMinFootHeight() - 4*dy;
	mGround = DARTUtils::createGround(ground_height);

	mWeldConstraint = nullptr;

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
	return n-6 + 6;
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
		position,rotation,linear_velocity,angular_velocity,position_prev,rotation_prev,1);
	
	Eigen::VectorXd p,v,p_prev;

	mSimCharacter->computeSimPoseAndVel(position,rotation,linear_velocity,angular_velocity,p,v);
	mSimCharacter->computeSimPose(position_prev,rotation_prev,p_prev);

	mSimCharacter->reset(p_prev, v);
	mPrevPositions2 = mSimCharacter->getPositions();
	mSimCharacter->reset(p, v);
	mPrevPositions = mSimCharacter->getPositions();
	mPrevCOM = mSimCharacter->getSkeleton()->getCOM();
	mPrevOrientation = mSimCharacter->getReferenceOrientation();

	mTargetBodyNode = mSimCharacter->getSkeleton()->getBodyNode("RightHand");
	mBodyCenter = mSimCharacter->getReferenceTransform().inverse()*mTargetBodyNode->getCOM();

	// if(mWeldConstraint == nullptr)
	// {
	// 	mWeldConstraint = std::make_shared<dart::constraint::BallJointConstraint>(mSimCharacter->getSkeleton()->getBodyNode("Head"),
	// 																			mRod->getBodyNode(0), head_com);
	// 	mWorld->getConstraintSolver()->addConstraint(mWeldConstraint);
	// }

	if(mWeldConstraint != nullptr){
		mWorld->getConstraintSolver()->removeConstraint(mWeldConstraint);
		mWeldConstraint = nullptr;
	}
	

	if(mObstacle != nullptr)
		mWorld->removeSkeleton(mObstacle);
	Eigen::Isometry3d Td = Eigen::Isometry3d::Identity();
	Td.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
	Td.translation() = Eigen::Vector3d(-1.58,0.0,0.65);

	double door_x = dart::math::Random::uniform<double>(-0.3, -0.05);
	double door_y = dart::math::Random::uniform<double>(-0.2, 0.2);
	door_x = -0.1;
	door_y = 0.0;
	mBallJointPos = Eigen::Vector3d(0.449879,1.12146,0.558332);
	mBallJointPos[0] += door_x;
	mBallJointPos[1] += door_y;
	mDoorSize = 2.0*dart::math::Random::uniform<double>(0.3, 0.9);
	// mDoorSize = 2.0*0.5;
	double w0 = 2.0 - mDoorSize;
	//[0.6,1.8]
	mDoorMass = 50.0*mDoorSize;

	mObstacle = DARTUtils::createDoor(Td, w0, mDoorSize, mDoorMass);
	mDoorKd = 100.0*std::pow(3.0,mDoorSize);
	// int r = dart::math::Random::uniform<int>(0,2);
	int xxxx = dart::math::Random::uniform<int>(0,1);
	// if(xxxx==1)
	mDoorKd = 0.1*mDoorKd;
	// mObstacle->getJoint(1)->setSpringStiffness(0, door_kp);
	mObstacle->getJoint(1)->setDampingCoefficient(0, mDoorKd);
	mWorld->addSkeleton(mObstacle);

	double dhat = 1.0;
	Eigen::Vector3d dhat_vec = Eigen::Vector3d::Constant(dhat);
	dhat_vec[1] = 0.0;
	mSimCharacter->setRootDHat(dhat_vec);

	mRewardPosition = 1.0;
	mRewardTask = 1.0;

	mForceTargetPosition.setZero();

	mObstacleForce = Eigen::Vector3d::Zero();
	this->updateForceTargetPosition();
	this->recordState();
}
void
Environment::
step(const Eigen::VectorXd& action)
{
	int num_sub_steps = mSimulationHz/mControlHz;

	mSimCharacter->clearCummulatedForces();
	bool contactRF = true;
	bool contactLF = false;
	mContactObstacle = false;
	// mObstacleForce = Eigen::Vector3d::Zero();

	int n = mSimCharacter->getSkeleton()->getNumDofs();
	Eigen::VectorXd action_pos = action.head(n-6);
	Eigen::Vector6d ghost_force = action.tail<6>();

	if(mFrame == 17)
	{
		mWeldConstraint = std::make_shared<dart::constraint::BallJointConstraint>(mSimCharacter->getSkeleton()->getBodyNode("RightHand"),
																				mObstacle->getBodyNode(1), mBallJointPos);
		mWorld->getConstraintSolver()->addConstraint(mWeldConstraint);
	}
	if(mFrame ==51)
	{
		mWorld->getConstraintSolver()->removeConstraint(mWeldConstraint);
		mWeldConstraint = nullptr;
	}


	for(int i=0;i<num_sub_steps;i++)
	{
		mSimCharacter->actuate(action);

		mSimCharacter->addGhostForce(mSimCharacter->getSkeleton()->getBodyNode("RightHand"), ghost_force);
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
			if(skel1->getName() == "humanoid" && skel2->getName() == "door"){
				// mContactObstacle = true;
				// mObstacleBodyNode = bn1;
				// mObstacleForce += contact.force;
				mContactEOE = true;
			}
			else if(skel1->getName() == "door" && skel2->getName() == "humanoid"){
				// mContactObstacle = true;
				// mObstacleBodyNode = bn2;
				// mObstacleForce -= contact.force;
				mContactEOE = true;
			}

		}
	}

	// if(mObstacle->getCOM()[1]<1.5)
	// 	mContactEOE = true;
	this->recordState();
	mPrevPositions2 = mPrevPositions;
	mPrevPositions = mSimCharacter->getPositions();
	mPrevCOM = mSimCharacter->getSkeleton()->getCOM();
	mPrevOrientation = mSimCharacter->getReferenceOrientation();
	mFrame++;

	
	if(mFrame == 17)
	{
		Eigen::Vector3d right_hand_com = mSimCharacter->getSkeleton()->getBodyNode("RightHand")->getCOM();
		double distance = (mBallJointPos - right_hand_com).norm();
		if(distance>0.05)
			mContactEOE = true;
	}
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
			Eigen::VectorXd phase = Eigen::VectorXd::Constant(10,(double)i/mPhaseDenom);
			Eigen::VectorXd ss1(n*2 + 10);
			// ss1<<state_amp[i], state_amp[i+1], s_idx;
			ss1<<state_amp[i], state_amp[i+1],phase;
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

	Eigen::Vector3d ball_joint_pos_local = T_ref.inverse()*mBallJointPos;
	double door_angle = mObstacle->getPositions()[0];
	
	// mState.resize(state.rows() + 3 + 1);
	// mState<<state, state_position, com_ang_vel;
	mState.resize(state.rows() + 3 + 1 + 3 + 1 + 3);
	mState<<state, state_position, com_ang_vel, door_angle, mDoorSize, mDoorMass, mDoorKd, ball_joint_pos_local;
	// mState.resize(state.rows() + 3 + 1 + 4);
	// mState<<state, state_position, com_ang_vel, door_angle, mDoorSize, mDoorMass,mDoorKd;

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
	mRewardPosition = 1.0;
	door_angle = std::min(0.7,door_angle);
	double door_error = 0.7 - door_angle;

	mRewardTask = std::exp(-1.0*door_error);

	mRewardPosition = mRewardPosition*mRewardTask;
	Eigen::VectorXd s = mSimCharacter->getStateAMP(mPrevPositions,
												mPrevPositions2);
	Eigen::VectorXd s1 = mSimCharacter->getStateAMP(mSimCharacter->getPositions(),
													mPrevPositions);
	int importance = 1;
	Eigen::VectorXd phase = Eigen::VectorXd::Constant(10,std::min(1.0,(double)mFrame/mPhaseDenom));
	mStateAMP.resize(s.rows() + s1.rows() + 10);
	mStateAMP<<s,s1,phase;
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
	return;
}