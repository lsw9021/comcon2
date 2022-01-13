#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__
#include "dart/dart.hpp"
#include "kinematics/kinematics.h"
class Character;
class Environment
{
public:
	Environment();

	int getDimState();
	int getDimStateAMP();
	int getDimAction();

	void reset();
	void step(const Eigen::VectorXd& action);
	void verbose();
	
	const double& getRewardPosition(){return mRewardPosition;}
	const double& getRewardTask(){return mRewardTask;}
	const bool& eoe();
	
	const Eigen::MatrixXd& getStateAMPExperts();

	const Eigen::VectorXd& getState(){return mState;}
	const Eigen::VectorXd& getStateAMP(){return mStateAMP;}
	
	const Eigen::VectorXd& getForceInfo();

	const dart::simulation::WorldPtr& getWorld(){return mWorld;}
	const dart::dynamics::SkeletonPtr& getGround(){return mGround;}
	Character* getSimCharacter(){return mSimCharacter;}
	const Eigen::VectorXd& getPrevPositions(){return mPrevPositions;}
	const Eigen::VectorXd& getPrevPositions2(){return mPrevPositions2;}

	void updateForceTargetPosition();

	dart::dynamics::BodyNode* getTargetBodyNode(){return mTargetBodyNode;}
	Eigen::Vector3d getForceTargetPosition(){
		if(mForceTimeCount<mForceTime)
			return mForceTargetPosition;
		return Eigen::Vector3d::Zero();}

	// const Eigen::VectorXd& getForceFunction(){return mForceFunction;}
	// void setForceFunction(const Eigen::VectorXd& f);
	// double getMaxForce(double phi);

	void setForceTargetPosition(const Eigen::Vector3d& f);
	const Eigen::Vector3d& getAppliedForce(){return mForceTargetPosition;}

	void updateObstacle();
	void forceCreateObstacle(){mObstacleCount = 999999;}

	dart::dynamics::SkeletonPtr getObstacle(){return mObstacle;}

	const Eigen::Vector3d& getObstacleForce(){return mObstacleForce;}
	bool isObstacleForced(){return mObstacleCount<mObstacleDuration;}
	Eigen::Vector3d mRodRelativeTransform;
	Eigen::Vector3d mConstraintForce;
public:
	void recordState();

	Eigen::VectorXd convertToRealActionSpace(const Eigen::VectorXd& a_normalized);

	Eigen::VectorXd mActionSpace;

	dart::simulation::WorldPtr mWorld;
	int mControlHz, mSimulationHz;
	int mFrame, mMaxFrame;

	Character* mSimCharacter;
	kin::Character* mKinCharacter;

	dart::dynamics::SkeletonPtr mGround;
	dart::dynamics::SkeletonPtr mObstacle;
	bool mContactObstacle;
	Eigen::Vector3d mObstacleForce;
	dart::dynamics::BodyNode* mObstacleBodyNode;

	int mObstacleCount, mObstacleDuration,mObstacleIdleTime;

	Eigen::VectorXd mPrevPositions, mPrevPositions2;
	Eigen::Vector3d mPrevCOM;
	double mPrevOrientation;

	Eigen::VectorXd mState, mStateAMP;
	double mRewardPosition, mRewardTask;

	Eigen::MatrixXd mStateAMPExperts;

	bool mContactEOE;
	bool mEOE;

	dart::dynamics::BodyNode* mTargetBodyNode;
	Eigen::Vector3d mForceTargetPosition;
	Eigen::Vector3d mBodyCenter;
	int mForceTimeCount,mForceTime,mForceIdleTime;

	bool mTask;
	Eigen::VectorXd mStateTask;

	int mToggleCount, mToggleDuration;
	bool mCreateObstacle;

	dart::constraint::BallJointConstraintPtr mWeldConstraint;

	double mPhaseDenom;
	Eigen::Vector3d mBallJointPos = Eigen::Vector3d(0.449879,1.12146,0.558332);
	double mDoorMass;
	double mDoorSize;
	double mDoorKd;
};

#endif