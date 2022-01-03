#ifndef __CHARACTER_H__
#define __CHARACTER_H__
#include "dart/dart.hpp"
#include "kinematics/kinematics.h"

class Character
{
public:
	Character(dart::dynamics::SkeletonPtr& skel);
	void reset(const Eigen::VectorXd& p, const Eigen::VectorXd& v);

	void addEndEffector(const std::string& bn_name);
	void addKinematicMap(int sid, int kid);
	void setPDGain(const std::string& sim_body_name, double kp);
	void setMaxForces(const std::string& sim_body_name, double mf);
	

	Eigen::Isometry3d getReferenceTransform();
	double getReferenceOrientation();

	void computeSimPose(const Eigen::Vector3d& position,
						const Eigen::MatrixXd& rotation,
						Eigen::VectorXd& p);

	void computeSimPoseAndVel(const Eigen::Vector3d& position,
							const Eigen::MatrixXd& rotation,
							const Eigen::Vector3d& linear_velocity,
							const Eigen::MatrixXd& angular_velocity,
							Eigen::VectorXd& p,
							Eigen::VectorXd& v);

	
	void actuate(const Eigen::VectorXd& action);

	const Eigen::Vector3d& getDHat(){return mdHat;}
	void setDHat(const Eigen::Vector3d& dhat){mdHat = dhat;}

	const Eigen::Vector3d& getRootDHat(){return mRootdHat;}
	void setRootDHat(const Eigen::Vector3d& dhat){mRootdHat = dhat;}

	Eigen::Matrix3d computeStiffnessMatrix(dart::dynamics::BodyNode* bn,
						const Eigen::Vector3d& offset);
	void addExternalForce(dart::dynamics::BodyNode* bn,
						const Eigen::Vector3d& offset,
						const Eigen::Vector3d& force);
	void getExternalForce(std::string& bn_name, Eigen::Vector3d& offset, Eigen::Vector3d& force);
	void step();

	Eigen::VectorXd getPositions();
	void setPositions(const Eigen::VectorXd& pu);
	Eigen::VectorXd computeDisplacedPositions(const Eigen::VectorXd& pu);
	Eigen::VectorXd computeDisplacedPositions(const Eigen::VectorXd& p, const Eigen::VectorXd& u);
	Eigen::VectorXd computeOriginalPositions(const Eigen::VectorXd& pu);
	Eigen::VectorXd computeOriginalPositions(const Eigen::VectorXd& p, const Eigen::VectorXd& u);
	Eigen::VectorXd getState();
	Eigen::VectorXd getStateAMP(const Eigen::VectorXd& pu_curr, const Eigen::VectorXd& pu_prev);

	void pushState();
	void popState();

	const Eigen::VectorXd& getU(){return mU;}
	const Eigen::VectorXd& getdU(){return mdU;}

	const Eigen::Vector3d& getUroot(){return mUroot;}
	const Eigen::Vector3d& getdUroot(){return mdUroot;}

	dart::dynamics::SkeletonPtr getSkeleton(){return mSkeleton;}
	const std::map<int, int>& getKinematicMap(){return mKinematicMap;}

	const Eigen::VectorXd& getCummulatedForces(){return mCummulatedForces;}
	void clearCummulatedForces(){mCummulatedForces = Eigen::VectorXd::Zero(mSkeleton->getNumDofs());}

	int getBalanceType(const Eigen::VectorXd& force);
	int getCurrentBalanceType(){return mCurrentBalanceType;}
	int getLight(){return mLight;}
	void toggleLight();
	Eigen::Vector3d getTargetVelocity(){return mTargetVelocity;}
private:
	dart::dynamics::SkeletonPtr mSkeleton;
	std::vector<dart::dynamics::BodyNode*> mEndEffectors;

	std::map<int, int> mKinematicMap, mSimMap;

	Eigen::VectorXd mKp, mKv, mMaxForces;

	std::vector<Eigen::VectorXd> mStates;

	Eigen::Vector3d mDefaultVelocity;
	bool mAppliedForce;
	Eigen::VectorXd mU, mdU;
	Eigen::Vector3d mUroot, mdUroot;
	Eigen::Vector3d mdHat,mRootdHat;
	Eigen::Vector3d mOffset, mForce;
	Eigen::Vector3d mLeftFootPosition, mRightFootPosition;
	std::string mBodyNodeName;

	Eigen::Vector3d mURootBar;
	Eigen::VectorXd mCummulatedForces;

	std::vector<Eigen::VectorXd> mBoundaries;
	int mCurrentBalanceType;
	int mLight;
	Eigen::Vector3d mTargetVelocity;
};

#endif