#ifndef __KINEMATICS_CHARACTER_H__
#define __KINEMATICS_CHARACTER_H__
#include "Motion.h"
#include "Skeleton.h"
namespace kin
{
class Character
{
public:
	Character(Skeleton* skel);

	void setPose(const Eigen::Vector3d& pos, const Eigen::MatrixXd& rot);
	Eigen::Vector3d getJointPosition(const std::string& node);
	double computeMinFootHeight();
	void samplePoseFromMotion(
		int mid,
		Eigen::Vector3d& position,
		Eigen::MatrixXd& rotation,
		Eigen::Vector3d& linear_velocity,
		Eigen::MatrixXd& angular_velocity,
		Eigen::Vector3d& position_prev,
		Eigen::MatrixXd& rotation_prev);
	void addMotion(const std::string& bvh_file, int start=-1, int end=-1);
	void addMotion(const std::string& bvh_file, const std::string& time_line);
	Motion* getMotion(int i){return mMotions[i];}
	int getNumMotions(){return mMotions.size();}
	const std::vector<Motion*>& getMotions(){return mMotions;}
	int getIndexInSkeleton(const std::string& node);
	Skeleton* getSkeleton(){return mSkeleton;}
private:
	Skeleton* mSkeleton;
	std::vector<Motion*> mMotions;

	Eigen::Vector3d mPosition;
	Eigen::MatrixXd mRotation;
};
};

#endif