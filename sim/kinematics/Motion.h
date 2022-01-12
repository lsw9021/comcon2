#ifndef __MOTION_H__
#define __MOTION_H__
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace kin
{
class Skeleton;
class Motion
{
public:
	Motion(Skeleton* skel);

	void repeat(int frame, int num);
	void repeatLastFrame(int num);
	void rotate(double y);
	void translate(const Eigen::Vector3d& tr);
	void parseBVH(const std::string& file, int start=-1, int end=-1);
	void setTimeStep(double dt){mTimeStep = dt;}
	void set(const std::vector<Eigen::Vector3d>& positions,
				const std::vector<Eigen::MatrixXd>& rotations);
	double getTimeStep(){return mTimeStep;}
	int getNumFrames(){return mNumFrames;}
	const Eigen::Vector3d& getPosition(int i){return mPositions[i];}
	const Eigen::MatrixXd& getRotation(int i){return mRotations[i];}
	const Eigen::Vector3d& getLinearVelocity(int i){return mLinearVelocities[i];}
	const Eigen::MatrixXd& getAngularVelocity(int i){return mAngularVelocities[i];}
private:
	Skeleton* mSkeleton;
	double mTimeStep;
	int mNumFrames;

	std::vector<Eigen::Vector3d> mPositions;
	std::vector<Eigen::MatrixXd> mRotations;
	std::vector<Eigen::Vector3d> mLinearVelocities;
	std::vector<Eigen::MatrixXd> mAngularVelocities;
};

};

#endif