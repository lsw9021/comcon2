#include "Character.h"
#include "Motion.h"
#include "Utils.h"
using namespace kin;

Character::
Character(Skeleton* skel)
	:mSkeleton(skel)
{

}
void
Character::
setPose(const Eigen::Vector3d& pos, const Eigen::MatrixXd& rot)
{
	mPosition = pos;
	mRotation = rot;
}
Eigen::Vector3d
Character::
getJointPosition(const std::string& node)
{
	std::vector<Eigen::Isometry3d> Ts;

	int current = this->getIndexInSkeleton(node);
	int count = 0;
	auto parents = mSkeleton->getParents();
	auto offsets = mSkeleton->getOffsets();
	while(current != -1)
	{
		if(parents[current] == -1)
		{
			Eigen::Isometry3d T;
			T.linear() = mRotation.block<3,3>(0,3*current);
			T.translation() = mPosition;
			Ts.emplace_back(T);
		}
		else
		{
			Eigen::Isometry3d T;
			T.linear() = mRotation.block<3,3>(0,3*current);
			T.translation() = offsets[current];
			Ts.emplace_back(T);
		}
		count++;
		current = parents[current];
	}
	for(int i=count-2;i>=0;i--)
		Ts[i] = Ts[i+1]*Ts[i];

	return Ts[0].translation();
}
double
Character::
computeMinFootHeight()
{
	double y = 1e6;
	for(auto motion: mMotions)
	{
		int nf = motion->getNumFrames();
		for(int i=0;i<nf;i++)
		{
			this->setPose(motion->getPosition(i),motion->getRotation(i));
			Eigen::Vector3d lf = this->getJointPosition("simLeftFoot");
			Eigen::Vector3d rf = this->getJointPosition("simRightFoot");
			y = std::min(y, lf[1]);
			y = std::min(y, rf[1]);
		}
	}
	return y;
}
#include <iostream>
void
Character::
samplePoseFromMotion(
		int mid,
		Eigen::Vector3d& position,
		Eigen::MatrixXd& rotation,
		Eigen::Vector3d& linear_velocity,
		Eigen::MatrixXd& angular_velocity,
		Eigen::Vector3d& position_prev,
		Eigen::MatrixXd& rotation_prev,
		int prefered_motion_frame)
{
	int num_frames = mMotions[mid]->getNumFrames();

	int frame = Utils::uniformi(1, num_frames-1);
	if(prefered_motion_frame>=1)
		frame = prefered_motion_frame;
	position = mMotions[mid]->getPosition(frame);
	rotation = mMotions[mid]->getRotation(frame);
	linear_velocity = mMotions[mid]->getLinearVelocity(frame);
	angular_velocity = mMotions[mid]->getAngularVelocity(frame);
	position_prev = mMotions[mid]->getPosition(frame-1);
	rotation_prev = mMotions[mid]->getRotation(frame-1);
}
void
Character::
addMotion(const std::string& bvh_file, int start, int end)
{
	Motion* motion = new Motion(mSkeleton);
	motion->parseBVH(bvh_file, start, end);

	mMotions.emplace_back(motion);
}
void
Character::
addMotion(const std::string& bvh_file, const std::string& time_line)
{
	int fps = 30;
	std::stringstream ss(time_line);

	std::string start_str;
	std::string end_str;
	ss>>start_str>>end_str;

	std::stringstream ss1(start_str);
	std::stringstream ss2(end_str);
	std::string word;
	int minute, second, frame;
	std::getline(ss1, word, ':');
	minute = std::stoi(word);
	std::getline(ss1, word, ':');
	second = std::stoi(word);
	std::getline(ss1, word, ':');
	frame = std::stoi(word);

	int start = (minute*60+second)*fps + frame;

	std::getline(ss2, word, ':');
	minute = std::stoi(word);
	std::getline(ss2, word, ':');
	second = std::stoi(word);
	std::getline(ss2, word, ':');
	frame = std::stoi(word);

	int end = (minute*60+second)*fps + frame;
			
	this->addMotion(bvh_file, start, end);
	
}
int
Character::
getIndexInSkeleton(const std::string& node)
{
	return mSkeleton->getIndex(node);
}