#ifndef __KINEMATICS_SKELETON_H__
#define __KINEMATICS_SKELETON_H__
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace kin
{
class Skeleton
{
public:
	Skeleton(const std::string& name="");

	const std::string& getName(){return mName;}
	const std::vector<std::string>& getNodes(){return mNodes;}
	const std::vector<Eigen::Vector3d>& getOffsets(){return mOffsets;}
	const std::vector<int>& getParents(){return mParents;}

	int getIndex(const std::string& node){auto it = std::find(mNodes.begin(), mNodes.end(), node); return std::distance(mNodes.begin(), it);}
	int getNumJoints(){return mNumJoints;}
	static Skeleton* create(const std::string& file,const std::string& name="");
private:
	std::string mName;
	std::vector<std::string> mNodes;
	std::vector<Eigen::Vector3d> mOffsets;
	std::vector<int> mParents;
	int mNumJoints;
};
};

#endif