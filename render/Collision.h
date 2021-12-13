#ifndef __COLLISION_H__
#define __COLLISION_H__

#include <Eigen/Core>
#include <Eigen/Geometry>

class Collision
{
public:
	static double checkLineBox(const Eigen::Isometry3d& T_b,
						const Eigen::Vector3d& b_min,
						const Eigen::Vector3d& b_max,
						const Eigen::Vector3d& l0,
						const Eigen::Vector3d& l1,
						Eigen::Vector3d& hit);

	static double checkLineSphere(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& center, double r, Eigen::Vector3d& hit);
private:
	static bool getIntersection(double f_dst1, double f_dst2, const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, double& t, Eigen::Vector3d& hit);
	static bool inBox(const Eigen::Vector3d& hit, const Eigen::Vector3d& b_min, const Eigen::Vector3d& b_max, int axis);
};




#endif