#ifndef __KINEMATICS_UTILS_H__
#define __KINEMATICS_UTILS_H__
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kin
{

class Utils
{
public:
	static Eigen::Vector3d computeLinearVelocity(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, double dt_inv);
	static Eigen::Vector3d computeAngularVelocity(const Eigen::Matrix3d& R0, const Eigen::Matrix3d& R1, double dt_inv);

	static Eigen::Matrix3d eulerXYZToMatrix(const Eigen::Vector3d& _angle);
	static Eigen::Matrix3d eulerZXYToMatrix(const Eigen::Vector3d& _angle);
	static Eigen::Matrix3d eulerZYXToMatrix(const Eigen::Vector3d& _angle);

	static double computeAngleDiff(double a0, double a1);
	
	static bool seed;
	// static double uniformd(double min, double max);
	static int uniformi(int min, int max);
	// static Eigen::VectorXd uniformXd(double min, double max);
};

};

#endif