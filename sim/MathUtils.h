#ifndef __MATH_UTILS_H__
#define __MATH_UTILS_H__
#include <vector>
#include <cstdarg>
#include <Eigen/Core>
#include <Eigen/Geometry>
class MathUtils
{
public:

	static Eigen::Vector3d projectOnVector(const Eigen::Vector3d& u, const Eigen::Vector3d& v);
	static Eigen::Isometry3d orthonormalize(const Eigen::Isometry3d& T_old);
	static Eigen::VectorXd ravel(const std::vector<Eigen::Vector3d>& vv);
	static Eigen::VectorXd ravel(const std::vector<Eigen::VectorXd>& vv);
	static Eigen::MatrixXd toEigenMatrix(const std::vector<Eigen::VectorXd>& vv);
	static Eigen::MatrixXd toEigenMatrix(const std::vector<Eigen::Vector3d>& vv);

template<typename T>
	static T ravel(const T& first) {
		return first;
	}

	template<typename T, typename... Args>
	static T ravel(const T& first, Args... args) {
		Eigen::VectorXd second = ravel(args...);
		Eigen::VectorXd ret(first.rows() + second.rows());
		ret<<first,second;
		return ret;
	}


};
#endif