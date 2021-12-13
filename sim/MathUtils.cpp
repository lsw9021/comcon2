#include "MathUtils.h"



Eigen::Vector3d
MathUtils::
projectOnVector(const Eigen::Vector3d& u, const Eigen::Vector3d& v)
{
	Eigen::Vector3d projection;
	projection = v.dot(u)/v.dot(v)*v;
	return projection;
}
Eigen::Isometry3d
MathUtils::
orthonormalize(const Eigen::Isometry3d& T_old)
{
	Eigen::Isometry3d T;
	T.translation() = T_old.translation();
	Eigen::Vector3d v0,v1,v2;
	Eigen::Vector3d u0,u1,u2;
	v0 = T_old.linear().col(0);
	v1 = T_old.linear().col(1);
	v2 = T_old.linear().col(2);

	u0 = v0;
	u1 = v1 - projectOnVector(u0,v1);
	u2 = v2 - projectOnVector(u0,v2) - projectOnVector(u1,v2);

	u0.normalize();
	u1.normalize();
	u2.normalize();

	T.linear().col(0) = u0;
	T.linear().col(1) = u1;
	T.linear().col(2) = u2;
	return T;
}

Eigen::VectorXd
MathUtils::
ravel(const std::vector<Eigen::Vector3d>& vv)
{
	Eigen::VectorXd ret(vv.size()*3);
	for(int i = 0;i<vv.size();i++)
		ret.segment<3>(i*3) = vv[i];
	return ret;
}
Eigen::VectorXd
MathUtils::
ravel(const std::vector<Eigen::VectorXd>& vv)
{
	int n=0, o=0;
	for(int i = 0;i<vv.size();i++)
		n += vv[i].rows();
	Eigen::VectorXd ret(n);
	for(int i = 0;i<vv.size();i++){
		int ni = vv[i].rows();
		ret.segment(o,ni) = vv[i];
		o += ni;
	}
	return ret;
}

Eigen::MatrixXd
MathUtils::
toEigenMatrix(const std::vector<Eigen::VectorXd>& vv)
{
	Eigen::MatrixXd A(vv.size(), vv[0].rows());
	for(int i =0;i<vv.size();i++)
		A.row(i) = vv[i];
	return A;
}
Eigen::MatrixXd
MathUtils::
toEigenMatrix(const std::vector<Eigen::Vector3d>& vv)
{
	Eigen::MatrixXd A(vv.size(), 3);
	for(int i =0;i<vv.size();i++)
		A.row(i) = vv[i];
	return A;
}