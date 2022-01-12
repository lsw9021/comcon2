#ifndef __DART_UTILS_H__
#define __DART_UTILS_H__
#include "dart/dart.hpp"
namespace Eigen {
using Vector1d = Matrix<double, 1, 1>;
using Matrix1d = Matrix<double, 1, 1>;
}

std::vector<double> split_to_double(const std::string& input, int num);
Eigen::Vector1d string_to_vector1d(const std::string& input);
Eigen::Vector3d string_to_vector3d(const std::string& input);
Eigen::Vector4d string_to_vector4d(const std::string& input);
Eigen::VectorXd string_to_vectorXd(const std::string& input, int n);
Eigen::Matrix3d string_to_matrix3d(const std::string& input);

class DARTUtils
{
public:
	static dart::dynamics::ShapePtr makeSphereShape(double radius);
	static dart::dynamics::ShapePtr makeBoxShape(const Eigen::Vector3d& size);
	static dart::dynamics::ShapePtr makeCapsuleShape(double radius, double height);
	static dart::dynamics::Inertia makeInertia(const dart::dynamics::ShapePtr& shape,double mass);

	static dart::dynamics::FreeJoint::Properties*	makeFreeJointProperties(const std::string& name,const Eigen::Isometry3d& parent_to_joint = Eigen::Isometry3d::Identity(),const Eigen::Isometry3d& child_to_joint = Eigen::Isometry3d::Identity());
	static dart::dynamics::PlanarJoint::Properties* makePlanarJointProperties(const std::string& name,const Eigen::Isometry3d& parent_to_joint = Eigen::Isometry3d::Identity(),const Eigen::Isometry3d& child_to_joint = Eigen::Isometry3d::Identity());
	static dart::dynamics::BallJoint::Properties* makeBallJointProperties(const std::string& name,const Eigen::Isometry3d& parent_to_joint = Eigen::Isometry3d::Identity(),const Eigen::Isometry3d& child_to_joint = Eigen::Isometry3d::Identity(),const Eigen::Vector3d& lower = Eigen::Vector3d::Constant(-2.0),const Eigen::Vector3d& upper = Eigen::Vector3d::Constant(2.0));
	static dart::dynamics::RevoluteJoint::Properties* makeRevoluteJointProperties(const std::string& name,const Eigen::Vector3d& axis,const Eigen::Isometry3d& parent_to_joint = Eigen::Isometry3d::Identity(),const Eigen::Isometry3d& child_to_joint = Eigen::Isometry3d::Identity(),const Eigen::Vector1d& lower = Eigen::Vector1d::Constant(-2.0),const Eigen::Vector1d& upper = Eigen::Vector1d::Constant(2.0));
	static dart::dynamics::WeldJoint::Properties* makeWeldJointProperties(const std::string& name,const Eigen::Isometry3d& parent_to_joint = Eigen::Isometry3d::Identity(),const Eigen::Isometry3d& child_to_joint = Eigen::Isometry3d::Identity());

	static dart::dynamics::BodyNode* makeBodyNode(const dart::dynamics::SkeletonPtr& skeleton,dart::dynamics::BodyNode* parent,dart::dynamics::Joint::Properties* joint_properties,const std::string& joint_type,dart::dynamics::Inertia inertia);
	static dart::dynamics::SkeletonPtr buildFromFile(const std::string& path, 
								std::map<std::string, std::string>& bvh_map,
								std::map<std::string, double>& kp_map,
								std::map<std::string, double>& mf_map);


	static dart::dynamics::SkeletonPtr createGround(double y);
	static dart::dynamics::SkeletonPtr createDoor(const Eigen::Isometry3d& T_ref,
													double w0,double w1,double mass);
	static dart::dynamics::SkeletonPtr createDoor(const Eigen::Vector3d& c0, double width);

	static dart::dynamics::SkeletonPtr createBox(double density, const Eigen::Vector3d& size, const std::string& type="Free");
	static dart::dynamics::SkeletonPtr createBall(double density, double r, const std::string& type="Free");

	static dart::dynamics::SkeletonPtr createRod(double len, double width, double plate_radius);
	static Eigen::MatrixXd computeDiffPositions(const Eigen::MatrixXd& p1, const Eigen::MatrixXd& p2);
	static std::pair<dart::dynamics::BodyNode*, Eigen::Vector3d> getPointClosestBodyNode(dart::dynamics::SkeletonPtr skel, const Eigen::Vector3d& point);
};


#endif