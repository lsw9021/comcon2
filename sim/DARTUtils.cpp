#include "DARTUtils.h"
#include "tinyxml2.h"
#include "MathUtils.h"

using namespace dart::dynamics;

typedef tinyxml2::XMLElement TiXmlElement;
typedef tinyxml2::XMLDocument TiXmlDocument;


ShapePtr
DARTUtils::
makeSphereShape(double radius)
{
	return std::make_shared<SphereShape>(radius);
}
ShapePtr
DARTUtils::
makeBoxShape(const Eigen::Vector3d& size)
{
	return std::make_shared<BoxShape>(size);
}
ShapePtr
DARTUtils::
makeCapsuleShape(double radius, double height)
{
	return std::shared_ptr<CapsuleShape>(new CapsuleShape(radius,height));
}

dart::dynamics::Inertia
DARTUtils::
makeInertia(const dart::dynamics::ShapePtr& shape,double mass)
{
	dart::dynamics::Inertia inertia;

	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));
	return inertia;
}

FreeJoint::Properties*
DARTUtils::
makeFreeJointProperties(const std::string& name,const Eigen::Isometry3d& parent_to_joint,const Eigen::Isometry3d& child_to_joint)
{
	FreeJoint::Properties* props = new FreeJoint::Properties();

	props->mName = name;
	props->mT_ParentBodyToJoint = parent_to_joint;
	props->mT_ChildBodyToJoint = child_to_joint;
	props->mIsPositionLimitEnforced = false;
	props->mVelocityLowerLimits = Eigen::Vector6d::Constant(-100.0);
	props->mVelocityUpperLimits = Eigen::Vector6d::Constant(100.0);
	props->mDampingCoefficients = Eigen::Vector6d::Constant(0.1);

	return props;
}
PlanarJoint::Properties*
DARTUtils::
makePlanarJointProperties(const std::string& name,const Eigen::Isometry3d& parent_to_joint,const Eigen::Isometry3d& child_to_joint)
{
	PlanarJoint::Properties* props = new PlanarJoint::Properties();


	props->mName = name;
	props->mT_ParentBodyToJoint = parent_to_joint;
	props->mT_ChildBodyToJoint = child_to_joint;
	props->mIsPositionLimitEnforced = false;
	props->mVelocityLowerLimits = Eigen::Vector3d::Constant(-100.0);
	props->mVelocityUpperLimits = Eigen::Vector3d::Constant(100.0);
	props->mDampingCoefficients = Eigen::Vector3d::Constant(0.4);

	return props;
}
BallJoint::Properties*
DARTUtils::
makeBallJointProperties(const std::string& name,const Eigen::Isometry3d& parent_to_joint,const Eigen::Isometry3d& child_to_joint,const Eigen::Vector3d& lower,const Eigen::Vector3d& upper)
{
	BallJoint::Properties* props = new BallJoint::Properties();

	props->mName = name;
	props->mT_ParentBodyToJoint = parent_to_joint;
	props->mT_ChildBodyToJoint = child_to_joint;
	props->mIsPositionLimitEnforced = true;
	props->mPositionLowerLimits = lower;
	props->mPositionUpperLimits = upper;
	props->mVelocityLowerLimits = Eigen::Vector3d::Constant(-100.0);
	props->mVelocityUpperLimits = Eigen::Vector3d::Constant(100.0);
	props->mForceLowerLimits = Eigen::Vector3d::Constant(-1000.0); 
	props->mForceUpperLimits = Eigen::Vector3d::Constant(1000.0);
	props->mDampingCoefficients = Eigen::Vector3d::Constant(0.4);

	return props;
}
RevoluteJoint::Properties*
DARTUtils::
makeRevoluteJointProperties(const std::string& name,const Eigen::Vector3d& axis,const Eigen::Isometry3d& parent_to_joint,const Eigen::Isometry3d& child_to_joint,const Eigen::Vector1d& lower,const Eigen::Vector1d& upper)
{
	RevoluteJoint::Properties* props = new RevoluteJoint::Properties();

	props->mName = name;
	props->mT_ParentBodyToJoint = parent_to_joint;
	props->mT_ChildBodyToJoint = child_to_joint;
	props->mIsPositionLimitEnforced = true;
	props->mPositionLowerLimits = lower;
	props->mPositionUpperLimits = upper;
	props->mAxis = axis;
	props->mVelocityLowerLimits = Eigen::Vector1d::Constant(-100.0);
	props->mVelocityUpperLimits = Eigen::Vector1d::Constant(100.0);
	props->mForceLowerLimits = Eigen::Vector1d::Constant(-1000.0); 
	props->mForceUpperLimits = Eigen::Vector1d::Constant(1000.0);
	props->mDampingCoefficients = Eigen::Vector1d::Constant(0.4);

	return props;
}
WeldJoint::Properties*
DARTUtils::
makeWeldJointProperties(const std::string& name,const Eigen::Isometry3d& parent_to_joint,const Eigen::Isometry3d& child_to_joint)
{
	WeldJoint::Properties* props = new WeldJoint::Properties();

	props->mName = name;
	props->mT_ParentBodyToJoint = parent_to_joint;
	props->mT_ChildBodyToJoint = child_to_joint;

	return props;
}
BodyNode*
DARTUtils::
makeBodyNode(const SkeletonPtr& skeleton,BodyNode* parent,Joint::Properties* joint_properties,const std::string& joint_type,dart::dynamics::Inertia inertia)
{
	BodyNode* bn;

	if(joint_type == "Free")
	{
		FreeJoint::Properties* prop = dynamic_cast<FreeJoint::Properties*>(joint_properties);
		bn = skeleton->createJointAndBodyNodePair<FreeJoint>(
			parent,(*prop),BodyNode::AspectProperties(joint_properties->mName)).second;
	}
	else if(joint_type == "Planar")
	{
		PlanarJoint::Properties* prop = dynamic_cast<PlanarJoint::Properties*>(joint_properties);
		bn = skeleton->createJointAndBodyNodePair<PlanarJoint>(
			parent,(*prop),BodyNode::AspectProperties(joint_properties->mName)).second;
	}
	else if(joint_type == "Ball")
	{
		BallJoint::Properties* prop = dynamic_cast<BallJoint::Properties*>(joint_properties);
		bn = skeleton->createJointAndBodyNodePair<BallJoint>(
			parent,(*prop),BodyNode::AspectProperties(joint_properties->mName)).second;
	}
	else if(joint_type == "Revolute")
	{
		RevoluteJoint::Properties* prop = dynamic_cast<RevoluteJoint::Properties*>(joint_properties);
		bn = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
			parent,(*prop),BodyNode::AspectProperties(joint_properties->mName)).second;
	}
	else if(joint_type == "Weld")
	{
		WeldJoint::Properties* prop = dynamic_cast<WeldJoint::Properties*>(joint_properties);
		bn = skeleton->createJointAndBodyNodePair<WeldJoint>(
			parent,(*prop),BodyNode::AspectProperties(joint_properties->mName)).second;
	}

	bn->setInertia(inertia);
	return bn;
}

std::vector<double> splitToDouble(const std::string& input, int num)
{
    std::vector<double> result;
    std::string::size_type sz = 0, nsz = 0;
    for(int i = 0; i < num; i++){
        result.emplace_back(std::stof(input.substr(sz), &nsz));
        sz += nsz;
    }
    return result;
}
Eigen::Vector1d stringToVector1d(const std::string& input){
	std::vector<double> v = splitToDouble(input, 1);
	Eigen::Vector1d res;
	res << v[0];

	return res;
}
Eigen::Vector3d stringToVector3d(const std::string& input){
	std::vector<double> v = splitToDouble(input, 3);
	Eigen::Vector3d res;
	res << v[0], v[1], v[2];

	return res;
}
Eigen::Vector4d stringToVector4d(const std::string& input)
{
	std::vector<double> v = splitToDouble(input, 4);
	Eigen::Vector4d res;
	res << v[0], v[1], v[2],v[3];

	return res;	
}
Eigen::VectorXd stringToVectorXd(const std::string& input, int n){
	std::vector<double> v = splitToDouble(input, n);
	Eigen::VectorXd res(n);
	for(int i = 0; i < n; i++){
		res[i] = v[i];
	}

	return res;
}
Eigen::Matrix3d stringToMatrix3d(const std::string& input){
	std::vector<double> v = splitToDouble(input, 9);
	Eigen::Matrix3d res;
	res << v[0], v[1], v[2],
			v[3], v[4], v[5],
			v[6], v[7], v[8];

	return res;
}
SkeletonPtr
DARTUtils::
buildFromFile(const std::string& path, std::map<std::string, std::string>& bvh_map,
								std::map<std::string, double>& kp_map,
								std::map<std::string, double>& mf_map)
{
	TiXmlDocument doc;
	if(doc.LoadFile(path.c_str())){
		std::cout << "Can't open file : " << path << std::endl;
		return nullptr;
	}

	TiXmlElement *skeleton_elem = doc.FirstChildElement("Skeleton");
	std::string skel_name = skeleton_elem->Attribute("name");
	SkeletonPtr skel = Skeleton::create(skel_name);

	for(TiXmlElement* node = skeleton_elem->FirstChildElement("Node");node != nullptr;node = node->NextSiblingElement("Node"))
	{
		std::string name = node->Attribute("name");
		std::string parent_str = node->Attribute("parent");
		BodyNode* parent = nullptr;
		if(parent_str != "None")
			parent = skel->getBodyNode(parent_str);
		
		ShapePtr shape;
		Eigen::Isometry3d T_body = Eigen::Isometry3d::Identity();
		TiXmlElement* body = node->FirstChildElement("Body");
		

		std::string type = body->Attribute("type");
		std::string obj_file = "None";
		if(body->Attribute("obj"))
			obj_file = body->Attribute("obj");
		double mass = std::stod(body->Attribute("mass"));
		if(type == "Box")
		{
			Eigen::Vector3d size = stringToVector3d(body->Attribute("size"));
			shape = makeBoxShape(size);
		}
		else if(type=="Sphere")
		{
			double radius = std::stod(body->Attribute("radius"));
			shape = makeSphereShape(radius);
		}
		else if(type=="Capsule")
		{
			double radius = std::stod(body->Attribute("radius"));
			double height = std::stod(body->Attribute("height"));
			shape = makeCapsuleShape(radius,height);
		}
		bool contact = false;
		if(body->Attribute("contact")!=nullptr){
			std::string c = body->Attribute("contact");
			if(c == "On")
				contact = true;
		}
		dart::dynamics::Inertia inertia = makeInertia(shape,mass);
		T_body.linear() = stringToMatrix3d(body->FirstChildElement("Transformation")->Attribute("linear"));
		T_body.translation() = stringToVector3d(body->FirstChildElement("Transformation")->Attribute("translation"));
		T_body = MathUtils::orthonormalize(T_body);		
		// std::cout<<"translation : "<<T_body.translation().transpose()<<std::endl;
		TiXmlElement* joint = node->FirstChildElement("Joint");
		
		type = joint->Attribute("type");

		std::string kpstring = joint->Attribute("kp");
		double kp = stringToVector1d(kpstring)[0];
		kp_map.insert(std::make_pair(name, kp));
		
		std::string maxfstring = joint->Attribute("maxf");
		double maxf = stringToVector1d(maxfstring)[0];
		mf_map.insert(std::make_pair(name, maxf));

		std::string bvhstring = "None";
		if(joint->Attribute("bvh") != nullptr)
			bvhstring = joint->Attribute("bvh");
		bvh_map.insert(std::make_pair(name, bvhstring));

		Joint::Properties* props;
		Eigen::Isometry3d T_joint = Eigen::Isometry3d::Identity();
		T_joint.linear() = stringToMatrix3d(joint->FirstChildElement("Transformation")->Attribute("linear"));
		T_joint.translation() = stringToVector3d(joint->FirstChildElement("Transformation")->Attribute("translation"));

		T_joint = MathUtils::orthonormalize(T_joint);	
		Eigen::Isometry3d parent_to_joint;
		if(parent==nullptr)
			parent_to_joint = T_joint;
		else
			parent_to_joint = parent->getTransform().inverse()*T_joint;
		Eigen::Isometry3d child_to_joint = T_body.inverse()*T_joint;
		int dof = 0;
		if(type == "Free")
		{
			props = makeFreeJointProperties(name,parent_to_joint,child_to_joint);
			dof = 6;
		}
		else if(type == "Planar")
		{
			props = makePlanarJointProperties(name,parent_to_joint,child_to_joint);
			dof = 2;
		}
		else if(type == "Weld")
		{
			props = makeWeldJointProperties(name,parent_to_joint,child_to_joint);
			dof = 0;
		}
		else if(type == "Ball" )
		{
			Eigen::Vector3d lower = stringToVector3d(joint->Attribute("lower"));
			Eigen::Vector3d upper = stringToVector3d(joint->Attribute("upper"));
			props = makeBallJointProperties(name,parent_to_joint,child_to_joint,lower,upper);
			dof = 3;
		}
		else if(type == "Revolute")
		{
				
			Eigen::Vector1d lower = stringToVector1d(joint->Attribute("lower"));
			Eigen::Vector1d upper = stringToVector1d(joint->Attribute("upper"));
			Eigen::Vector3d axis = stringToVector3d(joint->Attribute("axis"));
			props = makeRevoluteJointProperties(name,axis,parent_to_joint,child_to_joint,lower,upper);
			dof = 1;
		}

		auto bn = makeBodyNode(skel,parent,props,type,inertia);
		bn->setRestitutionCoeff(0.2);
		if(bn->getName().find("Spine") != std::string::npos)
			bn->setRestitutionCoeff(0.7);
		
		if(contact)
			bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
		else
			bn->createShapeNodeWith<VisualAspect, DynamicsAspect>(shape);
	}

	return skel;
}

dart::dynamics::SkeletonPtr
DARTUtils::
createGround(double y)
{
	SkeletonPtr skel = Skeleton::create("ground");
	ShapePtr shape = makeBoxShape(Eigen::Vector3d(100.0,1.0,100.0));
	double mass = 1000.0;	
	dart::dynamics::Inertia inertia = makeInertia(shape,mass);
	
	Eigen::Isometry3d T_body = Eigen::Isometry3d::Identity();
	T_body.translation()[1] = y-0.5;

	Joint::Properties* props = makeWeldJointProperties("root",T_body,Eigen::Isometry3d::Identity());
	auto bn = makeBodyNode(skel,nullptr,props,"Weld",inertia);
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);

	return skel;
}
dart::dynamics::SkeletonPtr
DARTUtils::
createDoor(const Eigen::Vector3d& c0, double width)
{
	double depth = 0.05;
	double height = 2.0;
	double base_ratio = 2.0;
	Eigen::Vector3d size_base = Eigen::Vector3d(width*base_ratio,height,depth);
	Eigen::Vector3d size = Eigen::Vector3d(width,height,depth);
	SkeletonPtr skel = Skeleton::create("door");
	ShapePtr shape_base = makeBoxShape(size_base);
	ShapePtr shape = makeBoxShape(size);

	double mass_base = 1000.0;
	double mass = 10.0;
	dart::dynamics::Inertia inertia_base = makeInertia(shape_base,mass_base);
	dart::dynamics::Inertia inertia = makeInertia(shape,mass);

	Eigen::Isometry3d T_pj = Eigen::Isometry3d::Identity();
	T_pj.translation() = c0;
	T_pj.translation()[0] += -0.5*size_base[0];
	T_pj.translation()[1] += 0.5*size_base[1];
	T_pj.translation()[2] += 0.5*size_base[2];

	Joint::Properties* props = makeWeldJointProperties("base",T_pj,Eigen::Isometry3d::Identity());
	auto bn = makeBodyNode(skel,nullptr,props,"Weld",inertia_base);

	bn->createShapeNodeWith<VisualAspect,DynamicsAspect>(shape_base);

	T_pj.setIdentity();
	T_pj.translation() = 0.5*size_base;
	Eigen::Isometry3d T_cj = Eigen::Isometry3d::Identity();
	T_cj.translation()[0] = -0.5*size[0];
	T_cj.translation()[1] = 0.5*size[1];
	T_cj.translation()[2] = 0.5*size[2];

	props = makeRevoluteJointProperties("door",Eigen::Vector3d::UnitY(),T_pj,T_cj,Eigen::Vector1d(-2.0),Eigen::Vector1d(2.0));
	bn = makeBodyNode(skel,bn,props,"Revolute",inertia);

	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);
	skel->getJoint(1)->setSpringStiffness(0, 1.0);
	skel->getJoint(1)->setDampingCoefficient(0, 1.0);
	return skel;
}
dart::dynamics::SkeletonPtr
DARTUtils::
createBox(double density, const Eigen::Vector3d& size, const std::string& type)
{
	SkeletonPtr skel = Skeleton::create("Box");
	ShapePtr shape = makeBoxShape(size);
	double mass = density*size[0]*size[1]*size[2];
	dart::dynamics::Inertia inertia = makeInertia(shape,mass);
	Joint::Properties* props;
	if(type == "Free")
		props = makeFreeJointProperties("root",Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());
	else if(type == "Weld")
		props = makeWeldJointProperties("root",Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());
	auto bn = makeBodyNode(skel,nullptr,props,type,inertia);
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);

	return skel;	
}

dart::dynamics::SkeletonPtr
DARTUtils::
createBall(double density, double r, const std::string& type)
{
	SkeletonPtr skel = Skeleton::create("Ball");
	ShapePtr shape = makeSphereShape(r);
	double mass = density*r*r*r;
	dart::dynamics::Inertia inertia = makeInertia(shape, mass);
	Joint::Properties* props;
	if(type == "Free")
		props = makeFreeJointProperties("root",Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());
	else if(type == "Weld")
		props = makeWeldJointProperties("root",Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());
	auto bn = makeBodyNode(skel,nullptr,props,type,inertia);
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);

	return skel;	
}

dart::dynamics::SkeletonPtr
DARTUtils::
createRod(double len, double width, double plate_radius)
{
	SkeletonPtr skel = Skeleton::create("Rod");
	Eigen::Vector3d rod_size(width,len,width);
	ShapePtr shape = makeBoxShape(rod_size);
	double density = 300.0;
	double mass = density*rod_size[0]*rod_size[1]*rod_size[2];
	dart::dynamics::Inertia inertia = makeInertia(shape,mass);
	Joint::Properties* props;
	props = makeFreeJointProperties("root",Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());
	auto bn = makeBodyNode(skel,nullptr,props,"Free",inertia);
	bn->createShapeNodeWith<VisualAspect,DynamicsAspect>(shape);

	Eigen::Vector3d plate_size(plate_radius, 0.01, plate_radius);
	shape = makeBoxShape(plate_size);
	mass = density*plate_size[0]*plate_size[1]*plate_size[2];
	inertia = makeInertia(shape,mass);
	Eigen::Isometry3d T_pj,T_cj;
	T_pj.setIdentity();
	T_cj.setIdentity();
	T_pj.translation()[1] = 0.5*len;

	props = makeWeldJointProperties("plate",T_pj,T_cj);
	bn = makeBodyNode(skel,bn,props,"Weld",inertia);
	bn->createShapeNodeWith<VisualAspect,CollisionAspect,DynamicsAspect>(shape);

	return skel;	
}
Eigen::MatrixXd
DARTUtils::
computeDiffPositions(const Eigen::MatrixXd& p1, const Eigen::MatrixXd& p2)
{
	assert(p1.rows()==3 && p2.rows()==3 && p1.cols() == p2.cols());
	int n = p1.cols();

	Eigen::MatrixXd ret(3,n); 
	for(int i=0;i<n;i++)
	{
		Eigen::Matrix3d R1 = dart::math::expMapRot(p1.col(i));
		Eigen::Matrix3d R2 = dart::math::expMapRot(p2.col(i));
		
		ret.col(i) = dart::math::logMap(R1.transpose()*R2);
	}
	return ret;
}

std::pair<dart::dynamics::BodyNode*, Eigen::Vector3d>
DARTUtils::
getPointClosestBodyNode(dart::dynamics::SkeletonPtr skel, const Eigen::Vector3d& point)
{
	double min_distance = 1e6;
	dart::dynamics::BodyNode* ret = nullptr;
	Eigen::Vector3d point_local_ret;

	for(int i=0;i<skel->getNumBodyNodes();i++)
	{
		auto bn = skel->getBodyNode(i);
		auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();
		auto T = shapeNodes.back()->getTransform();
		auto shape = shapeNodes.back()->getShape().get();

		Eigen::Vector3d point_local = T.inverse()*point;
		double distance;
		if(shape->is<SphereShape>())
		{
			const auto* sphere = dynamic_cast<const SphereShape*>(shape);
			double d = point_local.norm();
			double r = sphere->getRadius();
			if(d<r)
				distance = 0.0;
			else
				distance = d-r;
		}
		else if(shape->is<BoxShape>()){
			const auto* box = dynamic_cast<const BoxShape*>(shape);
			distance = (Eigen::Vector3d::Zero().cwiseMax(point_local.cwiseAbs() - 0.5*box->getSize())).norm();
		}
		if(distance<min_distance)
		{
			ret = bn;
			min_distance = distance;
			point_local_ret = point_local;
		}
		if(min_distance<1e-6)
			return std::make_pair(ret, point_local_ret);
	}
	return std::make_pair(ret, point_local_ret);
}