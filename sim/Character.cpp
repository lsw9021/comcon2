#include "Character.h"
#include "MathUtils.h"
#include <Eigen/SVD>
using namespace dart;
using namespace dart::dynamics;
Character::
Character(SkeletonPtr& skel)
	:mSkeleton(skel)
{
	int n = mSkeleton->getNumDofs();
	mKp = Eigen::VectorXd::Zero(n);
	mKv = Eigen::VectorXd::Zero(n);
	mMaxForces = Eigen::VectorXd::Zero(n);

	mU = Eigen::VectorXd::Zero(n);
	mdU = Eigen::VectorXd::Zero(n);

	mdHat = Eigen::Vector3d::Constant(0.25);//ANKLE
	// mdHat = Eigen::Vector3d::Constant(0.5);//HIP
	// mdHat = Eigen::Vector3d::Constant(1.0);//HIP
	mRootdHat = Eigen::Vector3d::Ones();
	//#0 balance
	mDefaultVelocity = Eigen::Vector3d(0,0,0.0);

	//#1 walking
	// mDefaultVelocity = Eigen::Vector3d(0,0,1.4);

	Eigen::VectorXd ankle_boundary(16);
	ankle_boundary<<8.39154503,  8.82749394, 11.4503686 , 16.22168304, 18.44228347, 21.12059897,
 	21.91645705, 22.68074787 ,21.40073041 ,22.68074787, 21.91645705 ,21.12059897,
	18.44228347 ,16.22168304 ,11.4503686 ,  8.82749394;
	
	Eigen::VectorXd hip_boundary(16);
	hip_boundary<<32.74434118, 31.89924462, 45.16855115, 54.18451161, 63.49003247, 78.95132497,
				 85.65653504, 74.13815805, 69.79092297, 74.13815805, 85.65653504, 78.95132497,
				 63.49003247, 54.18451161, 45.16855115, 31.89924462;
	hip_boundary*=0.5;
	mBoundaries.emplace_back(ankle_boundary);
	mBoundaries.emplace_back(hip_boundary);		 
	this->clearCummulatedForces();
	mLight = 0;
}
void
Character::
reset(const Eigen::VectorXd& p, const Eigen::VectorXd& v)
{
	mSkeleton->setPositions(p);
	mSkeleton->setVelocities(v);
	mSkeleton->clearConstraintImpulses();
	mSkeleton->clearInternalForces();
	mSkeleton->clearExternalForces();

	int n = mSkeleton->getNumDofs();
	mU = Eigen::VectorXd::Zero(n);
	Eigen::Isometry3d T_ref = this->getReferenceTransform();
	// mU.head<3>() = mSkeleton->getBodyNode(0)->getCOM() + T_ref.linear()*mDefaultVelocity;
	// mURootBar = mU.head<3>();
	mLeftFootPosition = mSkeleton->getBodyNode("LeftFoot")->getCOM();
	mRightFootPosition = mSkeleton->getBodyNode("RightFoot")->getCOM();
	mdU = Eigen::VectorXd::Zero(n);
	mUroot = Eigen::Vector3d::Zero();
	mdUroot = Eigen::Vector3d::Zero();

	mLight = 0;
	mCurrentBalanceType = 2;
	mAppliedForce = false;
	mTargetVelocity.setZero();
	this->clearCummulatedForces();
}
void
Character::
addEndEffector(const std::string& bn_name)
{
	mEndEffectors.emplace_back(mSkeleton->getBodyNode(bn_name));
}
void
Character::
addKinematicMap(int sid, int kid)
{
	mKinematicMap.insert(std::make_pair(sid, kid));
	mSimMap.insert(std::make_pair(kid, sid));
}
void
Character::
setPDGain(const std::string& sim_body_name, double kp)
{
	auto joint = mSkeleton->getBodyNode(sim_body_name)->getParentJoint();

	int n = joint->getNumDofs();
	if(n==0)
		return;
	int o = joint->getIndexInSkeleton(0);

	mKp.segment(o,n) = Eigen::VectorXd::Constant(n, kp);
	mKv.segment(o,n) = Eigen::VectorXd::Constant(n, 2*std::sqrt(kp));
	if(n ==6)
	{
		kp = 30.0;
		mKp.segment(o,3) = Eigen::VectorXd::Constant(3, kp);
		mKv.segment(o,3) = Eigen::VectorXd::Constant(3, 2*std::sqrt(kp));
	}
}
void
Character::
setMaxForces(const std::string& sim_body_name, double mf)
{
	auto joint = mSkeleton->getBodyNode(sim_body_name)->getParentJoint();
	int n = joint->getNumDofs();
	if(n==0)
		return;
	int o = joint->getIndexInSkeleton(0);
	mMaxForces.segment(o, n) = Eigen::VectorXd::Constant(n, mf);
}
Eigen::Isometry3d
Character::
getReferenceTransform()
{
	Eigen::VectorXd q_root = mSkeleton->getPositions().head<6>();
	Eigen::Isometry3d T = FreeJoint::convertToTransform(q_root);
	Eigen::Matrix3d R = T.linear();
	Eigen::Vector3d p = T.translation();
	Eigen::Vector3d z = R.col(2);
	Eigen::Vector3d y = Eigen::Vector3d::UnitY();
	z -= MathUtils::projectOnVector(z, y);
	p -= MathUtils::projectOnVector(p, y);

	z.normalize();
	Eigen::Vector3d x = y.cross(z);

	R.col(0) = x;
	R.col(1) = y;
	R.col(2) = z;

	T.linear() = R;
	T.translation() = p;

	return T;
}

double
Character::
getReferenceOrientation()
{
	Eigen::Isometry3d T_ref = this->getReferenceTransform();

	Eigen::AngleAxisd aa(T_ref.linear());
	Eigen::Vector3d vec = aa.angle()*aa.axis();
	return vec[1];
}
void
Character::
computeSimPose(const Eigen::Vector3d& position,
				const Eigen::MatrixXd& rotation,
				Eigen::VectorXd& p)
{
	p = Eigen::VectorXd::Zero(mSkeleton->getNumDofs());
	p.segment<3>(3) = position;
	for(const auto& sim_kin : mKinematicMap)
	{
		int sid = sim_kin.first;
		int kid = sim_kin.second;
		auto joint = mSkeleton->getJoint(sid);
		if(joint->getNumDofs()==0)
			continue;
		int idx_in_skel = mSkeleton->getJoint(sid)->getIndexInSkeleton(0);
		Eigen::Matrix3d R = rotation.block<3,3>(0,kid*3);
		p.segment<3>(idx_in_skel) = BallJoint::convertToPositions(R);
	}
}
void
Character::
computeSimPoseAndVel(const Eigen::Vector3d& position,
					const Eigen::MatrixXd& rotation,
					const Eigen::Vector3d& linear_velocity,
					const Eigen::MatrixXd& angular_velocity,
					Eigen::VectorXd& p,
					Eigen::VectorXd& v)
{
	this->computeSimPose(position,rotation,p);

	v = Eigen::VectorXd::Zero(mSkeleton->getNumDofs());
	for(const auto& sim_kin : mKinematicMap)
	{
		int sid = sim_kin.first;
		int kid = sim_kin.second;
		auto joint = mSkeleton->getJoint(sid);
		if(joint->getNumDofs()==0)
			continue;
		
		int idx_in_skel = mSkeleton->getJoint(sid)->getIndexInSkeleton(0);
		
		v.segment<3>(idx_in_skel) = angular_velocity.col(kid);
		if(sid == 0){
			Eigen::Matrix3d R = rotation.block<3,3>(0,0);
			v.segment<3>(idx_in_skel+3) = R.transpose()*linear_velocity;
		}
	}
}

void
Character::
actuate(const Eigen::VectorXd& action)
{
	int n = mSkeleton->getNumDofs();
	Eigen::VectorXd target_position = Eigen::VectorXd::Zero(n);
	target_position.tail(n-6) = action;
	Eigen::VectorXd pu = Eigen::VectorXd::Zero(2*n);
	target_position = this->computeDisplacedPositions(target_position, mU);

	Eigen::VectorXd q = mSkeleton->getPositions();
	Eigen::VectorXd dq = mSkeleton->getVelocities();
	double dt = mSkeleton->getTimeStep();

	Eigen::MatrixXd M_inv = (mSkeleton->getMassMatrix() + Eigen::MatrixXd(dt*mKv.asDiagonal())).inverse();

	Eigen::VectorXd qdqdt = q + dq*dt;

	Eigen::VectorXd p_diff = -mKp.cwiseProduct(mSkeleton->getPositionDifferences(qdqdt,target_position));
	Eigen::VectorXd v_diff = -mKv.cwiseProduct(dq);
	Eigen::VectorXd ddq = M_inv*(-mSkeleton->getCoriolisAndGravityForces()+p_diff+v_diff+mSkeleton->getConstraintForces());

	Eigen::VectorXd tau = p_diff + v_diff - dt*mKv.cwiseProduct(ddq);

	Eigen::VectorXd min_forces = -mMaxForces;
	tau = dart::math::clip<Eigen::VectorXd,Eigen::VectorXd>(tau,min_forces,mMaxForces);

	mCummulatedForces += tau;
	mSkeleton->setForces(tau);
}
Eigen::Matrix3d
Character::
computeStiffnessMatrix(dart::dynamics::BodyNode* bn,const Eigen::Vector3d& offset)
{
	int n = mSkeleton->getNumDofs()-6;
	Eigen::MatrixXd kp_inv = mKp.tail(n).cwiseInverse().asDiagonal();
	Eigen::MatrixXd J = mSkeleton->getLinearJacobian(bn, offset);
	J = J.rightCols(n);
	Eigen::Matrix3d K = (J*kp_inv*J.transpose()).inverse();
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(K, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Vector3d d = svd.singularValues();
	d.normalize();
	d = d.cwiseInverse();
	Eigen::Matrix3d matU = svd.matrixU();
	Eigen::Matrix3d matV = svd.matrixV();
	// matU.col(0) *=d[0];
	// matU.col(1) *=d[1];
	// matU.col(2) *=d[2];
	// std::cout<<matU.col(0).transpose()<<" "<<d[0]<<std::endl;
	// std::cout<<matU.col(1).transpose()<<" "<<d[1]<<std::endl;
	// std::cout<<matU.col(2).transpose()<<" "<<d[2]<<std::endl;
	// std::cout<<matU(0,0)<<","<<matU(1,0)<<","<<matU(2,0)<<","<<"0.0"<<","<<
	// 			matU(0,1)<<","<<matU(1,1)<<","<<matU(2,1)<<","<<"0.0"<<","<<
	// 			matU(0,2)<<","<<matU(1,2)<<","<<matU(2,2)<<","<<"0.0"<<",0.0,0.0,0.0,1.0"<<std::endl;

	// std::cout<<1.0/d[0]<<","<<1.0/d[1]<<","<<1.0/d[2]<<std::endl;
	
	d[0] *= mdHat[0];
	d[1] *= mdHat[1];
	d[2] *= mdHat[2];

	Eigen::Matrix3d Km = matU*(d.asDiagonal())*(matV.transpose());

	return K - Km;
}
void
Character::
addExternalForce(dart::dynamics::BodyNode* bn,
				const Eigen::Vector3d& offset,
				const Eigen::Vector3d& force)
{
	double h = 1.0/450.0;
	mAppliedForce = true;
	mOffset = offset;
	mForce = force;
	mBodyNodeName = bn->getName();
	mForce[1] = 0.0;
	mCurrentBalanceType = this->getBalanceType(mForce);
	// mLight = 0;
	// if(0)
	if(mCurrentBalanceType == 0)
	{
		mdHat = Eigen::Vector3d::Constant(0.25);//ANKLE
	// mdHat = Eigen::Vector3d::Constant(0.5);//HIP
	// mdHat = Eigen::Vector3d::Constant(1.0);//HIP
		// this->pushState();
		// this->setPositions(this->getPositions());
		int n = mSkeleton->getNumDofs();
		// Eigen::MatrixXd kp = mKp.asDiagonal();
		// Eigen::MatrixXd kp_inv = mKp.cwiseInverse().asDiagonal();

		Eigen::MatrixXd J = mSkeleton->getLinearJacobian(bn, offset);
		Eigen::MatrixXd J_int = 0.5*(mSkeleton->getLinearJacobian(bn, offset,mSkeleton->getBodyNode("LeftFoot"))
								 + mSkeleton->getLinearJacobian(bn, offset,mSkeleton->getBodyNode("RightFoot")));
		for(int i=6;i<n;i++)
		{
			if(J.col(i).norm()<1e-6)
				J.col(i) = -J_int.col(i);
		}
		// this->popState();

		mdU = h*J.transpose()*mdHat.cwiseProduct(force);
	}
	else if(mCurrentBalanceType == 1)
	{
		mdHat = Eigen::Vector3d::Constant(1.0);
		int n = mSkeleton->getNumDofs();
		// Eigen::MatrixXd kp = mKp.asDiagonal();
		// Eigen::MatrixXd kp_inv = mKp.cwiseInverse().asDiagonal();

		Eigen::MatrixXd J0 = mSkeleton->getLinearJacobian(bn, offset);
		Eigen::MatrixXd J0_int = 0.5*(mSkeleton->getLinearJacobian(bn, offset,mSkeleton->getBodyNode("LeftFoot"))
								 + mSkeleton->getLinearJacobian(bn, offset,mSkeleton->getBodyNode("RightFoot")));
		Eigen::MatrixXd J1 = J0;

		J0_int -= J0;
		J0_int.leftCols(6).setZero();
		J1.leftCols(6).setZero();

		Eigen::MatrixXd J = -0.3*(J0 - J0_int) + 10.0*J1;

		// this->popState();

		mdU = h*J.transpose()*mdHat.cwiseProduct(force);
	}
	else
	{
		mdHat = Eigen::Vector3d::Constant(0.7);
		// mdHat = Eigen::Vector3d::Constant(1.3);

		int n = mSkeleton->getNumDofs();

		Eigen::MatrixXd J = mSkeleton->getLinearJacobian(bn, offset);
		J.block<3,3>(0,0).setZero();
		mdU = h*J.transpose()*mdHat.cwiseProduct(force);


		double root_inv_mass = 5.0;
		// double root_inv_mass = 20.0;
		mdUroot = h*root_inv_mass*mRootdHat.cwiseProduct(force);
	}
	// Eigen::MatrixXd kp_inv = mKp.cwiseInverse().asDiagonal();
	// Eigen::Matrix3d K = (J*kp_inv*J.transpose()).inverse();
	// std::cout<<K.determinant()<<std::endl;

	// kp_inv = mKp.tail(n-6).cwiseInverse().asDiagonal();
	// K = (J_int*kp_inv*J_int.transpose()).inverse();
	// std::cout<<K.determinant()<<std::endl<<std::endl;


	// Eigen::MatrixXd Jt = J.transpose();

	// Eigen::JacobiSVD<Eigen::Matrix3d> svd(K, Eigen::ComputeFullU | Eigen::ComputeFullV);

	// Eigen::Vector3d d = svd.singularValues();
	// Eigen::Matrix3d matU = svd.matrixU();
	// Eigen::Matrix3d matV = svd.matrixV();

	// // d[0] = 0.05;
	// // d[1] = 0.05;
	// // d[2] = 0.05;
	// d[0] *= mdHat[0];
	// d[1] *= mdHat[1];
	// d[2] *= mdHat[2];

	// Eigen::Matrix3d Km = matU*(d.asDiagonal())*(matV.transpose());
	// Eigen::MatrixXd M_inv = mSkeleton->getInvMassMatrix().bottomRightCorner(n-6,n-6);
	// Eigen::Matrix3d M_work_inv = J_int*M_inv*J_int.transpose();
	// Eigen::MatrixXd M_inv = mSkeleton->getInvMassMatrix();
	// Eigen::Matrix3d M_work_inv = J*M_inv*J.transpose();

	// std::cout<<Km<<std::endl;
	// mdU = h*Jt*Km*M_work_inv*force;
	// mdU.segment<3>(3) *= 0.1;
	// mdU = h*Jt*force;

	// std::cout<<mSkeleton->getLinearJacobian(bn, offset,mSkeleton->getBodyNode("LeftFoot"))<<std::endl<<std::endl<<std::endl;
	
	// {
	// 	int n = mSkeleton->getNumDofs();
	// Eigen::MatrixXd kp = mKp.asDiagonal();
	// Eigen::MatrixXd kp_inv = mKp.cwiseInverse().asDiagonal();

	// Eigen::MatrixXd J0 = mSkeleton->getLinearJacobian(bn, offset);
	// Eigen::MatrixXd J_int = 0.5*(mSkeleton->getLinearJacobian(bn, offset,mSkeleton->getBodyNode("LeftFoot"))
	// 						 + mSkeleton->getLinearJacobian(bn, offset,mSkeleton->getBodyNode("RightFoot")));
	// Eigen::MatrixXd J = J0;
	// // J.leftCols(6) = -J0.leftCols(6);

	// // Eigen::MatrixXd J = 0.5*(mSkeleton->getLinearJacobian(bn, offset,mSkeleton->getBodyNode("LeftFoot"))
	// // 						 + mSkeleton->getLinearJacobian(bn, offset,mSkeleton->getBodyNode("RightFoot")));

	
	// // J = 0.5*(mSkeleton->getLinearJacobian(bn, offset,mSkeleton->getBodyNode("LeftFoot"))
	// // 						 + mSkeleton->getLinearJacobian(bn, offset,mSkeleton->getBodyNode("RightFoot")));
	// // J += 0.5*(mSkeleton->getLinearJacobian(mSkeleton->getBodyNode("LeftFoot")) + 
	// // 		mSkeleton->getLinearJacobian(mSkeleton->getBodyNode("RightFoot")));
	// // J += 0.5*(mSkeleton->getLinearJacobian(mSkeleton->getBodyNode("LeftFoot"))
	// // 						 + mSkeleton->getLinearJacobian(mSkeleton->getBodyNode("RightFoot")));

	// // std::cout<<J.leftCols(6)<<std::endl<<std::endl;
	// Eigen::MatrixXd Jt = J.transpose();
	// Eigen::Matrix3d K = (J*kp_inv*J.transpose()).inverse();

	// Eigen::JacobiSVD<Eigen::Matrix3d> svd(K, Eigen::ComputeFullU | Eigen::ComputeFullV);

	// Eigen::Vector3d d = svd.singularValues();
	// Eigen::Matrix3d matU = svd.matrixU();
	// Eigen::Matrix3d matV = svd.matrixV();
	// d[0] *= mdHat[0];
	// d[1] *= mdHat[1];
	// d[2] *= mdHat[2];

	// Eigen::Matrix3d Km = matU*(d.asDiagonal())*(matV.transpose());
	// Eigen::MatrixXd M_inv = mSkeleton->getInvMassMatrix();
	// Eigen::Matrix3d M_work_inv = J*M_inv*Jt;

	// mdU = h*Jt*Km*M_work_inv*force;
	// }
	int rknee = mSkeleton->getJoint("RightLeg")->getIndexInSkeleton(0);
	int lknee = mSkeleton->getJoint("LeftLeg")->getIndexInSkeleton(0);
	mdU[rknee+1] = 0.0;
	mdU[rknee+2] = 0.0;

	mdU[lknee+1] = 0.0;
	mdU[lknee+2] = 0.0;


	return;

	// int n = mSkeleton->getNumDofs()-6;
	// Eigen::MatrixXd kp = mKp.tail(n).asDiagonal();
	// Eigen::MatrixXd kp_inv = mKp.tail(n).cwiseInverse().asDiagonal();
	// Eigen::MatrixXd J = mSkeleton->getLinearJacobian(bn, offset);
	
	// J = J.rightCols(n);
	// Eigen::MatrixXd Jt = J.transpose();

	// Eigen::Matrix3d K = (J*kp_inv*J.transpose()).inverse();
	// Eigen::JacobiSVD<Eigen::Matrix3d> svd(K, Eigen::ComputeFullU | Eigen::ComputeFullV);

	// Eigen::Vector3d d = svd.singularValues();
	// Eigen::Matrix3d matU = svd.matrixU();
	// Eigen::Matrix3d matV = svd.matrixV();
	// d[0] *= mdHat[0];
	// d[1] *= mdHat[1];
	// d[2] *= mdHat[2];

	// Eigen::Matrix3d Km = matU*(d.asDiagonal())*(matV.transpose());
	// Eigen::MatrixXd M_inv = mSkeleton->getInvMassMatrix().bottomRightCorner(n,n);
	// Eigen::Matrix3d M_work_inv = J*M_inv*Jt;

	// mdU.tail(n) = h*Jt*Km*M_work_inv*force;

	// double root_inv_mass = 0.04;
	// Eigen::Vector3d root_d_hat = 60.0*mRootdHat;

	// mdU.head<3>() = h*root_inv_mass*root_d_hat.cwiseProduct(force);
	// mAppliedForce = true;
	// mOffset = offset;
	// mForce = force;
	// mBodyNodeName = bn->getName();
	// mCurrentBalanceType = this->getBalanceType(mForce);

	// {
	// 	int n = mSkeleton->getNumDofs()-6;
	// 	Eigen::MatrixXd kp = mKp.tail(n).asDiagonal();
	// 	Eigen::MatrixXd kp_inv = mKp.tail(n).cwiseInverse().asDiagonal();
	// 	Eigen::MatrixXd J = mSkeleton->getLinearJacobian(mSkeleton->getBodyNode("LeftFoot"));

	// 	J = J.rightCols(n);
	// 	Eigen::MatrixXd Jt = J.transpose();

	// 	Eigen::Matrix3d K = (J*kp_inv*J.transpose()).inverse();
	// 	Eigen::JacobiSVD<Eigen::Matrix3d> svd(K, Eigen::ComputeFullU | Eigen::ComputeFullV);

	// 	Eigen::Vector3d d = svd.singularValues();
	// 	Eigen::Matrix3d matU = svd.matrixU();
	// 	Eigen::Matrix3d matV = svd.matrixV();
	// 	d[0] *= mdHat[0];
	// 	d[1] *= mdHat[1];
	// 	d[2] *= mdHat[2];

	// 	Eigen::Matrix3d Km = matU*(d.asDiagonal())*(matV.transpose());
	// 	Eigen::MatrixXd M_inv = mSkeleton->getInvMassMatrix().bottomRightCorner(n,n);
	// 	Eigen::Matrix3d M_work_inv = J*M_inv*Jt;

	// 	mdU.tail(n) += 0.5*h*Jt*Km*M_work_inv*force;


	// }

	// {
	// 	int n = mSkeleton->getNumDofs()-6;
	// 	Eigen::MatrixXd kp = mKp.tail(n).asDiagonal();
	// 	Eigen::MatrixXd kp_inv = mKp.tail(n).cwiseInverse().asDiagonal();
	// 	Eigen::MatrixXd J = mSkeleton->getLinearJacobian(mSkeleton->getBodyNode("RightFoot"));

	// 	J = J.rightCols(n);
	// 	Eigen::MatrixXd Jt = J.transpose();

	// 	Eigen::Matrix3d K = (J*kp_inv*J.transpose()).inverse();
	// 	Eigen::JacobiSVD<Eigen::Matrix3d> svd(K, Eigen::ComputeFullU | Eigen::ComputeFullV);

	// 	Eigen::Vector3d d = svd.singularValues();
	// 	Eigen::Matrix3d matU = svd.matrixU();
	// 	Eigen::Matrix3d matV = svd.matrixV();
	// 	d[0] *= mdHat[0];
	// 	d[1] *= mdHat[1];
	// 	d[2] *= mdHat[2];

	// 	Eigen::Matrix3d Km = matU*(d.asDiagonal())*(matV.transpose());
	// 	Eigen::MatrixXd M_inv = mSkeleton->getInvMassMatrix().bottomRightCorner(n,n);
	// 	Eigen::Matrix3d M_work_inv = J*M_inv*Jt;

	// 	mdU.tail(n) += 0.5*h*Jt*Km*M_work_inv*force;
	// }
	// int rknee = mSkeleton->getJoint("RightLeg")->getIndexInSkeleton(0);
	// int lknee = mSkeleton->getJoint("LeftLeg")->getIndexInSkeleton(0);
	// mdU[rknee+1] = 0.0;
	// mdU[rknee+2] = 0.0;

	// mdU[lknee+1] = 0.0;
	// mdU[lknee+2] = 0.0;
	// Eigen::VectorXd du_lower_body = 0.4*(J_lf + J_rf).transpose()*(mdU.head<3>());
	// mdU.segment<3>(3) =	-0.6*du_lower_body.head<3>();

	// mdU.segment<3>(6) += 0.4*du_lower_body.head<3>();

	// mdU.tail(n) += du_lower_body.tail(n);

	
}
void
Character::
getExternalForce(std::string& bn_name, Eigen::Vector3d& offset, Eigen::Vector3d& force)
{
	if(mAppliedForce == false){
		bn_name = "";
		return;
	}
	bn_name = mBodyNodeName;
	force = mForce;
	offset = mOffset;
}
void
Character::
step()
{
	double h = 1.0/450.0;
	if(mAppliedForce == false)
	{
		mdU.setZero();
		mdUroot.setZero();

		{
			double m = 1.0;
			double k = 60.0;
			double d = 2.0*std::sqrt(k);
			// Eigen::Vector3d u_bar = mURootBar;
			// u_bar[0] = 0.0;
			Eigen::Vector3d u = mU.segment<3>(3);
			Eigen::Vector3d du = mdU.segment<3>(3);
			Eigen::Vector3d fn = -k*(u) - d*du;
			Eigen::Vector3d b1 = m*du + h*fn + h*k*u + h*d*du;
			Eigen::Vector3d b2 = u;
			double denom = 1.0/(m + h*d + h*h*k);

			mU.segment<3>(3) = denom*(h*b1+(m+h*d)*b2);
			mdU.segment<3>(3) = denom*(b1 + h*k*b2);
		}
		{
			// double m = 1.0;
			double m = 0.2;
			double k = 120.0;
			double d = 2.0*std::sqrt(k);

			Eigen::Vector3d u = mUroot;
			Eigen::Vector3d fn = -k*(u);
			if(mLight == 1)
				fn = fn + k*mTargetVelocity;
			Eigen::Vector3d b1 =  h*fn + h*k*u;
			Eigen::Vector3d b2 = u;
			double denom = 1.0/(m + h*d + h*h*k);

			mUroot = denom*(h*b1+(m+h*d)*b2);
			mdUroot = denom*(b1 + h*k*b2);

		}


		for(int i =0;i<mSkeleton->getNumJoints();i++)
		{
			if(mSkeleton->getJoint(i)->getType()=="BallJoint" || mSkeleton->getJoint(i)->getType()=="FreeJoint")
			{
				double m = 1.0;
				double k = 100.0;
				double d = 2.0*std::sqrt(k);
				double denom = 1.0/(m + h*d+ h*h*k);
				int idx = mSkeleton->getJoint(i)->getIndexInSkeleton(0);

				Eigen::Matrix3d Rn = BallJoint::convertToRotation(mU.segment<3>(idx));
				Eigen::Vector3d fn = mU.segment<3>(idx);
				Eigen::Vector3d wn = mdU.segment<3>(idx);
				Eigen::Vector3d wn1 = denom*(m*wn - h*k*fn);

				Eigen::Matrix3d Rn1 = Rn*BallJoint::convertToRotation(h*wn1);

				mdU.segment<3>(idx) = wn1;
				mU.segment<3>(idx)  = BallJoint::convertToPositions(Rn1);
			}
		}
	}
	else
	{
		if(mCurrentBalanceType!=2){
			
			mU += h*mdU;
		}
		else
		{
			int n = mdU.rows();
			mdU.head<6>().setZero();
			

			double m = 1.0;
			double k = 100.0;
			double d = 2.0*std::sqrt(k);
			double denom = 1.0/(m + h*d+ h*h*k);
			int idx = 0;

			Eigen::Matrix3d Rn = BallJoint::convertToRotation(mU.segment<3>(idx));
			Eigen::Vector3d fn = mU.segment<3>(idx);
			Eigen::Vector3d wn = mdU.segment<3>(idx);
			Eigen::Vector3d wn1 = denom*(m*wn - h*k*fn);

			Eigen::Matrix3d Rn1 = Rn*BallJoint::convertToRotation(h*wn1);

			mdU.segment<3>(idx) = wn1;
			mU.segment<3>(idx)  = BallJoint::convertToPositions(Rn1);

			m = 1.0;
			k = 60.0;
			d = 2.0*std::sqrt(k);
			// Eigen::Vector3d u_bar = mURootBar;
			// u_bar[0] = 0.0;
			Eigen::Vector3d u = mU.segment<3>(3);
			Eigen::Vector3d du = mdU.segment<3>(3);
			fn = -k*(u) - d*du;
			Eigen::Vector3d b1 = m*du + h*fn + h*k*u + h*d*du;
			Eigen::Vector3d b2 = u;
			denom = 1.0/(m + h*d + h*h*k);

			mU.segment<3>(3) = denom*(h*b1+(m+h*d)*b2);
			mdU.segment<3>(3) = denom*(b1 + h*k*b2);

			mU.tail(n-6) += h*mdU.tail(n-6);
		}
		mUroot += h*mdUroot;
	}
	mAppliedForce = false;
	// return;
	// if(mAppliedForce == false)
	// {
	// 	mdU.setZero();
	// 	double m = 1.0;
	// 	double k = 30.0;
	// 	double d = 2.0*std::sqrt(k);
	// 	// Eigen::Vector3d u_bar = mURootBar;
	// 	Eigen::Vector3d u_bar = mSkeleton->getBodyNode(0)->getCOM();
	// 	u_bar += mDefaultVelocity;
	// 	// u_bar[0] = 0.0;
	// 	Eigen::Vector3d u = mU.head<3>();
	// 	Eigen::Vector3d du = mdU.head<3>();
	// 	Eigen::Vector3d fn = -k*(u - u_bar) - d*du;
	// 	Eigen::Vector3d b1 = m*du + h*fn + h*k*u + h*d*du;
	// 	Eigen::Vector3d b2 = u;
	// 	double denom = 1.0/(m + h*d + h*h*k);

	// 	mU.head<3>() = denom*(h*b1+(m+h*d)*b2);
	// 	mdU.head<3>() = denom*(b1 + h*k*b2);

	// 	for(int i =0;i<mSkeleton->getNumJoints();i++)
	// 	{
	// 		if(mSkeleton->getJoint(i)->getType()=="BallJoint")
	// 		{
	// 			double m = 1.0;
	// 			double k = 100.0;
	// 			double d = 2.0*std::sqrt(k);
	// 			double denom = 1.0/(m + h*d+ h*h*k);
	// 			int idx = mSkeleton->getJoint(i)->getIndexInSkeleton(0);

	// 			Eigen::Matrix3d Rn = BallJoint::convertToRotation(mU.segment<3>(idx));
	// 			Eigen::Vector3d fn = mU.segment<3>(idx);
	// 			Eigen::Vector3d wn = mdU.segment<3>(idx);
	// 			Eigen::Vector3d wn1 = denom*(m*wn - h*k*fn);

	// 			Eigen::Matrix3d Rn1 = Rn*BallJoint::convertToRotation(h*wn1);

	// 			mdU.segment<3>(idx) = wn1;
	// 			mU.segment<3>(idx)  = BallJoint::convertToPositions(Rn1);
	// 		}
	// 		else if(mSkeleton->getJoint(i)->getType()=="FreeJoint")
	// 		{
	// 			double m = 1.0;
	// 			double k = 100.0;
	// 			double d = 2.0*std::sqrt(k);
	// 			double denom = 1.0/(m + h*d+ h*h*k);
	// 			int idx = 3;

	// 			Eigen::Matrix3d Rn = BallJoint::convertToRotation(mU.segment<3>(idx));
	// 			Eigen::Vector3d fn = mU.segment<3>(idx);
	// 			Eigen::Vector3d wn = mdU.segment<3>(idx);
	// 			Eigen::Vector3d wn1 = denom*(m*wn - h*k*fn);

	// 			Eigen::Matrix3d Rn1 = Rn*BallJoint::convertToRotation(h*wn1);

	// 			mdU.segment<3>(idx) = wn1;
	// 			mU.segment<3>(idx)  = BallJoint::convertToPositions(Rn1);
	// 		}

	// 	}
	// }
	// else
	// {
	// 	mU.head<3>() += h*mdU.head<3>();
	// 	for(int i =0;i<mSkeleton->getNumJoints();i++)
	// 	{
	// 		if(mSkeleton->getJoint(i)->getType()=="BallJoint")
	// 		{
	// 			int idx = mSkeleton->getJoint(i)->getIndexInSkeleton(0);
	// 			Eigen::Matrix3d R0 = BallJoint::convertToRotation(mU.segment<3>(idx));
	// 			Eigen::Matrix3d R1 = BallJoint::convertToRotation(h*mdU.segment<3>(idx));
				
	// 			mU.segment<3>(idx) = BallJoint::convertToPositions(R0*R1);
				
	// 		}
	// 		else if(mSkeleton->getJoint(i)->getType()=="FreeJoint")
	// 		{
	// 			int idx = 3;
	// 			Eigen::Matrix3d R0 = BallJoint::convertToRotation(mU.segment<3>(idx));
	// 			Eigen::Matrix3d R1 = BallJoint::convertToRotation(h*mdU.segment<3>(idx));
				
	// 			mU.segment<3>(idx) = BallJoint::convertToPositions(R0*R1);
	// 		}
	// 	}

	// 	mURootBar = mU.head<3>();
	// }
	
		
	
	// mAppliedForce = false;
}
Eigen::VectorXd
Character::
getPositions()
{
	Eigen::VectorXd p = mSkeleton->getPositions();
	Eigen::VectorXd u = mU;

	Eigen::VectorXd pu(p.rows() + u.rows());
	pu<<p,u;

	return pu;
}
void
Character::
setPositions(const Eigen::VectorXd& pu)
{
	mSkeleton->setPositions(this->computeDisplacedPositions(pu));
}
Eigen::VectorXd
Character::
computeDisplacedPositions(const Eigen::VectorXd& p, const Eigen::VectorXd& u)
{
	Eigen::VectorXd ret = p;
	for(int i =0;i<mSkeleton->getNumJoints();i++)
	{
		if(mSkeleton->getJoint(i)->getType()=="BallJoint")
		{
			int idx = mSkeleton->getJoint(i)->getIndexInSkeleton(0);
			Eigen::Matrix3d R0 = BallJoint::convertToRotation(p.segment<3>(idx));
			Eigen::Matrix3d R1 = BallJoint::convertToRotation(u.segment<3>(idx));
			
			ret.segment<3>(idx) = BallJoint::convertToPositions(R0*R1);
		}
		else if(mSkeleton->getJoint(i)->getType()=="FreeJoint")
		{
			int idx = 0;

			// Eigen::Matrix3d R0 = BallJoint::convertToRotation(p.segment<3>(idx));
			// Eigen::Matrix3d R1 = BallJoint::convertToRotation(u.segment<3>(idx));
			
			// ret.segment<3>(idx) = BallJoint::convertToPositions(R0*R1);

			// Eigen::Vector3d p0 = p.segment<3>(idx+3);
			// Eigen::Vector3d p1 = u.segment<3>(idx+3);
			
			// ret.segment<3>(idx+3) = p0+ p1;

			Eigen::Isometry3d T0 = FreeJoint::convertToTransform(p.head<6>());
			Eigen::Isometry3d T1 = FreeJoint::convertToTransform(u.head<6>());
			ret.head<6>() = FreeJoint::convertToPositions(T0*T1);

			// Eigen::Matrix3d R0 = BallJoint::convertToRotation(p.segment<3>(0));
			// Eigen::Matrix3d R1 = BallJoint::convertToRotation(u.segment<3>(3));
			
			// ret.segment<3>(0) = BallJoint::convertToPositions(R0*R1);
		}
	}

	return ret;
}
Eigen::VectorXd
Character::
computeDisplacedPositions(const Eigen::VectorXd& pu)
{
	int n = pu.rows()/2;
	assert(n == mSkeleton->getNumDofs());
	Eigen::VectorXd p = pu.head(n);
	Eigen::VectorXd u = pu.tail(n);
	return this->computeDisplacedPositions(p, u);
}



Eigen::VectorXd
Character::
computeOriginalPositions(const Eigen::VectorXd& p, const Eigen::VectorXd& u)
{
	return this->computeDisplacedPositions(p, -u);
}
Eigen::VectorXd
Character::
computeOriginalPositions(const Eigen::VectorXd& pu)
{
	int n = pu.rows()/2;
	assert(n == mSkeleton->getNumDofs());
	Eigen::VectorXd p = pu.head(n);
	Eigen::VectorXd u = pu.tail(n);
	return this->computeOriginalPositions(p, u);
}
Eigen::VectorXd
Character::
getState()
{
	Eigen::Isometry3d T_ref = this->getReferenceTransform();

	Eigen::Isometry3d T_ref_inv = T_ref.inverse();
	Eigen::Matrix3d R_ref_inv = T_ref_inv.linear();

	int n = mSkeleton->getNumBodyNodes();
	std::vector<Eigen::Vector3d> ps(n),vs(n),ws(n);
	std::vector<Eigen::Matrix3d> Rs(n);

	std::vector<Eigen::Vector3d> pus(n);
	std::vector<Eigen::Matrix3d> Rus(n);

	for(int i=0;i<n;i++)
	{
		Eigen::Isometry3d Ti = T_ref_inv * mSkeleton->getBodyNode(i)->getTransform();

		ps[i] = Ti.translation();
		Rs[i] = Ti.linear();

		vs[i] = R_ref_inv*mSkeleton->getBodyNode(i)->getLinearVelocity();
		ws[i] = R_ref_inv*mSkeleton->getBodyNode(i)->getAngularVelocity();
	}

	Eigen::Vector3d p_com = T_ref_inv*mSkeleton->getCOM();
	Eigen::Vector3d v_com = R_ref_inv*mSkeleton->getCOMLinearVelocity();

	this->pushState();
	Eigen::VectorXd q = this->computeOriginalPositions(this->getPositions());
	mSkeleton->setPositions(q);
	for(int i=0;i<n;i++)
	{
		Eigen::Isometry3d Ti = T_ref_inv * mSkeleton->getBodyNode(i)->getTransform();
		pus[i] = Ti.translation();
		Rus[i] = Ti.linear();
	}
	this->popState();
	std::vector<Eigen::Vector3d> states(8*n+2);

	int o = 0;
	for(int i=0;i<n;i++) states[o+i] = ps[i]; o += n;
	for(int i=0;i<n;i++) states[o+i] = pus[i]; o += n;
	for(int i=0;i<n;i++) states[o+i] = Rs[i].col(0); o += n;
	for(int i=0;i<n;i++) states[o+i] = Rs[i].col(1); o += n;
	for(int i=0;i<n;i++) states[o+i] = Rus[i].col(0); o += n;
	for(int i=0;i<n;i++) states[o+i] = Rus[i].col(1); o += n;
	for(int i=0;i<n;i++) states[o+i] = vs[i]; o += n;
	for(int i=0;i<n;i++) states[o+i] = ws[i]; o += n;
	
	states[o+0] = p_com;
	states[o+1] = v_com;
	o += 2;

	// for(int i=0;i<m;i++) states[o+i] = us[i];

	return MathUtils::ravel(states);
}
Eigen::VectorXd
Character::
getStateAMP(const Eigen::VectorXd& pu_curr, const Eigen::VectorXd& pu_prev)
{
	Eigen::VectorXd p_curr = this->computeOriginalPositions(pu_curr);
	Eigen::VectorXd p_prev = this->computeOriginalPositions(pu_prev);

	std::vector<Eigen::VectorXd> states;
	this->pushState();
	mSkeleton->setPositions(p_curr);

	Eigen::Isometry3d T_ref = this->getReferenceTransform();
	// Eigen::Isometry3d T_ref = mSkeleton->getBodyNode(0)->getTrans();

	Eigen::Isometry3d T_ref_inv = T_ref.inverse();
	Eigen::Matrix3d R_ref_inv = T_ref_inv.linear();
	int n = mSkeleton->getNumBodyNodes();
	std::vector<Eigen::Isometry3d> T_curr(n);
	for(int i=0;i<n;i++)
	{
		auto bn = mSkeleton->getBodyNode(i);

		Eigen::Isometry3d T = T_ref_inv*bn->getTransform();

		T_curr[i] = T;
	}
	mSkeleton->setPositions(p_prev);
	for(int i=0;i<n;i++)
	{
		auto bn = mSkeleton->getBodyNode(i);
		Eigen::Isometry3d T = T_ref_inv*bn->getTransform();

		T = T_curr[i].inverse()*T;
		states.emplace_back(T_curr[i].linear().col(0));
		states.emplace_back(T_curr[i].linear().col(1));
		states.emplace_back(T.linear().col(0));
		states.emplace_back(T.linear().col(1));

		if(bn->getNumChildBodyNodes() == 0)
		{
			states.emplace_back(T_curr[i].translation());
			states.emplace_back(T.translation());	
		}
	}	
	this->popState();

	return MathUtils::ravel(states);

}
void
Character::
pushState()
{
	Eigen::VectorXd p = mSkeleton->getPositions();
	Eigen::VectorXd v = mSkeleton->getVelocities();
	Eigen::VectorXd u = mU;
	Eigen::VectorXd du = mdU;
	Eigen::VectorXd pvudu(p.rows()*4);
	pvudu<<p,v,u,du;

	mStates.emplace_back(pvudu);
}
void
Character::
popState()
{
	Eigen::VectorXd pvudu = mStates.back();
	int n = pvudu.rows()/4;
	Eigen::VectorXd p = pvudu.segment(0,n);
	Eigen::VectorXd v = pvudu.segment(n,n);
	Eigen::VectorXd u = pvudu.segment(2*n,n);
	Eigen::VectorXd du = pvudu.segment(3*n,n);
	mSkeleton->setPositions(p);
	mSkeleton->setVelocities(v);
	mU = u;
	mdU = du;
	mStates.pop_back();
}
int
Character::
getBalanceType(const Eigen::VectorXd& force)
{
	return 2;
	double r = force.norm();
	if(r<1e-6)
		return 0;
	double theta = std::atan2(force[0],force[2]);
	int n = mBoundaries[0].rows();
	double dtheta = 2*M_PI/(double)n;
	int i0 = ((int)std::floor(theta/dtheta)+n)%n;
	int i1 = (i0+1)%n;
	double res = theta/dtheta - std::floor(theta/dtheta);


	std::vector<double> boundaries;
	for(int i=0;i<mBoundaries.size();i++)
	{
		double val = (1-res)*mBoundaries[i][i0] + res*mBoundaries[i][i1];
		boundaries.emplace_back(val);
	}

	int count = 0;
	for(int i=0;i<boundaries.size();i++)
	{
		if(r<boundaries[i])
			break;
		count++;
	}
	return count;
}
void
Character::
toggleLight()
{
	mLight = 1 - mLight;
	double r = dart::math::Random::uniform<double>(1.5, 2.0);
	double theta = dart::math::Random::uniform<double>(0.0, 2*M_PI);
	mTargetVelocity = Eigen::Vector3d(r*std::cos(theta), 0.0, r*std::sin(theta));
}