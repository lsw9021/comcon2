#include "Motion.h"
#include "Skeleton.h"
#include "Utils.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <regex>

using namespace kin;

Motion::
Motion(Skeleton* skel)
	:mSkeleton(skel),mTimeStep(1.0/30.0)
{

}
void
Motion::
repeat(int frame, int num)
{
	Eigen::Vector3d pos = mPositions[frame];
	Eigen::MatrixXd rot = mRotations[frame];

	std::vector<Eigen::Vector3d> poss(num);
	std::vector<Eigen::MatrixXd> rots(num);

	for(int i=0;i<num;i++)
		poss[i] = pos;

	for(int i=0;i<num;i++)
		rots[i] = rot;
	this->set(poss, rots);
}
void
Motion::
rotate(double y)
{
	Eigen::Matrix3d Ry = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()).toRotationMatrix();
	Eigen::Vector3d p0 = mPositions[0];
	for(int i=0;i<mRotations.size();i++)
	{
		mRotations[i].block<3,3>(0,0) = Ry*(mRotations[i].block<3,3>(0,0));
		mPositions[i] =  p0 + Ry*(mPositions[i] - p0);
	}

	std::vector<Eigen::Vector3d> pos = mPositions;
	std::vector<Eigen::MatrixXd> rot = mRotations;
	this->set(pos, rot);
}
void
Motion::
translate(const Eigen::Vector3d& tr)
{
	for(int i=0;i<mPositions.size();i++)
		mPositions[i] -= tr;

	std::vector<Eigen::Vector3d> pos = mPositions;
	std::vector<Eigen::MatrixXd> rot = mRotations;
	this->set(pos, rot);
}
void
Motion::
parseBVH(const std::string& file, int start, int end)
{
	std::ifstream ifs(file);

	if(!(ifs.is_open()))
	{
		std::cout<<"Can't read file "<<file<<std::endl;
		exit(0);
	}

	std::string str;

	std::regex channel3_expr("\\s*CHANNELS\\s+3\\s+(\\S+)\\s+(\\S+)\\s+(\\S+)\\s*");
	std::regex frame_expr("\\s*Frames\\s*:\\s*([\\-\\d\\.e]+)\\s*");
	std::regex frame_time_expr("\\s*Frame Time\\s*:\\s*([\\-\\d\\.e]+)\\s*");
	std::smatch what;

	bool pass= false;
	std::string euler = "";
	std::vector<Eigen::Vector3d> positions;
	std::vector<Eigen::MatrixXd> rotations;
	while(!ifs.eof())
	{
		str.clear();
		std::getline(ifs,str);
		if(str.size() == 0)
			continue;
		if(str.find("MOTION") != std::string::npos){
			pass = true;
			continue;
		}
		if(std::regex_match(str,what,channel3_expr) && euler.size()==0)
		{
			std::vector<std::string> channel_order;
			channel_order.emplace_back(what[1]);
			channel_order.emplace_back(what[2]);
			channel_order.emplace_back(what[3]);

			for(int i=0;i<3;i++)
			{
				if(channel_order[i] == "Xrotation" || channel_order[i] == "XROTATION")
					euler += "x";
				else if(channel_order[i] == "Yrotation" || channel_order[i] == "YROTATION")
					euler += "y";
				else if(channel_order[i] == "Zrotation" || channel_order[i] == "ZROTATION")
					euler += "z";
			}
			continue;
		}
		if(pass == false)
			continue;

		if(std::regex_match(str,what,frame_expr)){
			mNumFrames = std::stoi(what[1]);
			continue;
		}

		if(std::regex_match(str,what,frame_time_expr)){
			mTimeStep = std::stod(what[1]);
			continue;
		}
		std::stringstream ss;
		ss.str(str);

		int num_dofs = 3+3*mSkeleton->getNumJoints();
		Eigen::VectorXd m_t(num_dofs);

		for(int i =0;i<num_dofs;i++)
			ss>>m_t[i];
		// Convert to exponential coordinate
		Eigen::Vector3d pos = m_t.head<3>()*0.01;
		Eigen::MatrixXd rot(3,3*mSkeleton->getNumJoints());
		for(int i =0;i<mSkeleton->getNumJoints();i++)
		{
			if(euler == "xyz")
				rot.block<3,3>(0,i*3) = Utils::eulerXYZToMatrix(m_t.segment<3>(i*3+3)*M_PI/180.0);
			else if(euler == "zxy")
				rot.block<3,3>(0,i*3) = Utils::eulerZXYToMatrix(m_t.segment<3>(i*3+3)*M_PI/180.0);
			else if(euler == "zyx")
				rot.block<3,3>(0,i*3) = Utils::eulerZYXToMatrix(m_t.segment<3>(i*3+3)*M_PI/180.0);
			else
				std::cout<<"Not supported bvh euler"<<std::endl;
		}
		positions.emplace_back(pos);
		rotations.emplace_back(rot);
	}
	ifs.close();
	// std::vector<Eigen::Vector3d> pp;
	// std::vector<Eigen::MatrixXd> rr;
	// pp.emplace_back(positions[85]);
	// pp.emplace_back(positions[85]);
	// pp.emplace_back(positions[85]);
	// pp.emplace_back(positions[85]);
	// pp.emplace_back(positions[85]);
	// rr.emplace_back(rotations[85]);
	// rr.emplace_back(rotations[85]);
	// rr.emplace_back(rotations[85]);
	// rr.emplace_back(rotations[85]);
	// rr.emplace_back(rotations[85]);
	// this->set(pp, rr);
	if(start!=-1)
	{
		positions = std::vector<Eigen::Vector3d>(positions.begin() + start, positions.begin() + end);
		rotations = std::vector<Eigen::MatrixXd>(rotations.begin() + start, rotations.begin() + end);
		
	}


	this->set(positions, rotations);
}
void
Motion::
set(const std::vector<Eigen::Vector3d>& positions, const std::vector<Eigen::MatrixXd>& rotations)
{
	mPositions.clear();
	mRotations.clear();
	mLinearVelocities.clear();
	mAngularVelocities.clear();

	mNumFrames = positions.size();
	mPositions = positions;
	mRotations = rotations;

	int num_joints = mSkeleton->getNumJoints();
	for(int i =0;i<mNumFrames;i++)
	{
		int frame1 = std::max(0,            i-1);
		int frame2 = std::min(mNumFrames-1, i+1);

		double dt_inv;
		if(frame1==frame2)
			dt_inv = 0.0;
		else
			dt_inv = 1.0/(mTimeStep*(frame2-frame1));

		Eigen::Vector3d pos1 = mPositions[frame1];
		Eigen::MatrixXd rot1 = mRotations[frame1];

		Eigen::Vector3d pos2 = mPositions[frame2];
		Eigen::MatrixXd rot2 = mRotations[frame2];

		mLinearVelocities.emplace_back(Utils::computeLinearVelocity(pos1, pos2, dt_inv));
		Eigen::MatrixXd angvels;
		angvels.resize(3,num_joints);

		for(int j=0;j<num_joints;j++)
			angvels.col(j) = Utils::computeAngularVelocity(rot1.block<3,3>(0,j*3), rot2.block<3,3>(0,j*3), dt_inv);
		mAngularVelocities.emplace_back(angvels);
	}

}