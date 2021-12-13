#include "Skeleton.h"
#include "Utils.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <regex>

using namespace kin;

Skeleton::
Skeleton(const std::string& name)
	:mName(name){}

Skeleton*
Skeleton::
create(const std::string& file, const std::string& name)
{
	Skeleton* skel = new Skeleton(name);

	std::ifstream ifs(file);

	if(!(ifs.is_open()))
	{
		std::cout<<"Can't read file "<<file<<std::endl;
		exit(0);
	}

	std::string str;

	std::regex root_expr("\\s*ROOT\\s(\\w+)\\s*");
	std::regex offset_expr("\\s*OFFSET\\s+([\\-\\d\\.e]+)\\s+([\\-\\d\\.e]+)\\s+([\\-\\d\\.e]+)\\s*");
	std::regex channel6_expr("\\s*CHANNELS\\s+6\\s+(\\S+)\\s+(\\S+)\\s+(\\S+)\\s+(\\S+)\\s+(\\S+)\\s+(\\S+)\\s*");
	std::regex channel3_expr("\\s*CHANNELS\\s+3\\s+(\\S+)\\s+(\\S+)\\s+(\\S+)\\s*");
	std::regex joint_expr("\\s*JOINT\\s+(\\w+)\\s*");
	std::regex frame_expr("\\s*Frames\\s*:\\s*([\\-\\d\\.e]+)\\s*");
	std::regex frame_time_expr("\\s*Frame Time\\s*:\\s*([\\-\\d\\.e]+)\\s*");
	std::smatch what;

	bool end_site = false;
	int active = -1;
	std::vector<std::string> channel_order;

	while(!ifs.eof())
	{
		str.clear();
		std::getline(ifs,str);
		if(str.size() == 0)
			continue;
		if(str.find("HIERARCHY") != std::string::npos)
			continue;
		if(str.find("MOTION") != std::string::npos)
			break;
		if(std::regex_match(str,what,root_expr))
		{
			skel->mNodes.emplace_back(what[1]);
			skel->mOffsets.emplace_back(Eigen::Vector3d::Zero());
			skel->mParents.emplace_back(active);
			active = skel->mParents.size() - 1;
			continue;
		}
		if(str.find("{") != std::string::npos)
			continue;
		if(str.find("}") != std::string::npos){
			if(end_site) end_site = false;
			else active = skel->mParents[active];
			continue;
		}
		
		if(std::regex_match(str,what,offset_expr))
		{
			if(end_site == false){
				Eigen::Vector3d offset = Eigen::Vector3d(std::stod(what[1]),std::stod(what[2]),std::stod(what[3]));
				skel->mOffsets[active] = 0.01*offset;
			}
			continue;
		}
		if(std::regex_match(str,what,channel3_expr))
		{
			for(int i =0;i<3;i++)
				channel_order.emplace_back(what[i+1]);
			continue;
		}

		if(std::regex_match(str,what,joint_expr))
		{
			skel->mNodes.emplace_back(what[1]);
			skel->mOffsets.emplace_back(Eigen::Vector3d::Zero());
			skel->mParents.emplace_back(active);
			active = (int)skel->mParents.size() - 1;
			continue;
		}

		if(str.find("End Site") != std::string::npos){
			end_site = true;
			continue;
		}
	}
	ifs.close();
	skel->mNumJoints = skel->mNodes.size();
	return skel;
}