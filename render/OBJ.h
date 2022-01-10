#ifndef __OBJ_H__
#define __OBJ_H__
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <regex>
#include <sstream>
#include <iterator>
#include <algorithm>
class OBJ
{
public:
	OBJ(const std::string& path);

	void draw();
private:
	std::vector<Eigen::Vector3d> mVertices;
	std::vector<Eigen::Vector3d> mNormals;

	std::vector<Eigen::Vector3i> mFaces;
	std::vector<Eigen::Vector3i> mFaceNormals;

	std::vector<Eigen::Vector3d> mVertexNormalFaces;
};


#endif