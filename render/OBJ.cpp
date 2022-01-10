#include "OBJ.h"

#include <GL/glew.h>
#include <GL/glut.h>

OBJ::
OBJ(const std::string& path)
{
	std::ifstream ifs(path);

	if(!(ifs.is_open()))
	{
		std::cout<<"Can't read file "<<path<<std::endl;
		exit(0);
	}

	std::string str;
	std::smatch what;

	std::regex v_expr("v\\s+([\\-\\d\\.e]+)\\s+([\\-\\d\\.e]+)\\s+([\\-\\d\\.e]+)\\s*");
	std::regex vn_expr("vn\\s+([\\-\\d\\.e]+)\\s+([\\-\\d\\.e]+)\\s+([\\-\\d\\.e]+)\\s*");
	// std::regex vn_expr;
	std::regex f_expr("f\\s+([\\-\\d\\.e]+)/([\\-\\d\\.e]+)/([\\-\\d\\.e]+)\\s+([\\-\\d\\.e]+)/([\\-\\d\\.e]+)/([\\-\\d\\.e]+)\\s+([\\-\\d\\.e]+)/([\\-\\d\\.e]+)/([\\-\\d\\.e]+)");

	Eigen::Vector3d v;
	Eigen::Vector3i id;
	while(!ifs.eof())
	{
		str.clear();
		std::getline(ifs, str);

		if(str.size()==0)
			continue;
		if(std::regex_match(str, what, v_expr))
		{
			v[0] = std::stod(what[1]);
			v[1] = std::stod(what[2]);
			v[2] = std::stod(what[3]);
			mVertices.emplace_back(0.01*v);
		}
		else if(std::regex_match(str, what, vn_expr))
		{
			v[0] = std::stod(what[1]);
			v[1] = std::stod(what[2]);
			v[2] = std::stod(what[3]);
			mNormals.emplace_back(v);
		}
		else if(std::regex_match(str, what, f_expr))
		{
			id[0] = std::stoi(what[1])-1;
			id[1] = std::stoi(what[4])-1;
			id[2] = std::stoi(what[7])-1;
			mFaces.emplace_back(id);

			id[0] = std::stoi(what[3])-1;
			id[1] = std::stoi(what[6])-1;
			id[2] = std::stoi(what[9])-1;
			mFaceNormals.emplace_back(id);
		}
	}
	ifs.close();

	int f = mFaces.size();
	mVertexNormalFaces.reserve(f*6);
	for(int i=0;i<f;i++)
	{
		mVertexNormalFaces.emplace_back(mVertices[mFaces[i][0]]);
		mVertexNormalFaces.emplace_back(mNormals[mFaceNormals[i][0]]);
		mVertexNormalFaces.emplace_back(mVertices[mFaces[i][1]]);
		mVertexNormalFaces.emplace_back(mNormals[mFaceNormals[i][1]]);
		mVertexNormalFaces.emplace_back(mVertices[mFaces[i][2]]);
		mVertexNormalFaces.emplace_back(mNormals[mFaceNormals[i][2]]);
	}
}

void
OBJ::
draw()
{
	int f = mVertexNormalFaces.size()/6;
	glBegin(GL_TRIANGLES);
	for(int i=0;i<f;i++)
	{
		glNormal3f(mVertexNormalFaces[i*6+1][0],mVertexNormalFaces[i*6+1][1],mVertexNormalFaces[i*6+1][2]);
		glVertex3f(mVertexNormalFaces[i*6+0][0],mVertexNormalFaces[i*6+0][1],mVertexNormalFaces[i*6+0][2]);

		glNormal3f(mVertexNormalFaces[i*6+3][0],mVertexNormalFaces[i*6+3][1],mVertexNormalFaces[i*6+3][2]);
		glVertex3f(mVertexNormalFaces[i*6+2][0],mVertexNormalFaces[i*6+2][1],mVertexNormalFaces[i*6+2][2]);

		glNormal3f(mVertexNormalFaces[i*6+5][0],mVertexNormalFaces[i*6+5][1],mVertexNormalFaces[i*6+5][2]);
		glVertex3f(mVertexNormalFaces[i*6+4][0],mVertexNormalFaces[i*6+4][1],mVertexNormalFaces[i*6+4][2]);
	}
	glEnd();
}