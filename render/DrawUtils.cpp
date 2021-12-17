#include "DrawUtils.h"
#include <iostream>
#include "lodepng.h"
#include <assert.h>


DrawMesh* DrawUtils::point = nullptr;
DrawMesh* DrawUtils::line = nullptr;
DrawMesh* DrawUtils::quad = nullptr;
DrawMesh* DrawUtils::tile = nullptr;
DrawMesh* DrawUtils::box_solid = nullptr;
DrawMesh* DrawUtils::box_wire = nullptr;
DrawMesh* DrawUtils::sphere = nullptr;
DrawMesh* DrawUtils::sphere_wire_simple = nullptr;
DrawMesh* DrawUtils::disk = nullptr;
DrawMesh* DrawUtils::triangle = nullptr;
DrawMesh* DrawUtils::cylinder = nullptr;
DrawMesh* DrawUtils::cylinder_wire_simple = nullptr;
DrawMesh* DrawUtils::cone = nullptr;
DrawMesh* DrawUtils::cone_wire_simple = nullptr;
DrawMesh* DrawUtils::capsule = nullptr;
DrawMesh* DrawUtils::capsule_wire_simple = nullptr;
bool DrawUtils::initialized = false;

GLuint DrawUtils::ground_texture_id = 0;
// GLuint DrawUtils::object_texture_id = 0;

void
DrawMesh::
createVBO(const std::vector<Eigen::Vector3d>& _vertices,
		const std::vector<Eigen::Vector3d>& _normals,
		const std::vector<Eigen::Vector2d>& _texcoords,
		const std::vector<int>& _indices)
{
	this->vboId = 0;
	this->vboId2 = 0;
	this->n = _vertices.size();
	this->m = _indices.size();
	assert(_normals.size() == this->n);
	assert(_texcoords.size() == n);

	this->vertices = new GLfloat[3*this->n];
	this->normals = new GLfloat[3*this->n];
	this->indices = new GLushort[this->m];

	for(int i=0;i<this->n;i++){
		this->vertices[i*3+0] = _vertices[i][0];
		this->vertices[i*3+1] = _vertices[i][1];
		this->vertices[i*3+2] = _vertices[i][2];
	}

	for(int i=0;i<this->n;i++){
		this->normals[i*3+0] = _normals[i][0];
		this->normals[i*3+1] = _normals[i][1];
		this->normals[i*3+2] = _normals[i][2];
	}

	for(int i=0;i<n;i++){
		this->texcoords[i*2+0] = _texcoords[i][0];
		this->texcoords[i*2+1] = _texcoords[i][1];
	}

	for(int i=0;i<this->m;i++){
		this->indices[i] = _indices[i];
	}


	glGenBuffers(1, &this->vboId);
	glBindBuffer(GL_ARRAY_BUFFER, this->vboId);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*8*this->n, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(GLfloat)*0*this->n, sizeof(GLfloat)*3*this->n, this->vertices);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(GLfloat)*3*this->n, sizeof(GLfloat)*3*this->n, this->normals);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*this->n, sizeof(GLfloat)*2*this->n, this->texcoords);

	glGenBuffers(1, &this->vboId2);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vboId2);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->m*sizeof(GLushort), this->indices, GL_STATIC_DRAW);
}
void
DrawMesh::
createVBO(const float* _vertices, const float* _normals, const float* _texcoords,
		const int* _indices, int _n, int _m,GLenum _mode)
{
	this->vboId = 0;
	this->vboId2 = 0;
	this->n = _n;
	this->m = _m;

	this->vertices = new GLfloat[3*this->n];
	this->normals = new GLfloat[3*this->n];
	this->texcoords = new GLfloat[2*this->n];

	this->indices = new GLushort[this->m];
	for(int i=0;i<3*this->n;i++)
		this->vertices[i] = _vertices[i];

	for(int i=0;i<3*this->n;i++)
		this->normals[i] = _normals[i];

	for(int i=0;i<2*this->n;i++)
		this->texcoords[i] = _texcoords[i];

	for(int i=0;i<this->m;i++)
		this->indices[i] = _indices[i];

	glGenBuffers(1, &this->vboId);
	glBindBuffer(GL_ARRAY_BUFFER, this->vboId);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*8*this->n, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(GLfloat)*0*this->n, sizeof(GLfloat)*3*this->n, this->vertices);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(GLfloat)*3*this->n, sizeof(GLfloat)*3*this->n, this->normals);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(GLfloat)*6*this->n, sizeof(GLfloat)*2*this->n, this->texcoords);

	glGenBuffers(1, &this->vboId2);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vboId2);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->m*sizeof(GLushort), this->indices, GL_STATIC_DRAW);
}

void
DrawMesh::
bindVBO()
{
	glBindBuffer(GL_ARRAY_BUFFER, this->vboId);
	
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);


	glVertexPointer(3, GL_FLOAT, 0, 0);
	glNormalPointer(GL_FLOAT, 0, (void*)(sizeof(GLfloat)*3*this->n));
	glTexCoordPointer(2, GL_FLOAT, 0, (void*)(sizeof(GLfloat)*6*this->n));

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vboId2);
	
}
void
DrawMesh::
draw(GLenum mode)
{
	this->draw(mode,0);
}
void
DrawMesh::
draw(GLenum mode, int idx_start)
{
	this->bindVBO();
	glDrawElements(mode, m - idx_start, GL_UNSIGNED_SHORT, (void*)(idx_start * sizeof(GLushort)));
	this->unbindVBO();
}
void
DrawMesh::
draw(GLenum mode, int idx_start, int idx_end)
{
	this->bindVBO();
	int num_elem = std::min(idx_end, m) - idx_start;
	glDrawElements(mode, num_elem, GL_UNSIGNED_SHORT, (void*)(idx_start * sizeof(GLushort)));
	this->unbindVBO();
}
void
DrawMesh::
unbindVBO()
{
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

DrawMesh*
MeshUtils::
buildPointMesh()
{
	const int num_verts = 1;
	const int pos_len = num_verts * 3;
	const int norm_len = num_verts * 3;
	const int coord_len = num_verts * 2;
	const int idx_len = num_verts;

	const float vert_data[pos_len] =
	{
		0, 0, 0
	};

	const float norm_data[norm_len] =
	{
		0, 0, 1
	};

	const float coord_data[coord_len] = {
		0, 0
	};

	int idx_data[idx_len];
	for (int i = 0; i < idx_len; ++i)
	{
		idx_data[i] = i;
	}
	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(vert_data, norm_data, coord_data,idx_data, num_verts, idx_len);
	
	return mesh;
}
DrawMesh*
MeshUtils::
buildLineMesh()
{
	const int num_verts = 2;
	const int pos_len = num_verts * 3;
	const int norm_len = num_verts * 3;
	const int coord_len = num_verts * 2;
	const int idx_len = num_verts;

	const float vert_data[pos_len] =
	{
		0, 0, 0,
		1, 0, 0
	};

	const float norm_data[norm_len] =
	{
		0, 0, 1,
		0, 0, 1
	};

	const float coord_data[coord_len] = {
		0, 0,
		1, 0
	};

	int idx_data[idx_len];
	for (int i = 0; i < idx_len; ++i)
	{
		idx_data[i] = i;
	}
	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(vert_data, norm_data, coord_data,idx_data, num_verts, idx_len);
	
	return mesh;
}
DrawMesh*
MeshUtils::
buildQuadMesh()
{
	const int num_verts = 4;
	const int pos_len = num_verts  * 3;
	const int norm_len = num_verts * 3;
	const int coord_len = num_verts * 2;
	const int idx_len = num_verts;

	const float vert_data[pos_len] =
	{
		0, 0, 0,
		1, 0, 0,
		1, 1, 0,
		0, 1, 0
	};

	const float norm_data[norm_len] =
	{ 
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1
	};

	const float coord_data[coord_len] = { 
		0, 0,
		1, 0,
		1, 1,
		0, 1
	};

	int idx_data[idx_len];
	for (int i = 0; i < idx_len; ++i)
	{
		idx_data[i] = i;
	}
	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(vert_data, norm_data, coord_data,idx_data, num_verts, idx_len);
	
	return mesh;
}
DrawMesh*
MeshUtils::
buildTileMesh(int num)
{
	const int num_faces = num*num;

	const int pos_len = num_faces * 6 * 3;
	const int norm_len = num_faces * 6 * 3;
	const int coord_len = num_faces * 6 * 2;	

	const int idx_len = num_faces * 6;

	std::vector<float> pos_data;
	std::vector<float> norm_data;
	std::vector<float> coord_data;
	std::vector<int> idx_data;

	pos_data.reserve(pos_len);
	norm_data.reserve(norm_len);
	coord_data.reserve(coord_len);
	idx_data.resize(idx_len);

	for(int i=0;i<num;i++)
	{
		double x = (double)i/num-0.5f;
		double x1 = (double)(i+1)/num-0.5f;
		for(int j=0;j<num;j++)
		{
			double z = (double)j/num-0.5f;
			double z1 = (double)(j+1)/num-0.5f;
			pos_data.emplace_back(x);
			pos_data.emplace_back(0.0f);
			pos_data.emplace_back(z);

			pos_data.emplace_back(x1);
			pos_data.emplace_back(0.0f);
			pos_data.emplace_back(z);

			pos_data.emplace_back(x1);
			pos_data.emplace_back(0.0f);
			pos_data.emplace_back(z1);

			pos_data.emplace_back(x);
			pos_data.emplace_back(0.0f);
			pos_data.emplace_back(z);
			
			pos_data.emplace_back(x1);
			pos_data.emplace_back(0.0f);
			pos_data.emplace_back(z1);

			pos_data.emplace_back(x);
			pos_data.emplace_back(0.0f);
			pos_data.emplace_back(z1);

			norm_data.emplace_back(0.0f);
			norm_data.emplace_back(1.0f);
			norm_data.emplace_back(0.0f);

			norm_data.emplace_back(0.0f);
			norm_data.emplace_back(1.0f);
			norm_data.emplace_back(0.0f);

			norm_data.emplace_back(0.0f);
			norm_data.emplace_back(1.0f);
			norm_data.emplace_back(0.0f);

			norm_data.emplace_back(0.0f);
			norm_data.emplace_back(1.0f);
			norm_data.emplace_back(0.0f);

			norm_data.emplace_back(0.0f);
			norm_data.emplace_back(1.0f);
			norm_data.emplace_back(0.0f);

			norm_data.emplace_back(0.0f);
			norm_data.emplace_back(1.0f);
			norm_data.emplace_back(0.0f);

			coord_data.emplace_back(0.0f);
			coord_data.emplace_back(0.0f);

			coord_data.emplace_back(1.0f);
			coord_data.emplace_back(0.0f);

			coord_data.emplace_back(1.0f);
			coord_data.emplace_back(1.0f);
			
			coord_data.emplace_back(0.0f);
			coord_data.emplace_back(0.0f);

			coord_data.emplace_back(1.0f);
			coord_data.emplace_back(1.0f);	

			coord_data.emplace_back(0.0f);
			coord_data.emplace_back(1.0f);
		}
	}
	for (int i = 0; i < idx_len; ++i)
	{
		idx_data.emplace_back(i);
	}

	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(pos_data.data(),
		norm_data.data(),
		coord_data.data(),
		idx_data.data(), num_faces * 6, static_cast<int>(idx_data.size()));
	
	return mesh;
}
DrawMesh*
MeshUtils::
buildBoxSolidMesh()
{
	const int num_faces = 6;
	const int vert_size = num_faces * 6 * 3;
	const int norm_size = num_faces * 6 * 3;
	const int coord_size = num_faces * 6 * 2;
	const int idx_size = num_faces * 6;

	const float vert_data[vert_size] = {
		0.5, 0.5, -0.5, // top
		-0.5, 0.5, -0.5,
		-0.5, 0.5, 0.5,
		-0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, -0.5,

		0.5, -0.5, 0.5, // bottom
		-0.5, -0.5, 0.5,
		-0.5, -0.5, -0.5,
		-0.5, -0.5, -0.5,
		0.5, -0.5, -0.5,
		0.5, -0.5, 0.5,

		0.5, -0.5, 0.5, // front
		0.5, -0.5, -0.5,
		0.5, 0.5, -0.5,
		0.5, 0.5, -0.5,
		0.5, 0.5, 0.5,
		0.5, -0.5, 0.5,

		-0.5, -0.5, -0.5, // back
		-0.5, -0.5, 0.5,
		-0.5, 0.5, 0.5,
		-0.5, 0.5, 0.5,
		-0.5, 0.5, -0.5,
		-0.5, -0.5, -0.5,

		-0.5, -0.5, -0.5, // left
		-0.5, 0.5, -0.5,
		0.5, 0.5, -0.5,
		0.5, 0.5, -0.5,
		0.5, -0.5, -0.5,
		-0.5, -0.5, -0.5,

		0.5, -0.5, 0.5, // right
		0.5, 0.5, 0.5,
		-0.5, 0.5, 0.5,
		-0.5, 0.5, 0.5,
		-0.5, -0.5, 0.5,
		0.5, -0.5, 0.5
	};

	const float norm_data[vert_size] = {
		0, 1, 0, // top
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,

		0, -1, 0, // bottom
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,

		1, 0, 0, // front
		1, 0, 0,
		1, 0, 0,
		1, 0, 0,
		1, 0, 0,
		1, 0, 0,

		-1, 0, 0, // back
		-1, 0, 0,
		-1, 0, 0,
		-1, 0, 0,
		-1, 0, 0,
		-1, 0, 0,

		0, 0, -1, // left
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,
		0, 0, -1,

		0, 0, 1, // right
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
		0, 0, 1,
	};


	const float coord_data[coord_size] = {
		0.0, 0.5, // top
		1.0/3.0, 0.5,
		1.0/3.0, 1,
		1.0/3.0, 1,
		0, 1,
		0, 0.5,

		2.0/3.0, 0.5, // bottom
		2.0/3.0, 1,
		1.0/3.0, 1,
		1.0/3.0, 1,
		1.0/3.0, 0.5,
		2.0/3.0, 0.5,

		2.0/3.0, 0.5, // front
		1.0, 0.5,
		1.0, 1,
		1.0, 1,
		2.0/3.0, 1,
		2.0/3.0, 0.5,

		0, 0, // back
		1.0/3.0, 0,
		1.0/3.0, 0.5,
		1.0/3.0, 0.5,
		0, 0.5,
		0, 0,

		2.0/3.0, 0, // left
		2.0/3.0, 0.5,
		1.0/3.0, 0.5,
		1.0/3.0, 0.5,
		1.0/3.0, 0,
		2.0/3.0, 0,

		1, 0, // right
		1, 0.5,
		2.0/3.0, 0.5,
		2.0/3.0, 0.5,
		2.0/3.0, 0,
		1, 0
	};

	int idx_data[idx_size];
	for (int i = 0; i < idx_size; ++i)
	{
		idx_data[i] = i;
	}

	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(vert_data, norm_data, coord_data, idx_data, num_faces*6, idx_size);
	
	return mesh;
}
DrawMesh*
MeshUtils::
buildBoxWireMesh()
{
	const int num_edges = 12;
	const int vert_size = num_edges * 2 * 3;
	const int norm_size = num_edges * 2 * 3;
	const int coord_size = num_edges * 2 * 2;
	const int idx_size = num_edges * 2;

	const float vert_data[vert_size] = {
		0.5, 0.5, -0.5, // top
		-0.5, 0.5, -0.5,
		-0.5, 0.5, -0.5,
		-0.5, 0.5, 0.5,
		-0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, 0.5,
		0.5, 0.5, -0.5,

		0.5, -0.5, 0.5, // bottom
		-0.5, -0.5, 0.5,
		-0.5, -0.5, 0.5,
		-0.5, -0.5, -0.5,
		-0.5, -0.5, -0.5,
		0.5, -0.5, -0.5,
		0.5, -0.5, -0.5,
		0.5, -0.5, 0.5,

		0.5, -0.5, 0.5, // front
		0.5, 0.5, 0.5,

		-0.5, -0.5, -0.5, // back
		-0.5, 0.5, -0.5,
		
		0.5, 0.5, -0.5, // left
		0.5, -0.5, -0.5,
		
		-0.5, 0.5, 0.5, // right
		-0.5, -0.5, 0.5
	};

	const float norm_data[vert_size] = {
		0, 1, 0, // top
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,
		0, 1, 0,

		0, -1, 0, // bottom
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,
		0, -1, 0,

		1, 0, 0, // front
		1, 0, 0,

		-1, 0, 0, // back
		-1, 0, 0,

		0, 0, -1, // left
		0, 0, -1,

		0, 0, 1, // right
		0, 0, 1
	};


	const float coord_data[vert_size] = {
		0, 0, // top
		1, 0,
		1, 0,
		1, 1,
		1, 1,
		0, 1,
		0, 1,
		0, 0,

		1, 0, // bottom
		1, 1,
		1, 1,
		0, 1,
		0, 1,
		0, 0,
		0, 0,
		1, 0,

		0, 0, // front
		0, 1,
		
		0, 0, // back
		0, 1,

		0, 1, // left
		0, 0,

		0, 1, // right
		0, 0
	};

	int idx_data[idx_size];
	for (int i = 0; i < idx_size; ++i)
	{
		idx_data[i] = i;
	}
	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(vert_data, norm_data, coord_data, idx_data, num_edges*12, idx_size);
	
	return mesh;
}
DrawMesh*
MeshUtils::
buildSphereMesh(int stacks, int slices)
{
	int num_triangles = 2 * slices * (stacks - 1);

	std::vector<float> pos_data;
	std::vector<float> norm_data;
	std::vector<float> coord_data;
	std::vector<int> idx_data;
	pos_data.reserve(num_triangles * 3 * 3);
	norm_data.reserve(num_triangles * 3 * 3);
	coord_data.reserve(num_triangles * 3 * 2);
	idx_data.resize(num_triangles * 3);

	for (int i = 0; i < stacks; ++i)
	{
		float rho0 = 0.5 * M_PI - (i * M_PI) / stacks;
		float rho1 = 0.5 * M_PI - ((i + 1) * M_PI) / stacks;
		float coord_v0 = 0.20;//1 - static_cast<float>(i) / stacks;
		float coord_v1 = 0.21;//1 - static_cast<float>(i + 1) / stacks;

		float y0 = std::sin(rho0);
		float r0 = std::cos(rho0);
		float y1 = std::sin(rho1);
		float r1 = std::cos(rho1);
		float y2 = y1;
		float y3 = y0;

		for (int j = 0; j < slices; ++j)
		{
			float theta0 = (j * 2 * M_PI) / slices;
			float theta1 = ((j + 1) * 2 * M_PI) / slices;
			float coord_u0 = 0.20;//static_cast<float>(j) / slices;
			float coord_u1 = 0.21;//static_cast<float>(j + 1) / slices;

			float x0 = r0 * std::cos(theta0);
			float z0 = r0 * std::sin(-theta0);
			float u0 = coord_u0;
			float v0 = coord_v0;
			float x1 = r1 * std::cos(theta0);
			float z1 = r1 * std::sin(-theta0);
			float u1 = coord_u0;
			float v1 = coord_v1;
			float x2 = r1 * std::cos(theta1);
			float z2 = r1 * std::sin(-theta1);
			float u2 = coord_u1;
			float v2 = coord_v1;
			float x3 = r0 * std::cos(theta1);
			float z3 = r0 * std::sin(-theta1);
			float u3 = coord_u1;
			float v3 = coord_v0;

			if (i != stacks - 1)
			{
				pos_data.emplace_back(x0);
				pos_data.emplace_back(y0);
				pos_data.emplace_back(z0);
				norm_data.emplace_back(x0);
				norm_data.emplace_back(y0);
				norm_data.emplace_back(z0);
				coord_data.emplace_back(u0);
				coord_data.emplace_back(v0);

				pos_data.emplace_back(x1);
				pos_data.emplace_back(y1);
				pos_data.emplace_back(z1);
				norm_data.emplace_back(x1);
				norm_data.emplace_back(y1);
				norm_data.emplace_back(z1);
				coord_data.emplace_back(u1);
				coord_data.emplace_back(v1);

				pos_data.emplace_back(x2);
				pos_data.emplace_back(y2);
				pos_data.emplace_back(z2);
				norm_data.emplace_back(x2);
				norm_data.emplace_back(y2);
				norm_data.emplace_back(z2);
				coord_data.emplace_back(u2);
				coord_data.emplace_back(v2);
			}
			
			if (i != 0)
			{
				pos_data.emplace_back(x2);
				pos_data.emplace_back(y2);
				pos_data.emplace_back(z2);
				norm_data.emplace_back(x2);
				norm_data.emplace_back(y2);
				norm_data.emplace_back(z2);
				coord_data.emplace_back(u2);
				coord_data.emplace_back(v2);

				pos_data.emplace_back(x3);
				pos_data.emplace_back(y3);
				pos_data.emplace_back(z3);
				norm_data.emplace_back(x3);
				norm_data.emplace_back(y3);
				norm_data.emplace_back(z3);
				coord_data.emplace_back(u3);
				coord_data.emplace_back(v3);

				pos_data.emplace_back(x0);
				pos_data.emplace_back(y0);
				pos_data.emplace_back(z0);
				norm_data.emplace_back(x0);
				norm_data.emplace_back(y0);
				norm_data.emplace_back(z0);
				coord_data.emplace_back(u0);
				coord_data.emplace_back(v0);
			}
		}
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}
	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(pos_data.data(),
		norm_data.data(),
		coord_data.data(),
		idx_data.data(), num_triangles * 3, static_cast<int>(idx_data.size()));
	
	return mesh;
}
DrawMesh*
MeshUtils::
buildSphereWireSimpleMesh(int stacks, int slices)
{
	int slices0 = 4;
	int num_lines = slices0 * stacks + slices;

	std::vector<float> pos_data;
	std::vector<float> norm_data;
	std::vector<float> coord_data;
	std::vector<int> idx_data;
	pos_data.resize(num_lines * 2 * 3);
	norm_data.resize(num_lines * 2 * 3);
	coord_data.resize(num_lines * 2 * 2);
	idx_data.resize(num_lines * 2);

	for (int i = 0; i < stacks; ++i)
	{
		float rho0 = 0.5 * M_PI - (i * M_PI) / stacks;
		float rho1 = 0.5 * M_PI - ((i + 1) * M_PI) / stacks;
		float coord_v0 = 1 - static_cast<float>(i) / stacks;
		float coord_v1 = 1 - static_cast<float>(i + 1) / stacks;

		float y0 = std::sin(rho0);
		float r0 = std::cos(rho0);
		float y1 = std::sin(rho1);
		float r1 = std::cos(rho1);

		for (int j = 0; j < slices0; ++j)
		{
			float theta0 = (j * 2 * M_PI) / slices0;
			float coord_u0 = static_cast<float>(j) / slices0;

			float x0 = r0 * std::cos(theta0);
			float z0 = r0 * std::sin(-theta0);
			float u0 = coord_u0;
			float v0 = coord_v0;
			float x1 = r1 * std::cos(theta0);
			float z1 = r1 * std::sin(-theta0);
			float u1 = coord_u0;
			float v1 = coord_v1;

			int pos_offset = (i * slices0 + j) * 2 * 3;
			int norm_offset = (i * slices0 + j) * 2 * 3;
			int coord_offset = (i * slices0 + j) * 2 * 2;

			if (i >= stacks / 2)
			{
				pos_offset += slices * 2 * 3;
				norm_offset += slices * 2 * 3;
				coord_offset += slices * 2 * 2;
			}

			pos_data[pos_offset + 0] = x0;
			pos_data[pos_offset + 1] = y0;
			pos_data[pos_offset + 2] = z0;
			norm_data[norm_offset + 0] = x0;
			norm_data[norm_offset + 1] = y0;
			norm_data[norm_offset + 2] = z0;
			coord_data[coord_offset + 0] = u0;
			coord_data[coord_offset + 1] = v0;

			pos_data[pos_offset + 3] = x1;
			pos_data[pos_offset + 4] = y1;
			pos_data[pos_offset + 5] = z1;
			norm_data[norm_offset + 3] = x1;
			norm_data[norm_offset + 4] = y1;
			norm_data[norm_offset + 5] = z1;
			coord_data[coord_offset + 2] = u1;
			coord_data[coord_offset + 3] = v1;
		}
	}

	for (int j = 0; j < slices; ++j)
	{
		float theta0 = (j * 2 * M_PI) / slices;
		float theta1 = ((j + 1) * 2 * M_PI) / slices;
		float coord_u0 = static_cast<float>(j) / slices;
		float coord_v0 = 0.5;
		float coord_v1 = 0.5;

		float x0 = std::cos(theta0);
		float y0 = 0;
		float z0 = std::sin(-theta0);
		float u0 = coord_u0;
		float v0 = coord_v0;
		float x1 = std::cos(theta1);
		float y1 = 0;
		float z1 = std::sin(-theta1);
		float u1 = coord_u0;
		float v1 = coord_v1;

		int pos_offset = (stacks / 2 * slices0 + j) * 2 * 3;
		int norm_offset = (stacks / 2 * slices0 + j) * 2 * 3;
		int coord_offset = (stacks / 2 * slices0 + j) * 2 * 2;

		pos_data[pos_offset + 0] = x0;
		pos_data[pos_offset + 1] = y0;
		pos_data[pos_offset + 2] = z0;
		norm_data[norm_offset + 0] = x0;
		norm_data[norm_offset + 1] = y0;
		norm_data[norm_offset + 2] = z0;
		coord_data[coord_offset + 0] = u0;
		coord_data[coord_offset + 1] = v0;

		pos_data[pos_offset + 3] = x1;
		pos_data[pos_offset + 4] = y1;
		pos_data[pos_offset + 5] = z1;
		norm_data[norm_offset + 3] = x1;
		norm_data[norm_offset + 4] = y1;
		norm_data[norm_offset + 5] = z1;
		coord_data[coord_offset + 2] = u1;
		coord_data[coord_offset + 3] = v1;
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(pos_data.data(),
		norm_data.data(),
		coord_data.data(),
		idx_data.data(),num_lines * 2, static_cast<int>(idx_data.size()));
	
	return mesh;
}
DrawMesh*
MeshUtils::
buildDiskMesh(int slices)
{
	const int num_verts = 2 + slices;

	std::vector<float> pos_data(num_verts * 3);
	std::vector<float> norm_data(num_verts * 3);
	std::vector<float> coord_data(num_verts * 2);
	std::vector<int> idx_data(num_verts);

	pos_data[0] = 0;
	pos_data[1] = 0;
	pos_data[2] = 0;
	norm_data[0] = 0;
	norm_data[1] = 0;
	norm_data[2] = 1;
	coord_data[0] = 0.5;
	coord_data[1] = 0.5;

	for (int i = 0; i <= slices; ++i)
	{
		float theta = (i * 2 * M_PI) / slices;
		int pos_offset = (i + 1) * 3;
		int norm_offset = (i + 1) * 3;
		int coord_offset = (i + 1) * 2;

		pos_data[pos_offset + 0] = std::cos(theta);
		pos_data[pos_offset + 1] = std::sin(theta);
		pos_data[pos_offset + 2] = 0;
		norm_data[norm_offset + 0] = 0;
		norm_data[norm_offset + 1] = 0;
		norm_data[norm_offset + 2] = 1;
		coord_data[coord_offset + 0] = 0.5+0.5*std::cos(theta);
		coord_data[coord_offset + 1] = 0.5+0.5*std::sin(theta);
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(pos_data.data(),
		norm_data.data(),
		coord_data.data(),
		idx_data.data(),num_verts, static_cast<int>(idx_data.size()));
	
	return mesh;

}
DrawMesh*
MeshUtils::
buildTriangleMesh()
{
	const float side_len = 1;
	const float h = std::sqrt(0.75 * side_len * side_len);
	const int num_verts = 3;
	const int vert_len = num_verts * 3;
	const int norm_len = num_verts * 3;
	const int coord_len = num_verts * 2;
	const int idx_len = num_verts;

	const float vert_data[vert_len] =
	{
		-0.5f * side_len, -0.5f * h, 0.f,
		0.5f * side_len, -0.5f * h, 0.f,
		0.f, 0.5f * h, 0.f
	};

	const float norm_data[norm_len] =
	{
		0, 0, 1,
		0, 0, 1,
		0, 0, 1
	};


	const float coord_data[coord_len] = {
		0, 0,
		1, 0,
		0.5, 1
	};

	int idx_data[idx_len] = { 0, 1, 2 };

	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(vert_data, norm_data, coord_data, idx_data, num_verts, idx_len);
	
	return mesh;
}

DrawMesh*
MeshUtils::
buildCylinder(int slices)
{
	const int num_verts = 12 * slices;

	std::vector<float> pos_data(num_verts * 3);
	std::vector<float> norm_data(num_verts * 3);
	std::vector<float> coord_data(num_verts * 2);
	std::vector<int> idx_data(num_verts);
	
	for (int i = 0; i < slices; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices;
		double theta1 = ((i + 1) * 2 * M_PI) / slices;

		double x0 = std::cos(theta0);
		double z0 = std::sin(-theta0);
		double u0 = static_cast<float>(i) / slices;

		double x1 = std::cos(theta1);
		double z1 = std::sin(-theta1);
		double u1 = static_cast<float>(i + 1) / slices;

		Eigen::Vector3d n0 = Eigen::Vector3d(x0, 0, z0).normalized();
		Eigen::Vector3d n1 = Eigen::Vector3d(x1, 0, z1).normalized();

		int pos_offset = i * 6 * 3;
		int norm_offset = i * 6 * 3;
		int coord_offset = i * 6 * 2;

		pos_data[pos_offset] = x0;
		pos_data[pos_offset + 1] = -0.5;
		pos_data[pos_offset + 2] = z0;

		pos_data[pos_offset + 3] = x1;
		pos_data[pos_offset + 4] = -0.5;
		pos_data[pos_offset + 5] = z1;

		pos_data[pos_offset + 6] = x1;
		pos_data[pos_offset + 7] = 0.5;
		pos_data[pos_offset + 8] = z1;

		pos_data[pos_offset + 9] = x1;
		pos_data[pos_offset + 10] = 0.5;
		pos_data[pos_offset + 11] = z1;

		pos_data[pos_offset + 12] = x0;
		pos_data[pos_offset + 13] = 0.5;
		pos_data[pos_offset + 14] = z0;

		pos_data[pos_offset + 15] = x0;
		pos_data[pos_offset + 16] = -0.5;
		pos_data[pos_offset + 17] = z0;

		norm_data[norm_offset] = n0[0];
		norm_data[norm_offset + 1] = n0[1];
		norm_data[norm_offset + 2] = n0[2];
		norm_data[norm_offset + 3] = n1[0];
		norm_data[norm_offset + 4] = n1[1];
		norm_data[norm_offset + 5] = n1[2];
		norm_data[norm_offset + 6] = n1[0];
		norm_data[norm_offset + 7] = n1[1];
		norm_data[norm_offset + 8] = n1[2];
		norm_data[norm_offset + 9] = n1[0];
		norm_data[norm_offset + 10] = n1[1];
		norm_data[norm_offset + 11] = n1[2];
		norm_data[norm_offset + 12] = n0[0];
		norm_data[norm_offset + 13] = n0[1];
		norm_data[norm_offset + 14] = n0[2];
		norm_data[norm_offset + 15] = n0[0];
		norm_data[norm_offset + 16] = n0[1];
		norm_data[norm_offset + 17] = n0[2];

		coord_data[coord_offset] = u0;
		coord_data[coord_offset + 1] = 0.f;
		coord_data[coord_offset + 2] = u1;
		coord_data[coord_offset + 3] = 0.f;
		coord_data[coord_offset + 4] = u1;
		coord_data[coord_offset + 5] = 1;
		coord_data[coord_offset + 6] = u1;
		coord_data[coord_offset + 7] = 1.f;
		coord_data[coord_offset + 8] = u0;
		coord_data[coord_offset + 9] = 1.f;
		coord_data[coord_offset + 10] = u0;
		coord_data[coord_offset + 11] = 0.f;

		pos_offset = (slices * 6 + i * 6) * 3;
		norm_offset = (slices * 6 + i * 6) * 3;
		coord_offset = (slices * 6 + i * 6) * 2;

		pos_data[pos_offset] = x0;
		pos_data[pos_offset + 1] = 0.5;
		pos_data[pos_offset + 2] = z0;

		pos_data[pos_offset + 3] = x1;
		pos_data[pos_offset + 4] = 0.5;
		pos_data[pos_offset + 5] = z1;

		pos_data[pos_offset + 6] = 0.f;
		pos_data[pos_offset + 7] = 0.5;
		pos_data[pos_offset + 8] = 0.f;

		pos_data[pos_offset + 9] = 0.f;
		pos_data[pos_offset + 10] = -0.5;
		pos_data[pos_offset + 11] = 0.f;

		pos_data[pos_offset + 12] = x1;
		pos_data[pos_offset + 13] = -0.5;
		pos_data[pos_offset + 14] = z1;

		pos_data[pos_offset + 15] = x0;
		pos_data[pos_offset + 16] = -0.5;
		pos_data[pos_offset + 17] = z0;

		norm_data[norm_offset] = 0.f;
		norm_data[norm_offset + 1] = 1.f;
		norm_data[norm_offset + 2] = 0.f;
		norm_data[norm_offset + 3] = 0.f;
		norm_data[norm_offset + 4] = 1.f;
		norm_data[norm_offset + 5] = 0.f;
		norm_data[norm_offset + 6] = 0.f;
		norm_data[norm_offset + 7] = 1.f;
		norm_data[norm_offset + 8] = 0.f;
		norm_data[norm_offset + 9] = 0.f;
		norm_data[norm_offset + 10] = -1.f;
		norm_data[norm_offset + 11] = 0.f;
		norm_data[norm_offset + 12] = 0.f;
		norm_data[norm_offset + 13] = -1.f;
		norm_data[norm_offset + 14] = 0.f;
		norm_data[norm_offset + 15] = 0.f;
		norm_data[norm_offset + 16] = -1.f;
		norm_data[norm_offset + 17] = 0.f;

		coord_data[coord_offset + 0] = 0.5f + 0.5f*x0;
		coord_data[coord_offset + 1] = 0.5f + 0.5f*z0;

		coord_data[coord_offset + 2] = 0.5f + 0.5f*x1;
		coord_data[coord_offset + 3] = 0.5f + 0.5f*z1;

		coord_data[coord_offset + 4] = 0.5f;
		coord_data[coord_offset + 5] = 0.5f;

		coord_data[coord_offset + 6] = 0.5f;
		coord_data[coord_offset + 7] = 0.5f;

		coord_data[coord_offset + 8] = 0.5f + 0.5f*x1;
		coord_data[coord_offset + 9] = 0.5f + 0.5f*z1;

		coord_data[coord_offset + 10] = 0.5f + 0.5f*x0;
		coord_data[coord_offset + 11] = 0.5f + 0.5f*z0;
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(pos_data.data(),
		norm_data.data(),
		coord_data.data(),
		idx_data.data(),num_verts, static_cast<int>(idx_data.size()));
	
	return mesh;
}
DrawMesh*
MeshUtils::
buildCylinderWireSimple(int slices)
{
	const int slices0 = 4;
	const int num_verts = 4 * slices + 2 * slices0;

	std::vector<float> pos_data(num_verts * 3);
	std::vector<float> norm_data(num_verts * 3);
	std::vector<float> coord_data(num_verts * 2);
	std::vector<int> idx_data(num_verts);
	
	for (int i = 0; i < slices; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices;
		double theta1 = ((i + 1) * 2 * M_PI) / slices;

		double x0 = std::cos(theta0);
		double z0 = std::sin(-theta0);
		double u0 = static_cast<float>(i) / slices;

		double x1 = std::cos(theta1);
		double z1 = std::sin(-theta1);
		double u1 = static_cast<float>(i + 1) / slices;

		Eigen::Vector3d n0 = Eigen::Vector3d(0, -1, 0).normalized();
		Eigen::Vector3d n1 = Eigen::Vector3d(0, 1, 0).normalized();

		int pos_offset = i * 4 * 3;
		int norm_offset = i * 4 * 3;
		int coord_offset = i * 4 * 2;

		pos_data[pos_offset] = x0;
		pos_data[pos_offset + 1] = -0.5;
		pos_data[pos_offset + 2] = z0;
		pos_data[pos_offset + 3] = x1;
		pos_data[pos_offset + 4] = -0.5;
		pos_data[pos_offset + 5] = z1;

		norm_data[norm_offset] = n0[0];
		norm_data[norm_offset + 1] = n0[1];
		norm_data[norm_offset + 2] = n0[2];
		norm_data[norm_offset + 3] = n0[0];
		norm_data[norm_offset + 4] = n0[1];
		norm_data[norm_offset + 5] = n0[2];

		coord_data[coord_offset] = u0;
		coord_data[coord_offset + 1] = 0.f;
		coord_data[coord_offset + 2] = u1;
		coord_data[coord_offset + 3] = 0.f;

		pos_data[pos_offset + 6] = x0;
		pos_data[pos_offset + 7] = 0.5;
		pos_data[pos_offset + 8] = z0;
		pos_data[pos_offset + 9] = x1;
		pos_data[pos_offset + 10] = 0.5;
		pos_data[pos_offset + 11] = z1;

		norm_data[norm_offset + 6] = n1[0];
		norm_data[norm_offset + 7] = n1[1];
		norm_data[norm_offset + 8] = n1[2];
		norm_data[norm_offset + 9] = n1[0];
		norm_data[norm_offset + 10] = n1[1];
		norm_data[norm_offset + 11] = n1[2];

		coord_data[coord_offset + 4] = u0;
		coord_data[coord_offset + 5] = 1.f;
		coord_data[coord_offset + 6] = u1;
		coord_data[coord_offset + 7] = 1.f;
	}

	for (int i = 0; i < slices0; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices0;

		double x0 = std::cos(theta0);
		double z0 = std::sin(-theta0);
		double u0 = static_cast<float>(i) / slices0;

		Eigen::Vector3d n0 = Eigen::Vector3d(x0, 0, z0).normalized();

		int pos_offset = (slices * 4 + i * 2) * 3;
		int norm_offset = (slices * 4 + i * 2) * 3;
		int coord_offset = (slices * 4 + i * 2) * 2;

		pos_data[pos_offset] = x0;
		pos_data[pos_offset + 1] = -0.5;
		pos_data[pos_offset + 2] = z0;
		pos_data[pos_offset + 3] = x0;
		pos_data[pos_offset + 4] = 0.5;
		pos_data[pos_offset + 5] = z0;

		norm_data[norm_offset] = n0[0];
		norm_data[norm_offset + 1] = n0[1];
		norm_data[norm_offset + 2] = n0[2];
		norm_data[norm_offset + 3] = n0[0];
		norm_data[norm_offset + 4] = n0[1];
		norm_data[norm_offset + 5] = n0[2];

		coord_data[coord_offset] = u0;
		coord_data[coord_offset + 1] = 0.f;
		coord_data[coord_offset + 2] = u0;
		coord_data[coord_offset + 3] = 1.f;
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(pos_data.data(),
		norm_data.data(),
		coord_data.data(),
		idx_data.data(),num_verts, static_cast<int>(idx_data.size()));
	
	return mesh;
}
DrawMesh*
MeshUtils::
buildCone(int slices)
{
	const int num_verts = 6 * slices;

	std::vector<float> pos_data(num_verts * 3);
	std::vector<float> norm_data(num_verts * 3);
	std::vector<float> coord_data(num_verts * 2);
	std::vector<int> idx_data(num_verts);

	for (int i = 0; i < slices; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices;
		double theta1 = ((i + 1) * 2 * M_PI) / slices;

		double x0 = std::cos(theta0);
		double z0 = std::sin(-theta0);
		double u0 = static_cast<double>(i) / slices;

		double x1 = std::cos(theta1);
		double z1 = std::sin(-theta1);
		double u1 = static_cast<double>(i + 1) / slices;

		double u2 = (i + 0.5) / slices;

		Eigen::Vector3d n0 = Eigen::Vector3d(std::cos(theta0), 1.0, -std::sin(theta0)).normalized();
		Eigen::Vector3d n1 = Eigen::Vector3d(std::cos(theta1), 1.0, -std::sin(theta1)).normalized();
		Eigen::Vector3d n2 = Eigen::Vector3d(std::cos(0.5 * (theta0 + theta1)), 1.0, -std::sin(0.5 * (theta0 + theta1))).normalized();

		int pos_offset = i * 3 * 3;
		int norm_offset = i * 3 * 3;
		int coord_offset = i * 3 * 2;

		pos_data[pos_offset] = 0;
		pos_data[pos_offset + 1] = 1.0;
		pos_data[pos_offset + 2] = 0;
		pos_data[pos_offset + 3] = x0;
		pos_data[pos_offset + 4] = 0;
		pos_data[pos_offset + 5] = z0;
		pos_data[pos_offset + 6] = x1;
		pos_data[pos_offset + 7] = 0;
		pos_data[pos_offset + 8] = z1;
		
		norm_data[norm_offset] = n2[0];
		norm_data[norm_offset + 1] = n2[1];
		norm_data[norm_offset + 2] = n2[2];
		norm_data[norm_offset + 3] = n0[0];
		norm_data[norm_offset + 4] = n0[1];
		norm_data[norm_offset + 5] = n0[2];
		norm_data[norm_offset + 6] = n1[0];
		norm_data[norm_offset + 7] = n1[1];
		norm_data[norm_offset + 8] = n1[2];

		coord_data[coord_offset] = u2;
		coord_data[coord_offset + 1] = 1.f;
		coord_data[coord_offset + 2] = u0;
		coord_data[coord_offset + 3] = 0.f;
		coord_data[coord_offset + 4] = u1;
		coord_data[coord_offset + 5] = 0.f;

		pos_offset = (slices * 3 + i * 3) * 3;
		norm_offset = (slices * 3 + i * 3) * 3;
		coord_offset = (slices * 3 + i * 3) * 2;

		pos_data[pos_offset] = x1;
		pos_data[pos_offset + 1] = 0;
		pos_data[pos_offset + 2] = z1;
		pos_data[pos_offset + 3] = x0;
		pos_data[pos_offset + 4] = 0;
		pos_data[pos_offset + 5] = z0;
		pos_data[pos_offset + 6] = 0;
		pos_data[pos_offset + 7] = 0;
		pos_data[pos_offset + 8] = 0;

		norm_data[norm_offset] = 0.f;
		norm_data[norm_offset + 1] = -1.f;
		norm_data[norm_offset + 2] = 0.f;
		norm_data[norm_offset + 3] = 0.f;
		norm_data[norm_offset + 4] = -1.f;
		norm_data[norm_offset + 5] = 0.f;
		norm_data[norm_offset + 6] = 0.f;
		norm_data[norm_offset + 7] = -1.f;
		norm_data[norm_offset + 8] = 0.f;

		coord_data[coord_offset] = 0.5f + 0.5f*x1;
		coord_data[coord_offset + 1] = 0.5f + 0.5f*z1;
		coord_data[coord_offset + 2] = 0.5f + 0.5f*x0;
		coord_data[coord_offset + 3] = 0.5f + 0.5f*z0;
		coord_data[coord_offset + 4] = 0.5f;
		coord_data[coord_offset + 5] = 0.5f;
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(pos_data.data(),
		norm_data.data(),
		coord_data.data(),
		idx_data.data(),num_verts, static_cast<int>(idx_data.size()));
	
	return mesh;
}
DrawMesh*
MeshUtils::
buildConeWireSimple(int slices)
{
	const int slices0 = 4;
	const int num_verts = 2 * slices + 2 * slices0;

	std::vector<float> pos_data(num_verts * 3);
	std::vector<float> norm_data(num_verts * 3);
	std::vector<float> coord_data(num_verts * 2);
	std::vector<int> idx_data(num_verts);

	for (int i = 0; i < slices; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices;
		double theta1 = ((i + 1) * 2 * M_PI) / slices;

		double x0 = std::cos(theta0);
		double z0 = std::sin(-theta0);
		double u0 = static_cast<float>(i) / slices;

		double x1 = std::cos(theta1);
		double z1 = std::sin(-theta1);
		double u1 = static_cast<float>(i + 1) / slices;

		Eigen::Vector3d n0 = Eigen::Vector3d(0, -1, 0).normalized();

		int pos_offset = i * 2 * 3;
		int norm_offset = i * 2 * 3;
		int coord_offset = i * 2 * 2;

		pos_data[pos_offset] = x0;
		pos_data[pos_offset + 1] = 0;
		pos_data[pos_offset + 2] = z0;
		pos_data[pos_offset + 3] = x1;
		pos_data[pos_offset + 4] = 0;
		pos_data[pos_offset + 5] = z1;

		norm_data[norm_offset] = n0[0];
		norm_data[norm_offset + 1] = n0[1];
		norm_data[norm_offset + 2] = n0[2];
		norm_data[norm_offset + 3] = n0[0];
		norm_data[norm_offset + 4] = n0[1];
		norm_data[norm_offset + 5] = n0[2];

		coord_data[coord_offset] = u0;
		coord_data[coord_offset + 1] = 0.f;
		coord_data[coord_offset + 2] = u1;
		coord_data[coord_offset + 3] = 0.f;
	}

	for (int i = 0; i < slices0; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices0;

		double x0 = std::cos(theta0);
		double z0 = std::sin(-theta0);
		double u0 = static_cast<float>(i) / slices0;

		Eigen::Vector3d n0 = Eigen::Vector3d(std::cos(theta0), 1.0, -std::sin(theta0)).normalized();

		int pos_offset = (slices * 2 + i * 2) * 3;
		int norm_offset = (slices * 2 + i * 2) * 3;
		int coord_offset = (slices * 2 + i * 2) * 2;

		pos_data[pos_offset] = x0;
		pos_data[pos_offset + 1] = 0;
		pos_data[pos_offset + 2] = z0;
		pos_data[pos_offset + 3] = 0;
		pos_data[pos_offset + 4] = 1.0;
		pos_data[pos_offset + 5] = 0;

		norm_data[norm_offset] = n0[0];
		norm_data[norm_offset + 1] = n0[1];
		norm_data[norm_offset + 2] = n0[2];
		norm_data[norm_offset + 3] = n0[0];
		norm_data[norm_offset + 4] = n0[1];
		norm_data[norm_offset + 5] = n0[2];

		coord_data[coord_offset] = u0;
		coord_data[coord_offset + 1] = 0.f;
		coord_data[coord_offset + 2] = u0;
		coord_data[coord_offset + 3] = 1.f;
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(pos_data.data(),
		norm_data.data(),
		coord_data.data(),
		idx_data.data(),num_verts, static_cast<int>(idx_data.size()));
	
	return mesh;
}
DrawMesh*
MeshUtils::
buildCapsuleMesh(int stacks, int slices)
{
	stacks = std::max(stacks / 2, 1) * 2;
	int num_triangles = 2 * slices * (stacks - 1) + 2 * slices;

	std::vector<float> pos_data;
	std::vector<float> norm_data;
	std::vector<float> coord_data;
	std::vector<int> idx_data;
	pos_data.reserve(num_triangles * 3 * 3);
	norm_data.reserve(num_triangles * 3 * 3);
	coord_data.reserve(num_triangles * 3 * 2);
	idx_data.resize(num_triangles * 3);

	for (int i = 0; i < stacks; ++i)
	{
		float rho0 = 0.5 * M_PI - (i * M_PI) / stacks;
		float rho1 = 0.5 * M_PI - ((i + 1) * M_PI) / stacks;
		float coord_v0 = 1 - static_cast<float>(i) / stacks;
		float coord_v1 = 1 - static_cast<float>(i + 1) / stacks;

		float y0 = std::sin(rho0);
		float r0 = std::cos(rho0);
		float y1 = std::sin(rho1);
		float r1 = std::cos(rho1);
		float y2 = y1;
		float y3 = y0;

		float h_offset = -0.5 * ((i < stacks / 2) ? - 1.0 : 1.0);
		for (int j = 0; j < slices; ++j)
		{
			float theta0 = (j * 2 * M_PI) / slices;
			float theta1 = ((j + 1) * 2 * M_PI) / slices;
			float coord_u0 = static_cast<float>(j) / slices;
			float coord_u1 = static_cast<float>(j + 1) / slices;

			float x0 = r0 * std::cos(theta0);
			float z0 = r0 * std::sin(-theta0);
			float u0 = coord_u0;
			float v0 = coord_v0;
			float x1 = r1 * std::cos(theta0);
			float z1 = r1 * std::sin(-theta0);
			float u1 = coord_u0;
			float v1 = coord_v1;
			float x2 = r1 * std::cos(theta1);
			float z2 = r1 * std::sin(-theta1);
			float u2 = coord_u1;
			float v2 = coord_v1;
			float x3 = r0 * std::cos(theta1);
			float z3 = r0 * std::sin(-theta1);
			float u3 = coord_u1;
			float v3 = coord_v0;

			if (i != stacks - 1)
			{
				pos_data.emplace_back(x0);
				pos_data.emplace_back(y0 + h_offset);
				pos_data.emplace_back(z0);
				norm_data.emplace_back(x0);
				norm_data.emplace_back(y0);
				norm_data.emplace_back(z0);
				coord_data.emplace_back(u0);
				coord_data.emplace_back(v0);

				pos_data.emplace_back(x1);
				pos_data.emplace_back(y1 + h_offset);
				pos_data.emplace_back(z1);
				norm_data.emplace_back(x1);
				norm_data.emplace_back(y1);
				norm_data.emplace_back(z1);
				coord_data.emplace_back(u1);
				coord_data.emplace_back(v1);

				pos_data.emplace_back(x2);
				pos_data.emplace_back(y2 + h_offset);
				pos_data.emplace_back(z2);
				norm_data.emplace_back(x2);
				norm_data.emplace_back(y2);
				norm_data.emplace_back(z2);
				coord_data.emplace_back(u2);
				coord_data.emplace_back(v2);
			}

			if (i != 0)
			{
				pos_data.emplace_back(x2);
				pos_data.emplace_back(y2 + h_offset);
				pos_data.emplace_back(z2);
				norm_data.emplace_back(x2);
				norm_data.emplace_back(y2);
				norm_data.emplace_back(z2);
				coord_data.emplace_back(u2);
				coord_data.emplace_back(v2);

				pos_data.emplace_back(x3);
				pos_data.emplace_back(y3 + h_offset);
				pos_data.emplace_back(z3);
				norm_data.emplace_back(x3);
				norm_data.emplace_back(y3);
				norm_data.emplace_back(z3);
				coord_data.emplace_back(u3);
				coord_data.emplace_back(v3);

				pos_data.emplace_back(x0);
				pos_data.emplace_back(y0 + h_offset);
				pos_data.emplace_back(z0);
				norm_data.emplace_back(x0);
				norm_data.emplace_back(y0);
				norm_data.emplace_back(z0);
				coord_data.emplace_back(u0);
				coord_data.emplace_back(v0);
			}
		}
	}

	for (int i = 0; i < slices; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices;
		double theta1 = ((i + 1) * 2 * M_PI) / slices;

		double x0 = std::cos(theta0);
		double z0 = std::sin(-theta0);
		double u0 = static_cast<float>(i) / slices;

		double x1 = std::cos(theta1);
		double z1 = std::sin(-theta1);
		double u1 = static_cast<float>(i + 1) / slices;

		Eigen::Vector3d n0 = Eigen::Vector3d(x0, 0, z0).normalized();
		Eigen::Vector3d n1 = Eigen::Vector3d(x1, 0, z1).normalized();

		pos_data.emplace_back(x0);
		pos_data.emplace_back(-0.5);
		pos_data.emplace_back(z0);
		pos_data.emplace_back(x1);
		pos_data.emplace_back(-0.5);
		pos_data.emplace_back(z1);
		pos_data.emplace_back(x1);
		pos_data.emplace_back(0.5);
		pos_data.emplace_back(z1);
		pos_data.emplace_back(x1);
		pos_data.emplace_back(0.5);
		pos_data.emplace_back(z1);
		pos_data.emplace_back(x0);
		pos_data.emplace_back(0.5);
		pos_data.emplace_back(z0);
		pos_data.emplace_back(x0);
		pos_data.emplace_back(-0.5);
		pos_data.emplace_back(z0);

		norm_data.emplace_back(n0[0]);
		norm_data.emplace_back(n0[1]);
		norm_data.emplace_back(n0[2]);
		norm_data.emplace_back(n1[0]);
		norm_data.emplace_back(n1[1]);
		norm_data.emplace_back(n1[2]);
		norm_data.emplace_back(n1[0]);
		norm_data.emplace_back(n1[1]);
		norm_data.emplace_back(n1[2]);
		norm_data.emplace_back(n1[0]);
		norm_data.emplace_back(n1[1]);
		norm_data.emplace_back(n1[2]);
		norm_data.emplace_back(n0[0]);
		norm_data.emplace_back(n0[1]);
		norm_data.emplace_back(n0[2]);
		norm_data.emplace_back(n0[0]);
		norm_data.emplace_back(n0[1]);
		norm_data.emplace_back(n0[2]);

		coord_data.emplace_back(u0);
		coord_data.emplace_back(0.f);
		coord_data.emplace_back(u1);
		coord_data.emplace_back(0.f);
		coord_data.emplace_back(u1);
		coord_data.emplace_back(1);
		coord_data.emplace_back(u1);
		coord_data.emplace_back(1.f);
		coord_data.emplace_back(u0);
		coord_data.emplace_back(1.f);
		coord_data.emplace_back(u0);
		coord_data.emplace_back(0.f);
	}

	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}


	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(pos_data.data(),
		norm_data.data(),
		coord_data.data(),
		idx_data.data(),num_triangles * 3, static_cast<int>(idx_data.size()));
	
	return mesh;
}
DrawMesh*
MeshUtils::
buildCapsuleWireSimpleMesh(int stacks, int slices)
{
	const float r = 1.0;
	const float h = 1.0;
	int slices0 = 4;
	stacks = std::max(stacks / 2, 1) * 2;
	int num_lines = slices0 * stacks + 2 * slices + slices0;

	std::vector<float> pos_data;
	std::vector<float> norm_data;
	std::vector<float> coord_data;
	std::vector<int> idx_data;
	pos_data.reserve(num_lines * 2 * 3);
	norm_data.reserve(num_lines * 2 * 3);
	coord_data.reserve(num_lines * 2 * 3);
	idx_data.resize(num_lines * 2);

	for (int i = 0; i < stacks; ++i)
	{
		float rho0 = 0.5 * M_PI - (i * M_PI) / stacks;
		float rho1 = 0.5 * M_PI - ((i + 1) * M_PI) / stacks;
		float coord_v0 = 1 - static_cast<float>(i) / stacks;
		float coord_v1 = 1 - static_cast<float>(i + 1) / stacks;

		float y0 = r * std::sin(rho0);
		float r0 = r * std::cos(rho0);
		float y1 = r * std::sin(rho1);
		float r1 = r * std::cos(rho1);

		float h_offset = -0.5 * ((i < stacks / 2) ? -h : h);

		for (int j = 0; j < slices0; ++j)
		{
			float theta0 = (j * 2 * M_PI) / slices0;
			float theta1 = ((j + 1) * 2 * M_PI) / slices0;
			float coord_u0 = static_cast<float>(j) / slices0;

			float x0 = r0 * std::cos(theta0);
			float z0 = r0 * std::sin(-theta0);
			float u0 = coord_u0;
			float v0 = coord_v0;
			float x1 = r1 * std::cos(theta0);
			float z1 = r1 * std::sin(-theta0);
			float u1 = coord_u0;
			float v1 = coord_v1;

			pos_data.emplace_back(x0);
			pos_data.emplace_back(y0 + h_offset);
			pos_data.emplace_back(z0);
			norm_data.emplace_back(x0);
			norm_data.emplace_back(y0);
			norm_data.emplace_back(z0);
			coord_data.emplace_back(u0);
			coord_data.emplace_back(v0);

			pos_data.emplace_back(x1);
			pos_data.emplace_back(y1 + h_offset);
			pos_data.emplace_back(z1);
			norm_data.emplace_back(x1);
			norm_data.emplace_back(y1);
			norm_data.emplace_back(z1);
			coord_data.emplace_back(u1);
			coord_data.emplace_back(v1);
		}
	}

	for (int j = 0; j < slices; ++j)
	{
		float theta0 = (j * 2 * M_PI) / slices;
		float theta1 = ((j + 1) * 2 * M_PI) / slices;
		float coord_u0 = static_cast<float>(j) / slices;
		float coord_v0 = 0.5;
		float coord_v1 = 0.5;

		float x0 = r * std::cos(theta0);
		float y0 = 0;
		float z0 = r * std::sin(-theta0);
		float u0 = coord_u0;
		float v0 = coord_v0;
		float x1 = r * std::cos(theta1);
		float y1 = 0;
		float z1 = r * std::sin(-theta1);
		float u1 = coord_u0;
		float v1 = coord_v1;

		pos_data.emplace_back(x0);
		pos_data.emplace_back(y0 - 0.5 * h);
		pos_data.emplace_back(z0);
		pos_data.emplace_back(x1);
		pos_data.emplace_back(y1 - 0.5 * h);
		pos_data.emplace_back(z1);
		pos_data.emplace_back(x0);
		pos_data.emplace_back(y0 + 0.5 * h);
		pos_data.emplace_back(z0);
		pos_data.emplace_back(x1);
		pos_data.emplace_back(y1 + 0.5 * h);
		pos_data.emplace_back(z1);
		
		norm_data.emplace_back(x0);
		norm_data.emplace_back(y0);
		norm_data.emplace_back(z0);
		norm_data.emplace_back(x1);
		norm_data.emplace_back(y1);
		norm_data.emplace_back(z1);
		norm_data.emplace_back(x0);
		norm_data.emplace_back(y0);
		norm_data.emplace_back(z0);
		norm_data.emplace_back(x1);
		norm_data.emplace_back(y1);
		norm_data.emplace_back(z1);
		
		coord_data.emplace_back(u0);
		coord_data.emplace_back(v0);
		coord_data.emplace_back(u1);
		coord_data.emplace_back(v1);
		coord_data.emplace_back(u0);
		coord_data.emplace_back(v0);
		coord_data.emplace_back(u1);
		coord_data.emplace_back(v1);
	}

	for (int i = 0; i < slices0; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices0;
		double x0 = r * std::cos(theta0);
		double z0 = r * std::sin(-theta0);
		double u0 = static_cast<float>(i) / slices0;

		Eigen::Vector3d n0 = Eigen::Vector3d(x0, 0, z0).normalized();

		pos_data.emplace_back(x0);
		pos_data.emplace_back(-0.5 * h);
		pos_data.emplace_back(z0);
		pos_data.emplace_back(x0);
		pos_data.emplace_back(0.5 * h);
		pos_data.emplace_back(z0);

		norm_data.emplace_back(n0[0]);
		norm_data.emplace_back(n0[1]);
		norm_data.emplace_back(n0[2]);
		norm_data.emplace_back(n0[0]);
		norm_data.emplace_back(n0[1]);
		norm_data.emplace_back(n0[2]);

		coord_data.emplace_back(u0);
		coord_data.emplace_back(0.f);
		coord_data.emplace_back(u0);
		coord_data.emplace_back(0.f);
	}
	
	for (int i = 0; i < static_cast<int>(idx_data.size()); ++i)
	{
		idx_data[i] = i;
	}

	DrawMesh* mesh = new DrawMesh();

	mesh->createVBO(pos_data.data(),
		norm_data.data(),
		coord_data.data(),
		idx_data.data(),num_lines * 2, static_cast<int>(idx_data.size()));
	
	return mesh;

}
GLuint
MeshUtils::
buildTexture(const char* filename)
{
	std::vector<unsigned char> image;
	unsigned width, height;
	unsigned error = lodepng::decode(image, width, height, filename);

	size_t u2 = 1; while(u2 < width) u2 *= 2;
	size_t v2 = 1; while(v2 < height) v2 *= 2;
	// Ratio for power of two version compared to actual version, to render the non power of two image with proper size.
	double u3 = (double)width / u2;
	double v3 = (double)height / v2;

	// Make power of two version of the image.
	std::vector<unsigned char> image2(u2 * v2 * 4);
	for(size_t y = 0; y < height; y++)
	for(size_t x = 0; x < width; x++)
	for(size_t c = 0; c < 4; c++) {
		image2[4 * u2 * y + 4 * x + c] = image[4 * width * y + 4 * x + c];
	}


	GLuint id;
	glGenTextures(1, &id);
	glBindTexture(GL_TEXTURE_2D, id);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, 4, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, &image[0]);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	// glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 8, GL_RGB, 2048, 2048, GL_TRUE);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

	glBindTexture(GL_TEXTURE_2D, 0);
	return id;
}


void
DrawUtils::
drawRect(const Eigen::Vector3d& pos, const Eigen::Vector3d& size, eDrawMode draw_mode)
{
	Eigen::Vector3d a = Eigen::Vector3d(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2]);
	Eigen::Vector3d b = Eigen::Vector3d(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2]);
	Eigen::Vector3d c = Eigen::Vector3d(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2]);
	Eigen::Vector3d d = Eigen::Vector3d(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2]);

	drawQuad(a, b, c, d, draw_mode);
}
void
DrawUtils::
drawBox(const Eigen::Vector3d& pos, const Eigen::Vector3d& size, eDrawMode draw_mode)
{
	if (draw_mode == eDrawWire || draw_mode == eDrawWireSimple)
		drawBoxWire(pos, size);
	else if (draw_mode == eDrawSolid)
		drawBoxSolid(pos, size);
}
void
DrawUtils::
drawTriangle(const Eigen::Vector3d& pos, double side_len, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLES : GL_LINE_LOOP;
	glPushMatrix();
	translate(pos);
	scale(Eigen::Vector3d::Constant(side_len));
	triangle->draw(gl_mode);
	glPopMatrix();
}
void
DrawUtils::
drawQuad(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, const Eigen::Vector3d& d, eDrawMode draw_mode)
{
	Eigen::Vector3d n0;
	n0 = (b-a).cross(d-a);
	double normal_len = n0.norm();
	if (normal_len == 0)
		n0 = Eigen::Vector3d(0, 0, 1);
	else
		n0 = (n0 / normal_len);

	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLE_FAN : GL_LINE_LOOP;
	glBegin(gl_mode);
	glNormal3d(n0[0],n0[1],n0[2]);
	glTexCoord2d(0.0,0.0);
	glVertex3d(a[0],a[1],a[2]);
	glTexCoord2d(1.0,0.0);
	glVertex3d(b[0],b[1],b[2]);
	glTexCoord2d(1.0,1.0);
	glVertex3d(c[0],c[1],c[2]);
	glTexCoord2d(0.0,1.0);
	glVertex3d(d[0],d[1],d[2]);
	glEnd();
}
void
DrawUtils::
drawGround(double y, double size, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLES : GL_LINE_LOOP;

	DrawUtils::enableTexture(ground_texture_id);
	glPushMatrix();
	translate(y*Eigen::Vector3d::UnitY());
	scale(Eigen::Vector3d::Constant(size));
	tile->draw(gl_mode);
	glPopMatrix();
	DrawUtils::disableTexture();
}
void
DrawUtils::
drawDisk(const Eigen::Vector3d& pos, double r, eDrawMode draw_mode)
{
	drawDisk(pos, Eigen::Vector3d(r, r, r), draw_mode);
}
void
DrawUtils::
drawDisk(const Eigen::Vector3d& pos, const Eigen::Vector3d& r, eDrawMode draw_mode)
{
	glPushMatrix();
	translate(pos);
	drawDisk(r, draw_mode);
	glPopMatrix();
}
void
DrawUtils::
drawDisk(double r, eDrawMode draw_mode)
{
	drawDisk(Eigen::Vector3d(r, r, r), draw_mode);
}
void
DrawUtils::
drawDisk(const Eigen::Vector3d& r, eDrawMode draw_mode)
{
	glPushMatrix();
	scale(r);

	if (draw_mode == eDrawWireSimple || draw_mode == eDrawWire)
	{
		disk->draw(GL_LINE_STRIP, 1);
	}
	else if (draw_mode == eDrawSolid)
	{
		disk->draw(GL_TRIANGLE_FAN);
	}
	glPopMatrix();
}
void
DrawUtils::
drawPoint(const Eigen::Vector3d& pt)
{
	glPushMatrix();
	translate(pt);
	point->draw(GL_POINTS);
	glPopMatrix();
}
void
DrawUtils::
drawLine(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
	glBegin(GL_LINES);
	glVertex3d(a[0], a[1], a[2]);
	glVertex3d(b[0], b[1], b[2]);
	glEnd();
}
void
DrawUtils::
drawLineStrip(const std::vector<Eigen::Vector3d>& pts)
{
	int num_pts = static_cast<int>(pts.size());
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < num_pts - 1; ++i)
	{
		const Eigen::Vector3d& a = pts[i];
		const Eigen::Vector3d& b = pts[i + 1];
		glVertex3d(a[0], a[1], a[2]);
		glVertex3d(b[0], b[1], b[2]);
	}
	glEnd();
}
void
DrawUtils::
drawStrip(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double width, eDrawMode draw_mode)
{
	Eigen::Vector3d delta = b - a;
	Eigen::Vector3d orthogonal = Eigen::Vector3d(-delta[1], delta[0], 0);
	orthogonal.normalize();

	Eigen::Vector3d v0 = a - width * 0.5 * orthogonal;
	Eigen::Vector3d v1 = b - width * 0.5 * orthogonal;
	Eigen::Vector3d v2 = b + width * 0.5 * orthogonal;
	Eigen::Vector3d v3 = a + width * 0.5 * orthogonal;

	drawQuad(v0, v1, v2, v3, draw_mode);
}
void
DrawUtils::
drawCross(const Eigen::Vector3d& pos, double size)
{
	drawLine(Eigen::Vector3d(pos[0] - 0.5 * size, pos[1], pos[2]),
		Eigen::Vector3d(pos[0] + 0.5 * size, pos[1], pos[2]));
	drawLine(Eigen::Vector3d(pos[0], pos[1] - 0.5 * size, pos[2]),
		Eigen::Vector3d(pos[0], pos[1] + 0.5 * size, pos[2]));
	drawLine(Eigen::Vector3d(pos[0], pos[1], pos[2] - 0.5 * size),
		Eigen::Vector3d(pos[0], pos[1], pos[2] + 0.5 * size));
}

void
DrawUtils::
drawSphere(double r, eDrawMode draw_mode)
{
	glPushMatrix();
	scale(Eigen::Vector3d(r,r,r));
	
	if (draw_mode == eDrawSolid)
		sphere->draw(GL_TRIANGLES);
	else if (draw_mode == eDrawWire)
		sphere->draw(GL_LINES);
	else if (draw_mode == eDrawWireSimple)
		sphere_wire_simple->draw(GL_LINES);
	glPopMatrix();
}
void
DrawUtils::
drawCircleArrow(double width, double phi_start, double phi_end)
{
	int stride = 64;
	double dphi = 2*M_PI/(double)stride;
	glBegin(GL_QUADS);
	for(double phi = phi_start;phi<=phi_end;phi+=dphi)
	{
		double sr0 = std::sin(phi);
		double sr1 = std::sin(phi+dphi);
		double cr0 = std::cos(phi);
		double cr1 = std::cos(phi+dphi);

		double sR0 = (1.0-width)*std::sin(phi);
		double sR1 = (1.0-width)*std::sin(phi+dphi);
		double cR0 = (1.0-width)*std::cos(phi);
		double cR1 = (1.0-width)*std::cos(phi+dphi);
		glVertex3f(cr0,0.0,sr0);
		glVertex3f(cr1,0.0,sr1);
		glVertex3f(cR1,0.0,sR1);
		glVertex3f(cR0,0.0,sR0);
	}
	glEnd();
	glPushMatrix();
	if(phi_start<0.0){
		glRotatef(-phi_start*180.0/M_PI,0.0,1.0,0.0);
		glBegin(GL_TRIANGLES);
		glVertex3f(1.0+width,0.0,0.0);
		glVertex3f(1.0-2*width,0.0,0.0);
		glVertex3f(1.0-width,0.0,-width*2.5);
		glEnd();
	}
	else{
		glRotatef(-phi_end*180.0/M_PI,0.0,1.0,0.0);
		glBegin(GL_TRIANGLES);
		glVertex3f(1.0+width,0.0,0.0);
		glVertex3f(1.0-2*width,0.0,0.0);
		glVertex3f(1.0-width,0.0,width*2.5);
		glEnd();
	}
		
	glPopMatrix();

}
void
DrawUtils::
drawCylinder(double r, double h, eDrawMode draw_mode)
{
	if (draw_mode == eDrawSolid || draw_mode == eDrawWire)
		drawCylinderSolidWire(r, h, draw_mode);
	else if (draw_mode == eDrawWireSimple)
		drawCylinderWireSimple(r, h);
}
void
DrawUtils::
drawCylinder2(double r0, double r1, double h, eDrawMode draw_mode)
{
	int slice = 16;

	double delta = 2*M_PI/(double)slice;
	for(int i = 0;i<slice;i++)
	{
		double s0 = std::sin(delta*i);
		double c0 = std::cos(delta*i);
		double s1 = std::sin(delta*(i+1));
		double c1 = std::cos(delta*(i+1));

		glBegin(GL_QUADS);
		glVertex3d(r0*c0,r0*s0,0.0);
		glVertex3d(r0*c1,r0*s1,0.0);
		glVertex3d(r1*c1,r1*s1,h);
		glVertex3d(r1*c0,r1*s0,h);
		glEnd();
	}
}
void
DrawUtils::
drawCone(double r, double h, eDrawMode draw_mode)
{
	if (draw_mode == eDrawSolid || draw_mode == eDrawWire)
		drawConeSolidWire(r, h, draw_mode);
	else if (draw_mode == eDrawWireSimple)
		drawConeWireSimple(r, h);
}
void
DrawUtils::
drawCapsule(double r, double h, eDrawMode draw_mode)
{
	glPushMatrix();

	drawCylinder(r, h, draw_mode);

	translate(Eigen::Vector3d(0, 0.5*h, 0));
	drawSphere(r, draw_mode);

	translate(Eigen::Vector3d(0, -h, 0));
	drawSphere(r, draw_mode);
	glPopMatrix();
}
void
DrawUtils::
drawArrow2D(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double head_size)
{
	GLboolean prev_enable;
	glGetBooleanv(GL_CULL_FACE, &prev_enable);
	glDisable(GL_CULL_FACE);

	Eigen::Vector3d dir = Eigen::Vector3d(0, 1, 0);
	double dir_len = 0;
	if (start != end)
	{
		dir = end - start;
		dir_len = dir.norm();
		dir /= dir_len;
	}

	dir[3] = 0;
	Eigen::Vector3d axis = Eigen::Vector3d(0, 0, 1);
	Eigen::Vector3d tangent = axis.cross(dir);
	tangent.normalize();

	const double width = head_size * 0.1854;
	Eigen::Vector3d body_end = end - dir * head_size;

	Eigen::Vector3d a = start - width * tangent;
	Eigen::Vector3d b = body_end - width * tangent;
	Eigen::Vector3d c = body_end + width * tangent;
	Eigen::Vector3d d = start + width * tangent;
	drawQuad(a, b, c, d);

	Eigen::Vector3d e0 = body_end - tangent * head_size * 0.5f;
	Eigen::Vector3d e1 = body_end + tangent * head_size * 0.5f;
	drawQuad(end, e1, e0, end);

	if (prev_enable)
		glEnable(GL_CULL_FACE);
}
void
DrawUtils::
drawArrow3D(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double head_size)
{
	Eigen::Vector3d dir = Eigen::Vector3d(0, 1, 0);
	double dir_len = 0;
	if (start != end)
	{
		dir = end - start;
		dir_len = dir.norm();
		dir /= dir_len;
	}

	Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 1, 0), dir);

	double body_len = dir_len - head_size;
	double body_radius = head_size * 0.1854;
	double head_len = head_size;
	double head_radius = 0.5 * head_size;

	glPushMatrix();

	translate(start);
	rotate(q);
	translate(Eigen::Vector3d(0, body_len, 0));
	drawCone(head_radius, head_len, eDrawSolid);
	translate(Eigen::Vector3d(0, -0.5 * body_len, 0));
	drawCylinder(body_radius, body_len, eDrawSolid);

	glPopMatrix();
}



void
DrawUtils::
drawGrid2D(const Eigen::Vector3d& origin, const Eigen::Vector3d& size, double spacing, double line_width)
{
	double w = size[0];
	double h = size[1];

	double min_x = origin(0) - w;
	double min_y = origin(1) - h;
	double max_x = origin(0) + w;
	double max_y = origin(1) + h;

	const double offset_z = origin[2];

	glLineWidth(line_width);
	glColor3f(188 / 255.f, 219 / 255.f, 242 / 255.f);

	for (double x = min_x - std::fmod(min_x, spacing); x < max_x; x += spacing)
	{
		Eigen::Vector3d a = Eigen::Vector3d(x, min_y, offset_z);
		Eigen::Vector3d b = Eigen::Vector3d(x, max_y, offset_z);
		drawLine(a, b);
	}

	for (double y = min_y - std::fmod(min_y, spacing); y < max_y; y += spacing)
	{
		Eigen::Vector3d a = Eigen::Vector3d(min_x, y, offset_z);
		Eigen::Vector3d b = Eigen::Vector3d(max_x, y, offset_z);
		drawLine(a, b);
	}
}
void
DrawUtils::
drawRuler2D(const Eigen::Vector3d& origin, const Eigen::Vector3d& size, const Eigen::Vector3d& col, double line_width,double marker_spacing, double marker_h, double marker_line_width)
{
	double w = size[0];
	double h = size[1];

	double min_x = origin(0) - w * 0.5;
	double max_x = origin(0) + w * 0.5;
	double max_y = origin(1) + h * 0.5;

	if (line_width > 0)
	{
		glTexCoord2d(0, 0);
		glLineWidth(line_width);
		glColor3f(col[0],col[1],col[2]);
		drawRect(origin, size, eDrawSolid);
		glColor3f(0, 0, 0);
		drawLine(Eigen::Vector3d(min_x, max_y, 0), Eigen::Vector3d(max_x, max_y, 0));
	}

	// draw markers
	if (marker_line_width > 0)
	{
		glColor3f(0, 0, 0);
		glLineWidth(marker_line_width);
		for (double x = min_x - std::fmod(min_x, marker_spacing); x < max_x; x += marker_spacing)
		{
			Eigen::Vector3d a = Eigen::Vector3d(x, max_y + marker_h * 0.5f, 0);
			Eigen::Vector3d b = Eigen::Vector3d(x, max_y - marker_h * 0.5f, 0);
			drawLine(a, b);
		}
	}
}
void
DrawUtils::
drawSemiCircle(const Eigen::Vector3d& pos, double r, int slices, double min_theta, double max_theta, eDrawMode draw_mode)
{
	double d_theta = (max_theta - min_theta) / (slices - 1);

	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLE_FAN : GL_LINE_LOOP;
	glBegin(gl_mode);

	glTexCoord2d(0, 0);
	glNormal3d(0, 0, 1);
	glVertex3d(pos[0], pos[1], pos[2]);
	for (int i = 0; i < slices; ++i)
	{
		double theta0 = i * d_theta + min_theta;
		double theta1 = i * d_theta + min_theta;

		double x0 = r * std::cos(theta0) + pos[0];
		double y0 = r * std::sin(theta0) + pos[1];
		double x1 = r * std::cos(theta1) + pos[0];
		double y1 = r * std::sin(theta1) + pos[1];;

		glVertex3d(x0, y0, pos[2]);
		glVertex3d(x1, y1, pos[2]);
	}
	glEnd();
}
void
DrawUtils::
drawCalibMarker(const Eigen::Vector3d& pos, double r, int slices,const Eigen::Vector3d& col0, const Eigen::Vector3d& col1, eDrawMode draw_mode)
{
	glColor3f(col0[0],col0[1],col0[2]);
	drawSemiCircle(pos, r, slices, 0, M_PI * 0.5, draw_mode);
	drawSemiCircle(pos, r, slices, M_PI, M_PI * 1.5, draw_mode);

	glColor3f(col1[0],col1[1],col1[2]);
	drawSemiCircle(pos, r, slices, M_PI * 0.5, M_PI, draw_mode);
	drawSemiCircle(pos, r, slices, M_PI * 1.5, M_PI * 2, draw_mode);
}
void
DrawUtils::
drawString2D(const char *str, const Eigen::Vector3d& pos, const Eigen::Vector3d& color)
{
	glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT); // lighting and color mask
	glDisable(GL_LIGHTING);     // need to disable lighting for proper text color
	glDisable(GL_TEXTURE_2D);

	glColor3f(color[0],color[1],color[2]);          // set text color
	glRasterPos2f(pos[0],pos[1]);        // place text position

	// loop all characters in the string
	while(*str)
	{
	    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *str);
	    ++str;
	}
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glPopAttrib();
}
void
DrawUtils::
drawString3D(const char *str, const Eigen::Vector3d& pos, const Eigen::Vector3d& color)
{
	glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT); // lighting and color mask
	glDisable(GL_LIGHTING);     // need to disable lighting for proper text color
	glDisable(GL_TEXTURE_2D);

	glColor3f(color[0],color[1],color[2]);          // set text color
	glRasterPos3f(pos[0],pos[1],pos[2]);        // place text position

	// loop all characters in the string
	while(*str)
	{
	    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *str);
	    ++str;
	}

	glEnable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glPopAttrib();
}
void
DrawUtils::
drawPlot(const BarPlot& bar_plot, const Eigen::Vector3d& pos, const Eigen::Vector3d& size)
{
	glLineWidth(1.0);
	glColor4f(1.0,1.0,1.0,0.5);
	drawRect(pos,size);
	
	const Eigen::VectorXd& vals = bar_plot.vals;
	double min_val = bar_plot.min_val;
	double max_val = bar_plot.max_val; 
	double base_val = bar_plot.base_val;
	const Eigen::Vector4d& color = bar_plot.color;

	int num_vals = static_cast<int>(vals.size());

	double norm_base_val = (base_val - min_val) / (max_val - min_val);
	glColor4f(color[0],color[1],color[2],color[3]);
	for (int i = 0; i < num_vals; ++i)
	{
		double curr_val = vals[i];
		double norm_curr_val = (curr_val - min_val) / (max_val - min_val);

		Eigen::Vector3d bar_min = Eigen::Vector3d::Zero();
		Eigen::Vector3d bar_max = Eigen::Vector3d::Zero();

		bar_min[0] = static_cast<double>(i) / num_vals;
		bar_max[0] = static_cast<double>(i + 1) / num_vals;

		if(curr_val < base_val){
			bar_min[1] = norm_curr_val;
			bar_max[1] = norm_base_val;
		}
		else
		{
			bar_min[1] = norm_base_val;
			bar_max[1] = norm_curr_val;
		}
		bar_min[1] = std::min(std::max(bar_min[1], 0.0), 1.0);
		bar_max[1] = std::min(std::max(bar_max[1], 0.0), 1.0);

		bar_min = size.cwiseProduct(bar_min) + pos - 0.5* size;
		bar_max = size.cwiseProduct(bar_max) + pos - 0.5 * size;
		drawRect(0.5 * (bar_max + bar_min), bar_max - bar_min);
	}
	double base_y = size[1] * norm_base_val + pos[1] - 0.5 * size[1];
	Eigen::Vector3d base0 = pos;
	Eigen::Vector3d base1 = pos;
	base0[0] += -0.5 * size[0];
	base1[0] += 0.5 * size[0];
	base0[1] = base_y;
	base1[1] = base_y;

	glColor4f(0, 0, 0, 1.0);
	drawLine(base0, base1);
	drawRect(pos, size, eDrawWireSimple);
}
void
DrawUtils::
drawLinePlot(const BarPlot& bar_plot, const Eigen::Vector3d& pos, const Eigen::Vector3d& size)
{
	glLineWidth(1.0);

	glColor4f(bar_plot.background_color[0],
		bar_plot.background_color[1],
		bar_plot.background_color[2],
		bar_plot.background_color[3]);
	drawRect(pos,size);
	
	const Eigen::VectorXd& vals = bar_plot.vals;
	double min_val = bar_plot.min_val;
	double max_val = bar_plot.max_val; 
	double base_val = bar_plot.base_val;
	const Eigen::Vector4d& color = bar_plot.color;

	int num_vals = static_cast<int>(vals.size());

	double norm_base_val = (base_val - min_val) / (max_val - min_val);
	glColor4f(color[0],color[1],color[2],color[3]);
	glLineWidth(2.0);
	for(int i=0;i<num_vals-1;++i)
	{
		double curr_val = vals[i];
		double next_val = vals[i+1];
		double norm_curr_val = (curr_val - min_val) / (max_val - min_val);
		double norm_next_val = (next_val - min_val) / (max_val - min_val);
		Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
		Eigen::Vector3d p1 = Eigen::Vector3d::Zero();

		p0[0] = static_cast<double>(i) / num_vals;
		p1[0] = static_cast<double>(i + 1) / num_vals;

		p0[1] = norm_curr_val;
		p1[1] = norm_next_val;

		p0[1] = std::min(std::max(p0[1], 0.0), 1.0);
		p1[1] = std::min(std::max(p1[1], 0.0), 1.0);

		p0 = size.cwiseProduct(p0) + pos - 0.5* size;
		p1 = size.cwiseProduct(p1) + pos - 0.5* size;
		drawLine(p0, p1);
	}
	glLineWidth(1.0);
	double base_y = size[1] * norm_base_val + pos[1] - 0.5 * size[1];
	Eigen::Vector3d base0 = pos;
	Eigen::Vector3d base1 = pos;
	base0[0] += -0.5 * size[0];
	base1[0] += 0.5 * size[0];
	base0[1] = base_y;
	base1[1] = base_y;

	glColor4f(0, 0, 0, 1.0);
	drawLine(base0, base1);
	drawRect(pos, size, eDrawWireSimple);
}
void
DrawUtils::
translate(const Eigen::Vector3d& trans)
{
	glTranslatef(trans[0],trans[1],trans[2]);
}
void
DrawUtils::
scale(double scale)
{
	glScalef(scale,scale,scale);
}
void
DrawUtils::
scale(const Eigen::Vector3d& scale)
{
	glScalef(scale[0],scale[1],scale[2]);
}
void
DrawUtils::
rotate(double theta, const Eigen::Vector3d& axis)
{
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	T.linear() = Eigen::AngleAxisd(theta, axis).toRotationMatrix();
	T.translation() = Eigen::Vector3d::Zero();
	transform(T);
}
void
DrawUtils::
rotate(const Eigen::Quaterniond& q)
{
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	T.linear() = q.toRotationMatrix();
	T.translation() = Eigen::Vector3d::Zero();
	
	transform(T);
}
void
DrawUtils::
transform(const Eigen::Isometry3d& T)
{
	glMultMatrixd(T.data());
}
void
DrawUtils::
buildMeshes()
{
	point = MeshUtils::buildPointMesh();
	line = MeshUtils::buildLineMesh();
	quad = MeshUtils::buildQuadMesh();
	tile = MeshUtils::buildTileMesh(50);
	box_solid = MeshUtils::buildBoxSolidMesh();
	box_wire = MeshUtils::buildBoxWireMesh();
	sphere = MeshUtils::buildSphereMesh(32,32);
	sphere_wire_simple = MeshUtils::buildSphereWireSimpleMesh(32,32);
	disk = MeshUtils::buildDiskMesh(32);
	triangle = MeshUtils::buildTriangleMesh();
	cylinder = MeshUtils::buildCylinder(32);
	cylinder_wire_simple = MeshUtils::buildCylinderWireSimple(32);
	cone = MeshUtils::buildCone(32);
	cone_wire_simple = MeshUtils::buildConeWireSimple(32);
	capsule = MeshUtils::buildCapsuleMesh(32,32);
	capsule_wire_simple	 = MeshUtils::buildCapsuleWireSimpleMesh(32,32);

	// object_texture_id = MeshUtils::buildTexture((std::string(ROOT_DIR)+"/data/object.png").c_str());
	ground_texture_id = MeshUtils::buildTexture((std::string(ROOT_DIR)+"/data/textures/ground.png").c_str());
	initialized = true;
}
void
DrawUtils::
enableTexture(GLuint texture_id)
{
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture_id);
}
void
DrawUtils::
disableTexture()
{
	glDisable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);
}

void
DrawUtils::
drawBoxSolid(const Eigen::Vector3d& pos, const Eigen::Vector3d& size)
{
	glPushMatrix();
	translate(pos);
	scale(size);
	box_solid->draw(GL_TRIANGLES);
	glPopMatrix();
}
void
DrawUtils::
drawBoxWire(const Eigen::Vector3d& pos, const Eigen::Vector3d& size)
{
	glPushMatrix();
	translate(pos);
	scale(size);
	box_wire->draw(GL_LINES);
	glPopMatrix();
}
void
DrawUtils::
drawCylinderSolidWire(double r, double h, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLES : GL_LINES;
	glPushMatrix();
	scale(Eigen::Vector3d(r, h, r));
	cylinder->draw(gl_mode);
	glPopMatrix();
}
void
DrawUtils::
drawCylinderWireSimple(double r, double h)
{
	glPushMatrix();
	scale(Eigen::Vector3d(r, h, r));
	cylinder_wire_simple->draw(GL_LINES);
	glPopMatrix();
}
void
DrawUtils::
drawConeSolidWire(double r, double h, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLES : GL_LINES;
	glPushMatrix();
	scale(Eigen::Vector3d(r, h, r));
	cone->draw(gl_mode);
	glPopMatrix();
}
void
DrawUtils::
drawConeWireSimple(double r, double h)
{
	glPushMatrix();
	scale(Eigen::Vector3d(r, h, r));
	cone_wire_simple->draw(GL_LINES);
	glPopMatrix();
}
#include <sstream>
#include <stdlib.h>
Eigen::Vector3d
DrawUtils::
stringToRGB(const std::string& rgb)
{
	int number = (int)strtol(rgb.c_str(), NULL, 16);
	int r = number/65536;
	number -= r*65536;
	int g = number/256;
	number -= g*256;
	int b = number;

	return Eigen::Vector3d(r/(double)255.0, g/(double)255.0, b/255.0);
}
// void
// DrawUtils::
// drawBox()
// {
	
// }