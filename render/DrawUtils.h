#ifndef __DRAW_UTILS_H__
#define __DRAW_UTILS_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utility>
#include <vector>
#include <GL/glew.h>
#include <GL/glut.h>
struct DrawMesh
{
	GLfloat *vertices;
	GLfloat *normals;
	// GLfloat *colors;
	GLfloat *texcoords;

	GLushort *indices;

	int n,m;
	
	GLuint vboId, vboId2;

	void createVBO(const std::vector<Eigen::Vector3d>& _vertices,
					const std::vector<Eigen::Vector3d>& _normals,
					// const std::vector<Eigen::Vector3d>& _colors,
					const std::vector<Eigen::Vector2d>& _texcoords,
					const std::vector<int>& _indices);

	void createVBO(const float* _vertices, const float* _normals, const float* _texcoords,
					const int* _indices, int _n, int _m,GLenum _mode = GL_TRIANGLES);

	void bindVBO();
	void draw(GLenum mode);
	void draw(GLenum mode, int idx_start);
	void draw(GLenum mode, int idx_start, int idx_end);
	void unbindVBO();
};

class MeshUtils
{
public:
	static DrawMesh* buildPointMesh();
	static DrawMesh* buildLineMesh();
	static DrawMesh* buildQuadMesh();
	static DrawMesh* buildTileMesh(int num);
	static DrawMesh* buildBoxSolidMesh();
	static DrawMesh* buildBoxWireMesh();
	static DrawMesh* buildSphereMesh(int stacks, int slices);
	static DrawMesh* buildSphereWireSimpleMesh(int stacks, int slices);
	static DrawMesh* buildDiskMesh(int slices);
	static DrawMesh* buildTriangleMesh();
	static DrawMesh* buildCylinder(int slices);
	static DrawMesh* buildCylinderWireSimple(int slices);
	static DrawMesh* buildCone(int slices);
	static DrawMesh* buildConeWireSimple(int slices);
	static DrawMesh* buildCapsuleMesh(int stacks, int slices);
	static DrawMesh* buildCapsuleWireSimpleMesh(int stacks, int slices);

	static GLuint buildTexture(const char* filename);
};

class DrawUtils
{
public:
	enum eDrawMode
	{
		eDrawSolid,
		eDrawWire,
		eDrawWireSimple,
		eDrawMax
	};
	struct BarPlot
	{
		double min_val = 0.0;
		double max_val = 1.0;
		double base_val = 0.0;
		Eigen::VectorXd vals;
		Eigen::Vector4d color = Eigen::Vector4d::Zero();
		Eigen::Vector4d background_color = Eigen::Vector4d::Ones();
	};

	static void drawRect(const Eigen::Vector3d& pos, const Eigen::Vector3d& size, eDrawMode draw_mode = eDrawSolid);
	static void drawBox(const Eigen::Vector3d& pos, const Eigen::Vector3d& size, eDrawMode draw_mode = eDrawSolid);
	static void drawTriangle(const Eigen::Vector3d& pos, double side_len, eDrawMode draw_mode = eDrawSolid);
	static void drawQuad(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, const Eigen::Vector3d& d, eDrawMode draw_mode = eDrawSolid);
	static void drawGround(double y, double size, eDrawMode draw_mode = eDrawSolid);
	static void drawDisk(const Eigen::Vector3d& pos, double r, eDrawMode draw_mode = eDrawSolid);
	static void drawDisk(const Eigen::Vector3d& pos, const Eigen::Vector3d& r, eDrawMode draw_mode = eDrawSolid);
	static void drawDisk(double r, eDrawMode draw_mode = eDrawSolid);
	static void drawDisk(const Eigen::Vector3d& r, eDrawMode draw_mode = eDrawSolid);
	static void drawPoint(const Eigen::Vector3d& pt);
	static void drawLine(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
	static void drawLineStrip(const std::vector<Eigen::Vector3d>& pts);
	static void drawStrip(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double width, eDrawMode draw_mode = eDrawSolid);
	static void drawCross(const Eigen::Vector3d& pos, double size);
	static void drawSphere(double r, eDrawMode draw_mode = eDrawSolid);
	static void drawCylinder(double r, double h, eDrawMode draw_mode = eDrawSolid);
	static void drawCylinder2(double r0, double r1, double h, eDrawMode draw_mode = eDrawSolid);
	
	static void drawCone(double r, double h, eDrawMode draw_mode = eDrawSolid);
	static void drawCapsule(double r, double h, eDrawMode draw_mode = eDrawSolid);
	static void drawArrow2D(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double head_size);
	static void drawArrow3D(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double head_size);
	static void drawGrid2D(const Eigen::Vector3d& origin, const Eigen::Vector3d& size, double spacing, double line_width);
	static void drawRuler2D(const Eigen::Vector3d& origin, const Eigen::Vector3d& size, const Eigen::Vector3d& col, double line_width,double marker_spacing, double marker_h, double marker_line_width);
	static void drawSemiCircle(const Eigen::Vector3d& pos, double r, int slices, double min_theta, double max_theta,eDrawMode draw_mode = eDrawSolid);
	static void drawCalibMarker(const Eigen::Vector3d& pos, double r, int slices,const Eigen::Vector3d& col0, const Eigen::Vector3d& col1, eDrawMode draw_mode = eDrawSolid);

	static void drawString2D(const char *str, const Eigen::Vector3d& pos, const Eigen::Vector3d& color);
	static void drawString3D(const char *str, const Eigen::Vector3d& pos, const Eigen::Vector3d& color);
	static void drawPlot(const BarPlot& bar_plot, const Eigen::Vector3d& pos, const Eigen::Vector3d& size);
	static void drawLinePlot(const BarPlot& bar_plot, const Eigen::Vector3d& pos, const Eigen::Vector3d& size);
	static void translate(const Eigen::Vector3d& trans);
	static void scale(const Eigen::Vector3d& scale);
	static void rotate(double theta, const Eigen::Vector3d& axis);
	static void rotate(const Eigen::Quaterniond& q);
	static void transform(const Eigen::Isometry3d& T);

	static void buildMeshes();

	static void enableTexture(GLuint texture_id);
	static void disableTexture();

	static bool initialized;
private:
	static DrawMesh* point;
	static DrawMesh* line;
	static DrawMesh* quad;
	static DrawMesh* tile;
	static DrawMesh* box_solid;
	static DrawMesh* box_wire;
	static DrawMesh* sphere;
	static DrawMesh* sphere_wire_simple;
	static DrawMesh* disk;
	static DrawMesh* triangle;
	static DrawMesh* cylinder;
	static DrawMesh* cylinder_wire_simple;
	static DrawMesh* cone;
	static DrawMesh* cone_wire_simple;
	static DrawMesh* capsule;
	static DrawMesh* capsule_wire_simple;

	static GLuint ground_texture_id;
	// static GLuint object_texture_id;
	static void drawBoxSolid(const Eigen::Vector3d& pos, const Eigen::Vector3d& size);
	static void drawBoxWire(const Eigen::Vector3d& pos, const Eigen::Vector3d& size);
	static void drawCylinderSolidWire(double r, double h, eDrawMode draw_mode = eDrawSolid);
	static void drawCylinderWireSimple(double r, double h);
	static void drawConeSolidWire(double r, double h, eDrawMode draw_mode = eDrawSolid);
	static void drawConeWireSimple(double r, double h);
};




#endif