#ifndef __DART_RENDERING_H__
#define __DART_RENDERING_H__
#include "dart/dart.hpp"
#include "DrawUtils.h"
#include "Character.h"
#include <GL/glew.h>
#include <GL/glut.h>
class DARTRendering
{
public:
	struct Option
	{
		Eigen::Vector3d color_body = Eigen::Vector3d(0.8,0.8,0.8);
		Eigen::Vector3d color_joint = Eigen::Vector3d(0.1,0.1,0.1);
		Eigen::Vector3d color_edge = Eigen::Vector3d::Zero();
		DrawUtils::eDrawMode draw_mode = DrawUtils::eDrawSolid;
		double line_width = 3.0;

		bool drawLinks = true;
		bool drawJoints = true;

		double sensor_radius = 0.015;
		GLuint texture_id = 0;
	};

	static Option gRenderOption;
	static void drawLinks(const dart::dynamics::SkeletonPtr& skel);
	static void drawJoints(const dart::dynamics::SkeletonPtr& skel);
	static void drawObstacle(const dart::dynamics::SkeletonPtr& skel,const Option& option = DARTRendering::gRenderOption);
	static void drawSkeleton(const dart::dynamics::SkeletonPtr& skel,const Option& option = DARTRendering::gRenderOption);


	static void drawShape(const Eigen::Isometry3d& T, const dart::dynamics::Shape* shape, DrawUtils::eDrawMode draw_mode);
private:
	static bool mEE;
};
#endif