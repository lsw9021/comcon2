#include "DARTRendering.h"
#include "DrawUtils.h"
using namespace dart;
using namespace dart::dynamics;

DARTRendering::Option DARTRendering::gRenderOption = DARTRendering::Option();
void
DARTRendering::
drawLinks(const SkeletonPtr& skel)
{

}
void
DARTRendering::
drawJoints(const SkeletonPtr& skel)
{

}

void
DARTRendering::
drawSkeleton(const SkeletonPtr& skel,const Option& option)
{
	if(gRenderOption.texture_id == 0)
		gRenderOption.texture_id = MeshUtils::buildTexture((std::string(ROOT_DIR)+"/data/object.png").c_str());
	DrawUtils::enableTexture(option.texture_id);
	
	glLineWidth(option.line_width);
	if(option.drawLinks)
	for(int i=0;i<skel->getNumBodyNodes();i++)
	{
		auto bn = skel->getBodyNode(i);
		auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();

		auto T = shapeNodes.back()->getTransform();
		// if(T.translation()[1]>1.0)
			
		drawShape(T,shapeNodes.back()->getShape().get(), option.draw_mode);
	}
	// if(option.drawJoints)
	for(int i =0;i<skel->getNumJoints();i++)
	{
		auto parent = skel->getJoint(i)->getParentBodyNode();
		auto child = skel->getJoint(i)->getChildBodyNode();
		auto shapeNodes = child->getShapeNodesWith<VisualAspect>();
		auto shape = shapeNodes.back()->getShape().get();
		double volume = std::cbrt(shape->getVolume())*0.3;
		if(skel->getJoint(i)->getType()=="FreeJoint")
			continue;
		else if(skel->getJoint(i)->getType()=="BallJoint")
			glColor3f(0.8,0.2,0.2);
		else if(skel->getJoint(i)->getType()=="RevoluteJoint")
			glColor3f(0.2,0.8,0.2);
		Eigen::Isometry3d T;
		T.setIdentity();
		if(parent!=nullptr)
			T = parent->getTransform();

		T = T*skel->getJoint(i)->getTransformFromParentBodyNode();
		glPushMatrix();
		glMultMatrixd(T.data());
		glColor3f(0.8,0.8,0.8);
		glDisable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);
		if(option.draw_mode != DrawUtils::eDrawWireSimple)
			DrawUtils::drawSphere(volume, option.draw_mode);
		glColor3f(0.0,0.0,0.0);
		DrawUtils::drawSphere(volume*1.05,DrawUtils::eDrawWireSimple);
		glEnable(GL_TEXTURE_2D);
		glEnable(GL_LIGHTING);
		glPopMatrix();
    	
	}
	DrawUtils::disableTexture();
}

void
DARTRendering::
drawShape(const Eigen::Isometry3d& T,
	const dart::dynamics::Shape* shape,
	DrawUtils::eDrawMode draw_mode)
{
	glEnable(GL_LIGHTING);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glPushMatrix();
	glMultMatrixd(T.data());
	if(shape->is<SphereShape>())
	{
		const auto* sphere = dynamic_cast<const SphereShape*>(shape);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		
		if(draw_mode != DrawUtils::eDrawWireSimple)
			DrawUtils::drawSphere(sphere->getRadius(), draw_mode);
		glDisable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);
		glColor3f(0.0,0.0,0.0);
		DrawUtils::drawSphere(sphere->getRadius()*1.01,draw_mode);
		glEnable(GL_TEXTURE_2D);
		glEnable(GL_LIGHTING);
	}
	else if (shape->is<BoxShape>())
	{
		const auto* box = dynamic_cast<const BoxShape*>(shape);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    	DrawUtils::drawBox(Eigen::Vector3d::Zero(), box->getSize(), draw_mode);
	}

	glPopMatrix();
}