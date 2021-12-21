#include "DARTRendering.h"
#include "DrawUtils.h"
using namespace dart;
using namespace dart::dynamics;

DARTRendering::Option DARTRendering::gRenderOption = DARTRendering::Option();
bool DARTRendering::mEE = true;
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
	if(skel == nullptr)
		return;
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
		
		mEE = false;
		if(bn->getNumChildBodyNodes()==0)
			mEE = true;
		drawShape(T,shapeNodes.back()->getShape().get(), option.draw_mode);
		if(bn->getName()=="Spine")
		{

			Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
			offset.translation()[1] = 0.14;
			T = T*offset;

			glEnable(GL_LIGHTING);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glEnable(GL_COLOR_MATERIAL);
			glPushMatrix();
			glMultMatrixd(T.data());
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
			double r = 0.05;
			if(option.draw_mode != DrawUtils::eDrawWireSimple){
				glDisable(GL_TEXTURE_2D);
				Eigen::Vector3d color = DrawUtils::stringToRGB("8BC6D0");
				color *= 1.6;
				glColor3f(color[0],color[1],color[2]);
				DrawUtils::drawSphere(r, option.draw_mode);
			}
			glDisable(GL_LIGHTING);
			glDisable(GL_TEXTURE_2D);
			glColor3f(0.0,0.0,0.0);
			DrawUtils::drawSphere(r*1.01,DrawUtils::eDrawWireSimple);
			glEnable(GL_TEXTURE_2D);
			glEnable(GL_LIGHTING);

			glPopMatrix();
		}
		else if(bn->getName()=="Spine1")
		{
			Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
			offset.translation()[1] = 0.11;
			offset.translation()[0] = 0.07;
			
			Eigen::Isometry3d T0 = T*offset;
			offset.translation()[0] *= -1;
			Eigen::Isometry3d T1 = T*offset;


			glEnable(GL_LIGHTING);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glEnable(GL_COLOR_MATERIAL);
			glPushMatrix();
			glMultMatrixd(T0.data());
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
			Eigen::Vector3d bsize(0.09,0.045,0.045);
			if(option.draw_mode != DrawUtils::eDrawWireSimple){
				glDisable(GL_TEXTURE_2D);
				Eigen::Vector3d color = DrawUtils::stringToRGB("8BC6D0");
				color *= 1.6;
				glColor3f(color[0],color[1],color[2]);

				DrawUtils::drawBox(Eigen::Vector3d::Zero(), bsize, option.draw_mode);
			}
			glDisable(GL_LIGHTING);
			glDisable(GL_TEXTURE_2D);
			glColor3f(0.0,0.0,0.0);
			DrawUtils::drawBox(Eigen::Vector3d::Zero(), bsize, DrawUtils::eDrawWireSimple);

			glEnable(GL_TEXTURE_2D);
			glEnable(GL_LIGHTING);

			glPopMatrix();

			glEnable(GL_LIGHTING);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glEnable(GL_COLOR_MATERIAL);
			glPushMatrix();
			glMultMatrixd(T1.data());
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
			if(option.draw_mode != DrawUtils::eDrawWireSimple){
				glDisable(GL_TEXTURE_2D);
				Eigen::Vector3d color = DrawUtils::stringToRGB("8BC6D0");
				color *= 1.6;
				glColor3f(color[0],color[1],color[2]);

				DrawUtils::drawBox(Eigen::Vector3d::Zero(), bsize, option.draw_mode);
			}
			glDisable(GL_LIGHTING);
			glDisable(GL_TEXTURE_2D);
			glColor3f(0.0,0.0,0.0);
			DrawUtils::drawBox(Eigen::Vector3d::Zero(), bsize, DrawUtils::eDrawWireSimple);

			glEnable(GL_TEXTURE_2D);
			glEnable(GL_LIGHTING);

			glPopMatrix();
		}
	}
	// if(option.drawJoints)
	Eigen::Vector3d joint_color = DrawUtils::stringToRGB("E7B4BC");
	for(int i =0;i<skel->getNumJoints();i++)
	{
		auto parent = skel->getJoint(i)->getParentBodyNode();
		auto child = skel->getJoint(i)->getChildBodyNode();
		auto shapeNodes = child->getShapeNodesWith<VisualAspect>();
		auto shape = shapeNodes.back()->getShape().get();
		double volume = 0.02;
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
		glColor3f(joint_color[0],joint_color[1],joint_color[2]);
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
		
		if(draw_mode != DrawUtils::eDrawWireSimple){
			glDisable(GL_TEXTURE_2D);
			Eigen::Vector3d color = DrawUtils::stringToRGB("8BC6D0");
			Eigen::Vector3d color2 = DrawUtils::stringToRGB("86b1cc");
			color *= 1.4;
			color2 *= 1.4;
			color2[2] += 0.2;
			if(mEE)
				color=color2;
			glColor3f(color[0],color[1],color[2]);
			DrawUtils::drawSphere(sphere->getRadius(), draw_mode);
		}
		glDisable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);
		glColor3f(0.0,0.0,0.0);
		DrawUtils::drawSphere(sphere->getRadius()*1.01,DrawUtils::eDrawWireSimple);
		glEnable(GL_TEXTURE_2D);
		glEnable(GL_LIGHTING);
	}
	else if (shape->is<BoxShape>())
	{
		const auto* box = dynamic_cast<const BoxShape*>(shape);
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		if(draw_mode != DrawUtils::eDrawWireSimple){
			glDisable(GL_TEXTURE_2D);
			Eigen::Vector3d color = DrawUtils::stringToRGB("8BC6D0");
			Eigen::Vector3d color2 = DrawUtils::stringToRGB("86b1cc");
			color *= 1.4;
			color2 *= 1.4;
			color2[2] += 0.2;
			if(mEE)
				color=color2;
			glColor3f(color[0],color[1],color[2]);
			DrawUtils::drawBox(Eigen::Vector3d::Zero(), box->getSize(), draw_mode);
		}
		glDisable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);
		glColor3f(0.0,0.0,0.0);
		DrawUtils::drawBox(Eigen::Vector3d::Zero(), box->getSize(), DrawUtils::eDrawWireSimple);
		glEnable(GL_TEXTURE_2D);
		glEnable(GL_LIGHTING);
    	
	}

	glPopMatrix();
}