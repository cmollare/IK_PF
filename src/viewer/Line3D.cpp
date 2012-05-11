#include "Line3D.h"

Line3D::Line3D(Ogre::String& name) : Ogre::ManualObject(name)
{
}

Line3D::~Line3D()
{
}

void Line3D::setLine(const Ogre::Vector3& start, const Ogre::Vector3& stop)
{
	clear();
	begin("Purple",Ogre::RenderOperation::OT_LINE_LIST);
		position(start[0], start[1], start[2]);
		position(stop[0], stop[1], stop[2]);
	end();
}
