#include "Joint.h"

Joint::Joint(string name, Joint *parent)
{
	mParentJoint = parent;
	mName = name;
	
	mCurrent << 0,0,0,0,
				0,0,0,0,
				0,0,0,0,
				0,0,0,0;
				
	if (mParentJoint == NULL)
	{
		mParent << 0,0,0,0,
					0,0,0,0,
					0,0,0,0,
					0,0,0,0;
	}
	
}

Joint::~Joint()
{
}

Eigen::Matrix4f Joint::getOrientation()
{
	return mCurrent;
}
