#include "Joint.h"

Joint::Joint(string name, Joint *parent)
{
	mParentJoint = parent;
	mName = name;

	mQDefault << 1,0,
				0,1;
	if (mParentJoint != NULL)
		mQDefault = mParentJoint->getCurrOrientation()*mQDefault;
}

Joint::~Joint()
{
}

Eigen::Matrix2f Joint::getCurrOrientation()
{
	return mQCurrent;
}
