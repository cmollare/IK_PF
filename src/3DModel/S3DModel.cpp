#include "S3DModel.h"

S3DModel::S3DModel(int id)
{
	mId = id;
}

S3DModel::S3DModel(const Joint* jt, unsigned int id)
{
	mRootJoint = new Joint(*jt);
	mId = id;
}

S3DModel::~S3DModel()
{
	delete mRootJoint;
}

Joint* S3DModel::getRootJoint()
{
	return mRootJoint;
}


