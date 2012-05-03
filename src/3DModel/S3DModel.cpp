#include "S3DModel.h"

S3DModel::S3DModel(int id)
{
	mId = id;
}

S3DModel::S3DModel(const Joint* jt)
{
	mRootJoint = new Joint(*jt);
}

S3DModel::~S3DModel()
{
	delete mRootJoint;
}


