#ifndef S3DMODEL_H
#define S3DMODEL_H

#include <iostream>
#include "Joint.h"

class S3DModel
{
	public:
		S3DModel(int id);
		S3DModel(const Joint* jt);
		S3DModel(const S3DModel& model);
		~S3DModel();
		
	private:
		int mId;
		Joint *mRootJoint;
		
};

#endif
