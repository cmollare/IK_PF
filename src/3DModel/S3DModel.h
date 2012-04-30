#ifndef S3DMODEL_H
#define S3DMODEL_H

#include <iostream>
#include "Joint.h"

class S3DModel
{
	public:
		S3DModel(int id);
		~S3DModel();
		
	private:
		int mId;
		Joint *mRootJoint;
		
};

#endif
