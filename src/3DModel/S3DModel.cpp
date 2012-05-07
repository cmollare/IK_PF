#include "S3DModel.h"

S3DModel::S3DModel(int id)
{
	mId = id;
	mRootJoint = NULL;
	mNbJoints = -1;
}

S3DModel::S3DModel(const Joint* jt, unsigned int id)
{
	mRootJoint = new Joint(*jt);
	mId = id;
	mNbJoints = -1;
	createMaps();
	std::cout << "S3DModel : model index successfully created !" << std::endl;
}

S3DModel::~S3DModel()
{
	delete mRootJoint;
}

Joint* S3DModel::getRootJoint()
{
	return mRootJoint;
}

void S3DModel::createMaps()
{
	if (mRootJoint != NULL)
	{
		mNbJoints++;
		mStringToJoint[mRootJoint->getName()] = mRootJoint;
		mIntToJoint[mNbJoints] = mRootJoint;
		mStringToInt[mRootJoint->getName()] = mNbJoints;
		
		if (mRootJoint->hasChildren())
		{
			vector<Joint*> children = mRootJoint->getChildren();
			createMaps(children);
		}
	}
}

void S3DModel::createMaps(vector<Joint*>& jts)
{
	if (jts.size() > 0)
	{
		for (int i=0 ; i < jts.size() ; i++)
		{
			mNbJoints++;
			mStringToJoint[jts[i]->getName()] = jts[i];
			mIntToJoint[mNbJoints] = jts[i];
			mStringToInt[jts[i]->getName()] = mNbJoints;
			
			if (jts[i]->hasChildren())
			{
				vector<Joint*> children = jts[i]->getChildren();
				createMaps(children);
			}
		}
	}
}


