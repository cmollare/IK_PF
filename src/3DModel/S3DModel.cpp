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
	createOrientationVec();
	createOffsetVec();
	createNameVec();
	std::cout << "S3DModel : model index successfully created !" << std::endl;
}

S3DModel::S3DModel(const S3DModel& model)
{
	mRootJoint = new Joint(*(model.mRootJoint));
	mId = -2;
	mNbJoints = -1;
	createMaps();
	createOrientationVec();
	createOffsetVec();
	createNameVec();
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
	mStringToJoint.clear();
	mIntToJoint.clear();
	mStringToInt.clear();
	
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

int S3DModel::getNumberJoint()
{
	return mNbJoints+1;
}

vector<Eigen::Quaternionf*, Eigen::aligned_allocator<Eigen::Quaternionf*> > S3DModel::getOrientationVec()
{
	return mOrientationVec;
}

vector<Eigen::Translation3f*, Eigen::aligned_allocator<Eigen::Translation3f*> > S3DModel::getOffsetVector()
{
	return mOffsetVec;
}

vector<std::string> S3DModel::getNameVec()
{
	return mNameVec;
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

void S3DModel::createOrientationVec()
{
	if (mNbJoints != -1)
	{
		for (int i=0 ; i<=mNbJoints ; i++)
		{
			mOrientationVec.push_back(mIntToJoint[i]->getOrientation());
		}
	}
}

void S3DModel::createOffsetVec()
{
	if (mNbJoints != -1)
	{
		for (int i=0 ; i<=mNbJoints ; i++)
		{
			mOffsetVec.push_back(mIntToJoint[i]->getOffset());
		}
	}
}

void S3DModel::createNameVec()
{
	if (mNbJoints != -1)
	{
		for (int i=0 ; i<=mNbJoints ; i++)
		{
			mNameVec.push_back(mIntToJoint[i]->getName());
		}
	}
}


