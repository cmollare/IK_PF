#include "Joint.h"

Joint::Joint(string name, Joint *parent, vector<float> offset, Eigen::Quaternionf quat)
{
	mParentJoint = parent;
	mName = name;
	
	mQLocal = quat;
	//mQLocal = Quaternion(rand(), rand(), rand(), rand());
	if (offset.size() == 3)
		mLocalOffset = Eigen::Translation3f(offset[0], offset[1], offset[2]);
	else
		mLocalOffset = Eigen::Translation3f(0, 0, 0);
	//mLocalOffset = rand();
	//mLocalOffset/=100000000;

	/*if (mParentJoint != NULL)
		mQDefault = mParentJoint->getCurrOrientation()*mQDefault;*/
}

Joint::Joint(const Joint& jtCopy)
{
	mName = jtCopy.mName;
	mColors = jtCopy.mColors;
	//mOrientation = jtCopy.mOrientation;
	//mQDefault = jtCopy.mQDefault;
	mQLocal = jtCopy.mQLocal;
	//mQCurrent = jtCopy.mQCurrent;
	//mDefaultOffset = jtCopy.mDefaultOffset;
	mLocalOffset = jtCopy.mLocalOffset;
	//mCurrentOffset = jtCopy.mCurrentOffset;
	
	//Dynamics allocations
	
	if (jtCopy.mParentJoint == NULL);
		mParentJoint = NULL;
	
	if (jtCopy.mChildrenJoint.size() > 0)
	{
		for (int i=0 ; i < jtCopy.mChildrenJoint.size() ; i++)
		{
			mChildrenJoint.push_back(new Joint(*(jtCopy.mChildrenJoint[i])));
			mChildrenJoint[i]->setParentIfChild(this);
			//cout << "adresse : " << mChildrenJoint[i] << endl;
			//cout << "adresse2 : " << jtCopy.mChildrenJoint[i] << endl;
		}
	}
}

Joint::~Joint()
{
	if (mChildrenJoint.size() > 0)
	{
		for (int i=0 ; i < mChildrenJoint.size() ; i++)
		{
			delete mChildrenJoint[i];
		}
	}
}

Joint* Joint::getRoot()
{
	Joint* parent = this;
	Joint* prev = NULL;

	do
	{
		prev = parent;
		parent = parent->getParent();
	}while(parent != NULL);
	
	return prev;
}

void Joint::setParentIfChild(Joint *jt)
{
	vector<Joint*> parentChildren = jt->getChildren();
	bool chgt = false;
	if (parentChildren.size() > 0)
	{
		for (int i=0 ; i<parentChildren.size() ; i++)
		{
			if (parentChildren[i] == this)
			{
				mParentJoint = jt;
				chgt = true;
			}
		}
	}
	
	if (!chgt)
	{
		cout << "Error : Child doesn't exist for this parent" << endl;
	}
}

Joint* Joint::getParent()
{
	return mParentJoint;
}

Joint* Joint::getJointFromName(std::string name)
{
	Joint* root = this->getRoot();
	
	if (this->getName() == name)
	{
		return root;
	}
	
	Joint* result = getJointFromName(root, name);
	
	return result;
}

Joint* Joint::getJointFromName(Joint *jt, std::string name)
{
	if (jt->hasChildren())
	{
		std::vector<Joint*> children = jt->getChildren();
		Joint* result = NULL;
		for (int i=0 ; i<children.size() ; i++)
		{
			if (children[i]->getName() == name)
			{
				return children[i];
			}
			else
			{
				result=getJointFromName(children[i], name);
				if (result != NULL)
					return result;
			}
		}
	}
	
	return NULL;
}

std::string Joint::getName()
{
	return mName;
}

bool Joint::hasChildren()
{
	if (mChildrenJoint.size()>0)
		return true;
	else
		return false;
}

std::vector<Joint*>& Joint::getChildren()
{
	return mChildrenJoint;
}

Joint* Joint::addChild(std::string name, vector<float> offset, Eigen::Quaternionf quat)
{
	Joint *jt = new Joint(name, this, offset, quat);
	mChildrenJoint.push_back(jt);
	return jt;
}

Joint* Joint::addChild(Joint* jt)
{
	mChildrenJoint.push_back(jt);
	return jt;
}

void Joint::setOrientation(Eigen::Quaternionf quat)
{
	mQLocal = quat;
}

Eigen::Quaternionf Joint::getOrientation()
{
	return mQLocal;
}

Eigen::Translation3f Joint::getOffset()
{
	return mLocalOffset;
}

Eigen::Vector3f Joint::getXYZVect()
{
	Joint* jt = this;
	//Eigen::Vector4f result = jt->getTransformationMatrix()*Eigen::Vector4f(1,1,1,0);
	Eigen::Transform<float, 3, Eigen::Projective> result = jt->getTransformationMatrix();
	
	while(jt->getParent() != NULL)
	{
		jt = jt->getParent();
		result = jt->getTransformationMatrix()*result;
	}
	
	return result.translation();
	//return Eigen::Vector3f(result[0], result[1], result[2]);
}

Eigen::Transform<float, 3, Eigen::Projective> Joint::getTransformationMatrix()
{
	return mLocalOffset*mQLocal;
}
