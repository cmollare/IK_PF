#ifndef JOINT_H
#define JOINT_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;

class Joint
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Joint(string name, Joint *parent=NULL);
		Joint(const Joint& jtCopy);
		~Joint();
		
		//void setDof(vector<bool> dof);
		Eigen::Matrix2f getCurrOrientation();
		Joint *getRoot();
		void setParentIfChild(Joint *jt);
		Joint *getParent();
		Joint *getJointFromName(std::string name);
		Joint *getJointFromName(Joint *jt, std::string name);
		std::string getName();
		bool hasChildren();
		std::vector<Joint*>& getChildren();
		void addChild(std::string name);
		void addChild(Joint* jt);
		
		Eigen::Matrix2f getOrientation();
		float getOffset();
		
	private:
	
		std::string mName;
		Joint *mParentJoint;
		std::vector<Joint*> mChildrenJoint;
		Eigen::Vector3i mColors;
		Eigen::Matrix2f mOrientation;
		
		Eigen::Matrix2f mQDefault;
		Eigen::Matrix2f mQLocal;
		Eigen::Matrix2f mQCurrent;
		float mDefaultOffset;
		float mLocalOffset;
		float mCurrentOffset;
};

#endif
