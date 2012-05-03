#ifndef JOINT_H
#define JOINT_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "Quaternion.h"

using namespace std;

class Joint
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW	//For Eigen3
		
		Joint(string name, Joint *parent=NULL, Quaternion quat=Quaternion());
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
		
		void setOrientation(const Quaternion quat);
		Eigen::Vector4f getOrientation();
		float getOffset();
		
	private:
	
		std::string mName;
		Joint *mParentJoint;
		std::vector<Joint*> mChildrenJoint;
		Eigen::Vector3i mColors;
		Eigen::Matrix2f mOrientation;
		
		Eigen::Matrix2f mQDefault;
		Quaternion mQLocal;
		Eigen::Matrix2f mQCurrent;
		float mDefaultOffset;
		float mLocalOffset;
		float mCurrentOffset;
};

#endif
