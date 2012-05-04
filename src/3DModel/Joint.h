#ifndef JOINT_H
#define JOINT_H

#include <cstdlib> 
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "Quaternion.h"

using namespace std;

class Joint
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW	//For Eigen3
		
		Joint(string name, Joint *parent=NULL, vector<float> offset=vector<float>(), Quaternion quat=Quaternion());
		Joint(const Joint& jtCopy);
		~Joint();
		
		//void setDof(vector<bool> dof);
		Joint *getRoot();
		void setParentIfChild(Joint *jt);
		Joint *getParent();
		Joint *getJointFromName(std::string name);
		Joint *getJointFromName(Joint *jt, std::string name);
		std::string getName();
		bool hasChildren();
		std::vector<Joint*>& getChildren();
		Joint* addChild(std::string name, vector<float> offset=vector<float>(), Quaternion quat=Quaternion());
		Joint* addChild(Joint* jt);
		
		void setOrientation(const Quaternion quat);
		Eigen::Vector4f getOrientation();
		Eigen::Vector3f getOffset();
		
	private:
	
		std::string mName;
		Joint *mParentJoint;
		std::vector<Joint*> mChildrenJoint;
		Eigen::Vector3i mColors;
		//Eigen::Matrix2f mOrientation;
		
		//Eigen::Matrix2f mQDefault;
		Quaternion mQLocal;
		//Eigen::Matrix2f mQCurrent;
		//float mDefaultOffset;
		Eigen::Vector3f mLocalOffset;
		//float mCurrentOffset;
};

#endif
