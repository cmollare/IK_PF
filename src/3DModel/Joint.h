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
		~Joint();
		
		//void setDof(vector<bool> dof);
		Eigen::Matrix2f getCurrOrientation();
		Joint *getRoot();
		Joint *getParent();
		Joint *getJointFromName(std::string name);
		Joint *getJointFromName(Joint *jt, std::string name);
		std::string getName();
		bool hasChildren();
		std::vector<Joint*>& getChildren();
		void addChild(std::string name);
		
	private:
		std::string mName;
		Joint *mParentJoint;
		std::vector<Joint*> mChildrenJoint;
		Eigen::Vector3i mColors;
		Eigen::Matrix2f mOrientation;
		
		Eigen::Matrix2f mQDefault;
		Eigen::Matrix2f mQLocal;
		Eigen::Matrix2f mQCurrent;
		int mDefaultOffset;
		int mLocalOffset;
		int mCurrentOffset;
};

#endif
