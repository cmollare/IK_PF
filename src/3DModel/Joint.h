#ifndef JOINT_H
#define JOINT_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;

class Joint
{
	public:
		Joint(string name, Joint *parent=NULL);
		~Joint();
		
		void setDof(vector<bool> dof);
		Eigen::Matrix2f getCurrOrientation();
		
	private:
		string mName;
		Joint *mParentJoint;
		vector<Joint*> mChildrenJoint;
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
