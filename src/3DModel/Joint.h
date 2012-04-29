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
		Eigen::Matrix4f getOrientation();
		
	private:
		string mName;
		Joint *mParentJoint;
		vector<Joint*> mChildrenJoint;
		Eigen::Vector3i mColors;
		Eigen::Matrix2f mOrientation;
		int mOffset;
		
		Eigen::Matrix4f mCurrent;
		Eigen::Matrix4f mParent;
		Eigen::Matrix4f mWorld;
};

#endif
