#ifndef JOINT_H
#define JOINT_H

#include <iostream>
#include <vector>

using namespace std;

class Joint
{
	public:
		Joint(string name, Joint *parent=NULL);
		~Joint();
		
		void setDof(vector<bool> dof);
		
	private:
		Joint *mParentJoint;
		vector<Joint*> mChildrenJoint;
};

#endif
