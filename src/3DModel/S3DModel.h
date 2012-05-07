#ifndef S3DMODEL_H
#define S3DMODEL_H

#include <iostream>
#include <map>
#include <Eigen/Dense>
#include "Joint.h"

class S3DModel
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW	//For Eigen3
	
		S3DModel(int id);
		S3DModel(const Joint* jt, unsigned int id=0);
		S3DModel(const S3DModel& model);
		~S3DModel();
		
		Joint* getRootJoint();
		void createMaps();
		int getNumberJoint();
		
	private:
		void createMaps(vector<Joint*>& jts);
		void createOrientationVec();
		
		unsigned int mId;
		Joint *mRootJoint;
		
		std::map<std::string, Joint*> mStringToJoint;
		std::map<std::string, int> mStringToInt;
		std::map<int, Joint*> mIntToJoint;
		int mNbJoints;
		
		vector<Eigen::Quaternionf, Eigen::aligned_allocator<Eigen::Quaternionf> > mOrientationVec;
};

#endif
