#ifndef S3DMODEL_H
#define S3DMODEL_H

#include <iostream>
#include <map>
#include "Joint.h"

class S3DModel
{
	public:
		S3DModel(int id);
		S3DModel(const Joint* jt, unsigned int id=0);
		S3DModel(const S3DModel& model);
		~S3DModel();
		
		Joint* getRootJoint();
		
	private:
		void createMaps();
		void createMaps(vector<Joint*>& jts);
		
		unsigned int mId;
		Joint *mRootJoint;
		
		std::map<std::string, Joint*> mStringToJoint;
		std::map<std::string, int> mStringToInt;
		std::map<int, Joint*> mIntToJoint;
		int mNbJoints;
		
};

#endif
