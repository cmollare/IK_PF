#include "IKSolverPF.h"

IKSolverPF::IKSolverPF(std::vector<S3DModel*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions) : IKSolver(mods)
{
	vector<std::string> jtNames = mModels[0]->getNameVec();
	mCurrentFrame = jointsXYZPositions;
	for (int i=0 ; i < jtNames.size() ; i++)
	{
		mJointNameToPosName[jtNames[i]] = posNames[i];
		mJointNameToInt[jtNames[i]] = i;
		if (posNames[i]=="NULL")
			mJointNameToPos[jtNames[i]]=-1;
		else
			mJointNameToPos[jtNames[i]]=i;
	}
}

void IKSolverPF::initFilter()
{
	for (int i=0 ; i<mModels.size() ; i++)
	{
		mOrientationVec.push_back(mModels[i]->getOrientationVec());
		mOffsetVec.push_back(mModels[i]->getOffsetVector());
		mNameVec.push_back(mModels[i]->getNameVec());
	}
	
	for (int i=0 ; i<mOrientationVec.size() ; i++)
	{
		for (int j=0 ; j < mOrientationVec[i].size() ; j++)
		{
			Eigen::Quaternionf quat;
			quat.setIdentity();
			(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 3.14, 1, 1, 1);//A modifier suivant les contraintes
			(*mOffsetVec[i][j])=Eigen::Translation3f(this->randn()*0.1, this->randn()*0.1, this->randn()*0.1);//A modifier suivant les contraintes
		}
	}
}


