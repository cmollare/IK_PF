#include "IKSolverPF.h"

IKSolverPF::IKSolverPF(vector<S3DModel*> mods) : IKSolver(mods)
{
}

void IKSolverPF::initFilter()
{
	for (int i=0 ; i<mModels.size() ; i++)
	{
		mOrientationVec.push_back(mModels[i]->getOrientationVec());
		mNameVec.push_back(mModels[i]->getNameVec());
	}
	
	for (int i=0 ; i<mOrientationVec.size() ; i++)
	{
		for (int j=0 ; j < mOrientationVec[i].size() ; j++)
		{
			Eigen::Quaternionf quat;
			quat.setIdentity();
			(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 3.14, 1, 1, 1);
		}
	}
}


