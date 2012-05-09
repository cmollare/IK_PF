#include "IKSolverPF.h"

IKSolverPF::IKSolverPF(vector<S3DModel*>& mods)
{
	vector<S3DModel*> models = mods;
	Eigen::Quaternionf mean;
	mean.setIdentity();
	samplePrior(mean,1,1,1,1);
}

void IKSolverPF::initFilter()
{
}


