#include "IKSolverPF.h"

IKSolverPF::IKSolverPF(vector<S3DModel*>& mods)
{
	vector<S3DModel*> models = mods;
	samplePrior(1,1,1,1);
}

void IKSolverPF::initFilter()
{
}


