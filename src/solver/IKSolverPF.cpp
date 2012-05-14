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
	mCurrentWeights.setConstant(mModels.size(), 1, 1./mModels.size());
	mCurrentDistances.resize(mModels.size(), 1);
	mCurrentLikelihood.resize(mModels.size(), 1);
	
	for (int i=0 ; i<mModels.size() ; i++)
	{
		mOrientationVec.push_back(mModels[i]->getOrientationVec());
		mOffsetVec.push_back(mModels[i]->getOffsetVector());
		mNameVec.push_back(mModels[i]->getNameVec());
	}
	
	for (int i=0 ; i<mModels.size() ; i++)
	{
		for (int j=0 ; j < mOrientationVec[i].size() ; j++)
		{
			Eigen::Quaternionf quat;
			quat.setIdentity();
			bool valide = true;
			while(valide)
			{
				(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 3.14, 1, 1, 1);//A modifier suivant les contraintes
				//cout << Eigen::Matrix3f(*mOrientationVec[i][j]) << endl;
				(*mOffsetVec[i][j])=Eigen::Translation3f(this->randn()*0.1, this->randn()*0.1, this->randn()*0.1);//A modifier suivant les contraintes
				valide = (mOrientationVec[i][j]->w() != mOrientationVec[i][j]->w());
			}
			
			
		}
	}
}

void IKSolverPF::computeDistance()
{	
	//ajouter plus de poids au root ???
	std::map<std::string, int>::iterator it;
	for (int i=0 ; i<mModels.size() ; i++)
	{
		float distance=0;
		for (it = mJointNameToPos.begin() ; it != mJointNameToPos.end() ; it++)
		{
			if ((*it).second != -1)
			{
				Eigen::Vector3f jtPos = mModels[i]->getJoint((*it).first)->getXYZVect();
				Eigen::Vector3f jtObs(mCurrentFrame[(*it).second][1], mCurrentFrame[(*it).second][2], mCurrentFrame[(*it).second][3]);
				Eigen::Vector3f diff = jtPos - jtObs;
				Eigen::Matrix3f cov;
				cov.setIdentity();
				distance += diff.transpose()*(cov*diff);
			}
		}
		mCurrentDistances[i] = sqrt(distance);
	}
}

void IKSolverPF::computeLikelihood()
{	
	//contraintes a ajouter
	this->computeDistance();
	
	for (int i=0 ; i<mCurrentDistances.size() ; i++)
	{
		mCurrentLikelihood[i] = exp(-abs(mCurrentDistances[i]));
	}
	//cout << mCurrentLikelihood << endl;
}


