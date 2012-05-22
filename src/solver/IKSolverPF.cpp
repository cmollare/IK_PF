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
	mMaxWeightIndex=0;
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
		mDefaultOrientationVec.push_back(mModels[i]->getDefaultOrientationVec());
		mDefaultOffsetVec.push_back(mModels[i]->getDefaultOffsetVector());
		mNameVec.push_back(mModels[i]->getNameVec());
		mConstOffsetVec.push_back(mModels[i]->getConstOffsetVec());
		mConstOrientVec.push_back(mModels[i]->getConstOrientVec());
		
	}
	
	for (int i=0 ; i<mModels.size() ; i++)
	{
		for (int j=0 ; j < mOrientationVec[i].size() ; j++)
		{
			Eigen::Quaterniond quat = mDefaultOrientationVec[i][j];
			//quat.setIdentity();
			bool valide = true;
			Eigen::Vector3d offs = mDefaultOffsetVec[i][j].vector();
			while(valide)
			{
				valide = false;
				if (mConstOrientVec[i][j] == ORIENT_CONST_FREE)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 3.14, 1, 1, 1);//A modifier suivant les contraintes
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TWIST)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 3.14, 1, 0.1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 3.14, 0.1, 1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 3.14, 1, 1, 0.1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_BIFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 3.14, 0.1, 1, 1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FIXED)
				{
					(*mOrientationVec[i][j]) = (*mOrientationVec[i][j]);
				}
				else
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 3.14, 1, 1, 1);
				}
				mOrientationVec[i][j]->normalize();
				
				Eigen::Vector3d tempo;
				if (mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
				{
					tempo = Eigen::Vector3d(this->randn()*0.01, this->randn()*0.01, this->randn()*0.01);
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_BONE)
				{
					tempo = Eigen::Vector3d(this->randn()*0.1, 0, 0);
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_FIXED)
				{
					tempo = Eigen::Vector3d(0, 0, 0);
				}
				tempo+=offs;
				(*mOffsetVec[i][j])=Eigen::Translation3d(tempo);//A modifier suivant les contraintes

				//To avoid infinite and NaN cases
				valide |= ((mOffsetVec[i][j]->x() == std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->y() == std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->z() == std::numeric_limits<double>::infinity()));
				valide |= ((mOffsetVec[i][j]->x() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->y() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->z() == -std::numeric_limits<double>::infinity()));
				valide |= (mOrientationVec[i][j]->w() != mOrientationVec[i][j]->w());
			}//*/
			
			
		}
	}//*/
}

void IKSolverPF::computeDistance()
{	
	//ajouter plus de poids au root ???
	std::map<std::string, int>::iterator it;
	for (int i=0 ; i<mModels.size() ; i++)
	{
		double distance=0;
		for (it = mJointNameToPos.begin() ; it != mJointNameToPos.end() ; it++)
		{
			if ((*it).second != -1)
			{
				Eigen::Vector3d jtPos = mModels[i]->getJoint((*it).first)->getXYZVect();
				Eigen::Vector3d jtObs(mCurrentFrame[(*it).second][1], mCurrentFrame[(*it).second][2], mCurrentFrame[(*it).second][3]);
				Eigen::Vector3d diff = jtPos - jtObs;
				Eigen::Matrix3d cov;
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
	//cout << mCurrentLikelihood << "//" << endl;
}

void IKSolverPF::step()
{
	for (int i=0 ; i<mModels.size() ; i++)
	{
		mModels[i]->setPrincipal(true);
		for (int j=0 ; j < mOrientationVec[i].size() ; j++)
		{
			bool valide = true;
			Eigen::Quaterniond quat = mDefaultOrientationVec[i][j];
			Eigen::Vector3d offs;
			if (mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
			{
				offs = mOffsetVec[i][j]->vector();
			}
			else
			{
				offs = mDefaultOffsetVec[i][j].vector();
			}
			while(valide)
			{
				valide = false;
				if (mConstOrientVec[i][j] == ORIENT_CONST_FREE)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 1, 1, 1, 1);//A modifier suivant les contraintes
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TWIST)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 3.14, 0.5, 0.1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 3.14, 0.1, 1, 0.05);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_TFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 3.14, 1, 1, 0.1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_BIFLEX)
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 3.14, 0.1, 1, 1);
				}
				else if(mConstOrientVec[i][j] == ORIENT_CONST_FIXED)
				{
					(*mOrientationVec[i][j]) = (*mOrientationVec[i][j]);
				}
				else
				{
					(*mOrientationVec[i][j])=this->sampleQuTEM(quat, 3.14, 1, 1, 1);
				}
				mOrientationVec[i][j]->normalize();

				Eigen::Vector3d tempo;
				if (mConstOffsetVec[i][j] == OFFSET_CONST_FREE)
				{
					tempo = Eigen::Vector3d(this->randn()*0.01, this->randn()*0.01, this->randn()*0.01);
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_BONE)
				{
					tempo = Eigen::Vector3d(this->randn()*0.01, 0, 0);
				}
				else if (mConstOffsetVec[i][j] == OFFSET_CONST_FIXED)
				{
					tempo = Eigen::Vector3d(0, 0, 0);
				}
				tempo+=offs;
				(*mOffsetVec[i][j])=Eigen::Translation3d(tempo);//A modifier suivant les contraintes

				//To avoid infinite and NaN cases
				valide |= ((mOffsetVec[i][j]->x() == std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->y() == std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->z() == std::numeric_limits<double>::infinity()));
				valide |= ((mOffsetVec[i][j]->x() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->y() == -std::numeric_limits<double>::infinity()) || (mOffsetVec[i][j]->z() == -std::numeric_limits<double>::infinity()));
				valide |= (mOrientationVec[i][j]->w() != mOrientationVec[i][j]->w());
			}
			
			
		}
	}
	
	this->updateWeights();
	double Neff = this->computeNeff();;
	//cout << Neff << "****" << endl;
	if (Neff < 1.5 || Neff < mModels.size()*0.1)
		this->resample();
}

double IKSolverPF::computeNeff()
{
	return 1./(mCurrentWeights.dot(mCurrentWeights));
}

void IKSolverPF::updateWeights()
{
	double sum=0;
	this->computeLikelihood();
	mModels[mMaxWeightIndex]->setPrincipal(false);
	
	for (int i=0 ; i<mCurrentLikelihood.size() ; i++)
	{
		mCurrentWeights[i] = mCurrentWeights[i]*mCurrentLikelihood[i];
		sum+=mCurrentWeights[i];
		
		/*if(mCurrentWeights[i]>=mCurrentWeights[mMaxWeightIndex])
		{
			mMaxWeightIndex=i;
		}*/
	}
	for (int i=0 ; i<mCurrentLikelihood.size() ; i++)
	{
		if(mCurrentWeights[i]>=mCurrentWeights[mMaxWeightIndex])
		{
			mMaxWeightIndex=i;
		}
	}
	
	mCurrentWeights/=sum;
	mModels[mMaxWeightIndex]->setPrincipal(true);
	//cout << mMaxWeightIndex << endl;
}

void IKSolverPF::resample()
{
	double invNbSamp = 1./mModels.size();
	Eigen::VectorXf cdf(mModels.size());
	cdf[0]=mCurrentWeights[0];
	for (int i=1 ; i<mModels.size() ; i++)
	{
		cdf[i]=cdf[i-1]+mCurrentWeights[i];
	}
	
	int i=0;
	double u = randUnif(invNbSamp);
	for (int j=0 ; j<mModels.size() ; j++)
	{
		while(u>cdf[i])
		{
			i++;
		}
		for (int k=0 ; k<mOrientationVec[i].size() ; k++)
		{
			(*mOrientationVec[j][k])=(*mOrientationVec[i][k]);
			(*mOffsetVec[j][k])=(*mOffsetVec[i][k]);
		}
		mCurrentWeights[j]=invNbSamp;
		u=u+invNbSamp;
	}
}


