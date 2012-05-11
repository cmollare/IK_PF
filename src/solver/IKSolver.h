#ifndef IKSOLVER_H
#define IKSOLVER_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include "../3DModel/S3DModel.h"

using namespace std;

class IKSolver
{
	public:
		IKSolver(std::vector<S3DModel*> mods);
		
		virtual void initFilter()=0;
		Eigen::Quaternionf sampleQuTEM(Eigen::Quaternionf mean, float sigma, float sigma1=1, float sigma2=1, float sigma3=1);
		
	protected:
		float randn();//Box-Muller algorithm
		void mapXYZPositions(std::vector<std::string> JointsNames, std::vector<int> jointsXYZPositions);
		
		std::map<std::string, int> mJointNameToPos;
		std::vector<S3DModel*> mModels;
		std::vector<std::vector<Eigen::Quaternionf*, Eigen::aligned_allocator<Eigen::Quaternionf*> > >mOrientationVec;
		std::vector<std::vector<std::string> > mNameVec;
};

#endif
