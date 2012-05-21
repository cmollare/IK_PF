#ifndef IKSOLVER_H
#define IKSOLVER_H

/*!
 * \file IKSolver.h
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>
#include "../3DModel/S3DModel.h"

using namespace std;

/*!
 * \class IKSolver
 * \brief Abstract class for the inverse kinematic solver
 */
class IKSolver
{
	public:
		IKSolver(std::vector<S3DModel*> mods);
		
		virtual void initFilter()=0;
		virtual void computeLikelihood()=0;
		virtual void step()=0;
		Eigen::Quaternionf sampleQuTEM(Eigen::Quaternionf mean, float sigma, float sigma1=1, float sigma2=1, float sigma3=1);
		
	protected:
		virtual void computeDistance()=0;
		
		/*!
		 * \fn float randn()
		 * \brief Sampling from a normal distribution using the Box-Muller algorithm
		 * \return A float sampled following a normal distribution.
		 */
		float randn();
		float randUnif(float sup=1.0);
		
		std::map<std::string, std::string> mJointNameToPosName; /*!< Map between Joint Names file and animation file */
		std::map<std::string, int> mJointNameToPos;
		std::map<std::string, int> mJointNameToInt; /*!< Name of the Joint to its position in the orientation vector */
		
		std::vector<S3DModel*> mModels;
		std::vector<std::vector<Eigen::Quaternionf*, Eigen::aligned_allocator<Eigen::Quaternionf*> > > mOrientationVec;
		std::vector<std::vector<Eigen::Translation3f*, Eigen::aligned_allocator<Eigen::Translation3f*> > > mOffsetVec;
		std::vector<std::vector<Eigen::Quaternionf, Eigen::aligned_allocator<Eigen::Quaternionf> > > mDefaultOrientationVec;
		std::vector<std::vector<Eigen::Translation3f, Eigen::aligned_allocator<Eigen::Translation3f> > > mDefaultOffsetVec;
		std::vector<std::vector<std::string> > mNameVec;
		std::vector<std::vector<std::string> > mConstOffsetVec;
		std::vector<std::vector<std::string> > mConstOrientVec;
		
		std::vector<std::vector<double> > mCurrentFrame;
		Eigen::VectorXf mCurrentDistances;
		Eigen::VectorXf mCurrentLikelihood;
};

#endif
