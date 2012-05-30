#ifndef FILTER_H
#define FILTER_H

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
 * \class Filter
 * \brief Abstract class for the posture filter
 */
class Filter
{
	public:
		Filter(std::vector<S3DModel*> mods);
		
		virtual void initFilter()=0;
		virtual void computeLikelihood()=0;
		virtual void step(std::vector<std::vector<double> > frame)=0;
		Eigen::Quaterniond sampleQuTEM(Eigen::Quaterniond mean, double sigma, double sigma1=1, double sigma2=1, double sigma3=1);
		
	protected:
		virtual void computeDistance()=0;
		virtual bool isPositive(double num);
		
		/*!
		 * \fn float randn()
		 * \brief Sampling from a normal distribution using the Box-Muller algorithm
		 * \return A float sampled following a normal distribution.
		 */
		double randn(double sigma=1);
		double randUnif(double sup=1.0);
		
		std::map<std::string, std::string> mJointNameToPosName; /*!< Map between Joint Names file and animation file */
		std::map<std::string, int> mJointNameToPos; /*!< Name of the Joint to its position in the observation vector, -1 if there is no observation associated */
		std::map<std::string, int> mJointNameToInt; /*!< Name of the Joint to its position in the orientation vector */
		std::vector<std::string> mPosNames; /*!< Names of observations */
		
		std::vector<S3DModel*> mModels;
		S3DModel* mModelMMSE;
		std::vector<std::vector<Eigen::Quaterniond*, Eigen::aligned_allocator<Eigen::Quaterniond*> > > mOrientationVec;
		std::vector<std::vector<Eigen::Translation3d*, Eigen::aligned_allocator<Eigen::Translation3d*> > > mOffsetVec;
		std::vector<std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > > mDefaultOrientationVec;
		std::vector<std::vector<Eigen::Translation3d, Eigen::aligned_allocator<Eigen::Translation3d> > > mDefaultOffsetVec;
		std::vector<std::vector<std::string> > mNameVec;
		std::vector<std::vector<std::string> > mConstOffsetVec;
		std::vector<std::vector<std::string> > mConstOrientVec;
		
		std::vector<std::vector<double> > mCurrentFrame;
		Eigen::VectorXf mCurrentDistances;
		Eigen::VectorXf mCurrentLikelihood;
};

#endif
