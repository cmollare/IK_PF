#ifndef PFFILTER_H
#define PFFILTER_H

#include "Filter.h"

#define TEMP 0.2

using namespace std;

class PFFilter : public Filter
{
	public:
		PFFilter(std::vector<S3DModel*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions);

		virtual void initFilter();
		virtual void computeLikelihood();
		virtual void step();
		virtual void stepAlt(std::vector<std::vector<double> > frame);
		double computeNeff();
		void mapJointToObs(std::map<std::string, std::string> jointNameToPosName);
		
	protected:
		virtual void computeDistance();
		void updateWeights();
		void resample();
		
		Eigen::VectorXf mCurrentWeights;
		int mMaxWeightIndex;
};

#endif
