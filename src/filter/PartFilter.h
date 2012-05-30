#ifndef PARTFILTER_H
#define PARTFILTER_H

#include "Filter.h"

#define TEMP 0.2

class PartFilter : public Filter
{
	public:
		PartFilter(std::vector<S3DModel*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions);
		
		virtual void initFilter();
		virtual void computeLikelihood();
		virtual void step(std::vector<std::vector<double> > frame);
		double computeNeff();
		void mapJointToObs(std::map<std::string, std::string> jointNameToPosName);
		S3DModel* computeMMSE();
		
	protected:
		virtual void computeDistance();
		void updateWeights();
		void resample();
		
		Eigen::VectorXf mCurrentWeights;
		int mMaxWeightIndex; /*!< Index of maximum weighted model */
		
		std::vector<std::multimap<int, std::string> > mOffsetPartToName;
		std::vector<std::multimap<int, std::string> > mOrientPartToName;
};

#endif
