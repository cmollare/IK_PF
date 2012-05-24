#include "IKSolver.h"
#include "../3DModel/S3DModel.h"
#include <vector>

#define PI 0.1

using namespace std;

class IKSolverPF : public IKSolver
{
	public:
		IKSolverPF(std::vector<S3DModel*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions);

		virtual void initFilter();
		virtual void computeLikelihood();
		virtual void step();
		virtual void stepAlt();
		double computeNeff();
		void mapJointToObs(std::map<std::string, std::string> jointNameToPosName);
		
	protected:
		virtual void computeDistance();
		virtual void computeDistanceHiera();
		void updateWeights();
		void resample();
		
		Eigen::VectorXf mCurrentWeights;
		int mMaxWeightIndex;
};
