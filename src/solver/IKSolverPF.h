#include "IKSolver.h"
#include "../3DModel/S3DModel.h"
#include <vector>

using namespace std;

class IKSolverPF : public IKSolver
{
	public:
		IKSolverPF(std::vector<S3DModel*> mods, std::vector<std::string> posNames, std::vector<std::vector<double> > jointsXYZPositions);

		virtual void initFilter();
		virtual void computeDistance();
		
	private:
		
		Eigen::VectorXf mCurrentWeights;
};
