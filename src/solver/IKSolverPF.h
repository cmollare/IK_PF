#include "IKSolver.h"
#include "../3DModel/S3DModel.h"

class IKSolverPF : public IKSolver
{
	public:
		IKSolverPF(vector<S3DModel*>& mods);
		
	private:
		//vector<S3DModel*> mModels;
};
