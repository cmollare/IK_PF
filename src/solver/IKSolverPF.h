#include "IKSolver.h"
#include "../3DModel/S3DModel.h"
#include <vector>

using namespace std;

class IKSolverPF : public IKSolver
{
	public:
		IKSolverPF(vector<S3DModel*>& mods);
		
		virtual void initFilter();
		
	private:
		//vector<S3DModel*> mModels;
};
