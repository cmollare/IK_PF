#include "IKSolver.h"
#include "../3DModel/S3DModel.h"
#include <vector>

using namespace std;

class IKSolverPF : public IKSolver
{
	public:
		IKSolverPF(std::vector<S3DModel*> mods);
		
		virtual void initFilter();
		
	private:
};
