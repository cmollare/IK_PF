#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>

using namespace std;

class IKSolver
{
	public:
		IKSolver();
		
		virtual void initFilter()=0;
		Eigen::Quaternionf samplePrior(Eigen::Quaternionf mean, float sigma, float sigma1=1, float sigma2=1, float sigma3=1);
		
	protected:
		float randn();//Box-Muller algorithm
		
};
