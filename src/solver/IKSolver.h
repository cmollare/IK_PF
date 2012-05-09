#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <Eigen/Dense>

class IKSolver
{
	public:
		IKSolver();
		
		virtual void initFilter()=0;
		void samplePrior();
		
	protected:
		float randn();//Box-Muller algorithm
		
};
