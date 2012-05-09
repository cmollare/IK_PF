#include "IKSolver.h"

IKSolver::IKSolver()
{
	srand (time(NULL));//Initialisation of randon numbers
}


float IKSolver::randn()
{
	int U1int = rand()%10001;
	int U2int = rand()%10001;
	float U1 = (float)U1int / 10001.;
	float U2 = (float)U2int / 10001.;
	
	float X = sqrt(-2*log(U1))*cos(2*3.14*U2);
	float Y = sqrt(-2*log(U1))*sin(2*3.14*U2);
	return X;
}
