#include "IKSolver.h"

IKSolver::IKSolver()
{
	srand (time(NULL));//Initialisation of randon numbers
}

Eigen::Quaternionf IKSolver::samplePrior(Eigen::Quaternionf mean, float sigma, float sigma1, float sigma2, float sigma3)
{
	Eigen::Vector4f axis(0, this->randn(), this->randn(), this->randn());
	
	//N
	axis.normalize();
	axis[1]=axis[1]*sigma1;
	axis[2]=axis[2]*sigma2;
	axis[3]=axis[3]*sigma3;
	
	//theta
	float theta = this->randn()*sigma;
	
	//exp(N*theta)
	axis=axis*sin(theta);
	axis[0]=cos(theta);
	
	//to quaternion
	Eigen::Quaternionf quat(axis[0], axis[1], axis[2], axis[3]);
	quat = mean*quat;
	
	return quat;
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
