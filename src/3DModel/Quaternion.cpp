#include "Quaternion.h"

Quaternion::Quaternion()
{
	mW = 1;
	mX = 0;
	mY = 0;
	mZ = 0;
}

Quaternion::Quaternion(float W, float X, float Y, float Z)
{
	mW = W;
	mX = X;
	mY = Y;
	mZ = Z;
}

Quaternion::Quaternion(int W, int X, int Y, int Z)
{
	mW = (float) W;
	mX = (float) X;
	mY = (float) Y;
	mZ = (float) Z;
}

Quaternion::~Quaternion()
{
}

float Quaternion::norm()
{
	return sqrt(mW*mW + mX*mX + mY*mY + mZ*mZ);
}

void Quaternion::normalize()
{
	float nor = norm();
	
	mW /= nor;
	mX /= nor;
	mY /= nor;
	mZ /= nor;
}

Eigen::Vector4f Quaternion::getVector4f()
{
	return Eigen::Vector4f(mW, mX, mY, mZ);
}
