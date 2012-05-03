#ifndef QUATERNION_H
#define QUATERNION_H

#include <Eigen/Dense>

class Quaternion
{
	public:
		Quaternion();
		Quaternion(float W, float X, float Y, float Z);
		~Quaternion();
		
		float norm();
		void normalize();
		Eigen::Vector4f getVector4f();
	
	private:
		float mW, mX, mY, mZ;
};

#endif
