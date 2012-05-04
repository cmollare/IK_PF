#ifndef YAMLBODYJOINT_H
#define YAMLBODYJOINT_H

#include <yaml-cpp/yaml.h>
#include "../3DModel/Joint.h"
#include <fstream>
#include <string>

typedef struct SBJoints
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	std::string Joint;
	std::string Parent;
	vector<float> Offset;
	Eigen::Quaternionf Orientation;
}SBJoints;
			
typedef struct SYmdFile
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	std::string NbPoses;
	std::string NbJoints;
	std::string FirstIndex;
	vector<SBJoints, Eigen::aligned_allocator<SBJoints> > BJoints;
}SYmdFile;

class YamlBodyJoint
{
	public:
		YamlBodyJoint(std::string fileName);
		~YamlBodyJoint();
		
		void parseModel();
		void createModel();
		Joint* getModel();
	
	private:
		
		std::string mFileName;
		Joint *mModel;
		SYmdFile mParsedFile;
		
		

};

void operator>> (const YAML::Node& node, SYmdFile &YmdFile);
void operator>> (const YAML::Node& node, vector<SBJoints, Eigen::aligned_allocator<SBJoints> > &BJoints);
void operator>> (const YAML::Node& node, Eigen::Quaternionf& quat);
void operator>> (const YAML::Node& node, vector<float>& Offset);

#endif
