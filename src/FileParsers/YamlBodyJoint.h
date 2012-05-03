#ifndef YAMLBODYJOINT_H
#define YAMLBODYJOINT_H

#include <yaml-cpp/yaml.h>
#include "../3DModel/Joint.h"
#include <fstream>
#include <string>

typedef struct SBJoints
{
	std::string Joint;
	std::string Parent;
	float Offset;
}SBJoints;
			
typedef struct SYmdFile
{
	std::string NbPoses;
	std::string NbJoints;
	std::string FirstIndex;
	vector<SBJoints> BJoints;
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
void operator>> (const YAML::Node& node, vector<SBJoints> &BJoints);

#endif
