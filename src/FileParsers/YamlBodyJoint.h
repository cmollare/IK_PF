#ifndef YAMLBODYJOINT_H
#define YAMLBODYJOINT_H

#include <yaml-cpp/yaml.h>
#include "../3DModel/Joint.h"
#include <fstream>
#include <string>

class YamlBodyJoint
{
	public:
		YamlBodyJoint(std::string fileName);
		~YamlBodyJoint();
		
		void parseModel();
	
	private:
		typedef struct SBJoints
		{
			std::string Joint;
			std::string Parent;
		}SBJoints;
			
		typedef struct SYmdFile
		{
			std::string NbPoses;
			std::string NbJoints;
			std::string FirstIndex;
			SBJoints BJoints;
		}SYmdFile;
		
		
		std::string mFileName;
		Joint *mModel;
		SYmdFile mParsedFile;
		
		

};

#endif
