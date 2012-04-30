#ifndef YAMLBODYJOINT_H
#define YAMLBODYJOINT_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>

class YamlBodyJoint
{
	public:
		YamlBodyJoint(std::string fileName);
		~YamlBodyJoint();
};

#endif
