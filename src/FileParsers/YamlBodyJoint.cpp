#include "YamlBodyJoint.h"

using namespace std;

YamlBodyJoint::YamlBodyJoint(std::string fileName)
{
	std::ifstream file(fileName.c_str());
	
	YAML::Parser parser(file);
	YAML::Node doc;
	
	parser.GetNextDocument(doc);
	
	doc[0];
	std::cout << doc.size() << endl;
	
	file.close();
}

YamlBodyJoint::~YamlBodyJoint()
{
}
