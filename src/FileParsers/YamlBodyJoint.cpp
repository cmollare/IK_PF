#include "YamlBodyJoint.h"

using namespace std;

YamlBodyJoint::YamlBodyJoint(std::string fileName)
{
	mFileName = fileName;
	
	/*std::ifstream file(fileName.c_str());
	
	YAML::Parser parser(file);
	YAML::Node doc;
	
	parser.GetNextDocument(doc);
	
	const YAML::Node& node = doc["BJoints"];
	const YAML::Node& node2 = node[0];
	
	string tempo;
	node2["Parent"] >> tempo;
	cout <<  tempo << endl;
	std::cout << doc.size() << endl;
	
	file.close();*/
}

YamlBodyJoint::~YamlBodyJoint()
{
}

void YamlBodyJoint::parseModel()
{
	std::ifstream file(mFileName.c_str());
	YAML::Parser parser(file);
	YAML::Node node;
	parser.GetNextDocument(node);
	
	file.close();
}

