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
	
	node >> mParsedFile;
	
	file.close();
}

void operator >> (const YAML::Node& node, SYmdFile &YmdFile)
{
	node["NbPoses"] >> YmdFile.NbPoses;
	node["NbJoints"] >> YmdFile.NbJoints;
	node["FirstIndex"] >> YmdFile.FirstIndex;
	node["BJoints"] >> YmdFile.BJoints;
}

void operator >> (const YAML::Node& node, vector<SBJoints> &BJoints)
{
	BJoints.resize(node.size());
	for (int i=0 ; i<node.size() ; i++)
	{
		const YAML::Node& joint = node[i];
		joint["Joint"] >> BJoints[i].Joint;
		joint["Parent"] >> BJoints[i].Parent;
	}
}

