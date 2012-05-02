#include "YamlBodyJoint.h"

using namespace std;

YamlBodyJoint::YamlBodyJoint(std::string fileName)
{
	mFileName = fileName;
	mModel = NULL;
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

void YamlBodyJoint::createModel()
{
	this->parseModel();
	
	if (mParsedFile.BJoints[0].Parent == "_Root")
	{
		Joint model(mParsedFile.BJoints[0].Joint);
	}
	else
	{
		std::cout << "Error : The first joint is not root" << std::endl;
	}
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
		
		if (i==0 && (BJoints[i].Parent != "_Root"))
		{
			std::cout << "Error : The first joint of the Yaml file have to be root" << std::endl;
			BJoints[i].Joint = "_Root";
			std::cout << "The joint : " << BJoints[i].Joint << "have been setted to _Root by default" << std::endl;
		}
	}
}

