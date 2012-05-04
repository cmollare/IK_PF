#include "YamlBodyJoint.h"

using namespace std;

YamlBodyJoint::YamlBodyJoint(std::string fileName)
{
	mFileName = fileName;
	mModel = NULL;
}

YamlBodyJoint::~YamlBodyJoint()
{
	delete mModel;
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
	if (mParsedFile.BJoints.size() <= 0)
		this->parseModel();
	
	if (mParsedFile.BJoints[0].Parent == "_Root")
	{
		mModel = new Joint(mParsedFile.BJoints[0].Joint, NULL, mParsedFile.BJoints[0].Orientation);
		
		for (int i=1 ; i<mParsedFile.BJoints.size() ; i++)
		{
			cout << "Searching " << mParsedFile.BJoints[i].Parent << " ..." << endl;
			Joint* parent = mModel->getJointFromName(mParsedFile.BJoints[i].Parent);
			if (parent != NULL)
			{
				parent->addChild(mParsedFile.BJoints[i].Joint, mParsedFile.BJoints[i].Orientation);//Create Children
				cout << mParsedFile.BJoints[i].Joint << " added to " << parent->getName() << endl;
			}
			else
			{
				cout << "Error to retrieve parent" << endl;
				cout << "Check yaml file" << endl;
			}
		}
		cout << "*******Skeleton constructed*******" << endl;
	}
	else
	{
		std::cout << "Error : The first joint has to be root" << std::endl;
	}
}

Joint* YamlBodyJoint::getModel()
{
	return mModel;
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
		joint["Offset"] >> BJoints[i].Offset;
		joint["Orientation"] >> BJoints[i].Orientation;
		
		if (i==0 && (BJoints[i].Parent != "_Root"))
		{
			std::cout << "Error : The first joint of the Yaml file have to be root" << std::endl;
			BJoints[i].Joint = "_Root";
			std::cout << "The joint : " << BJoints[i].Joint << "have been setted to _Root by default" << std::endl;
		}
	}
}

void operator>> (const YAML::Node& node, Quaternion& quat)
{
	quat = Quaternion(node["W"].to<float>(), node["X"].to<float>(), node["Y"].to<float>(), node["Z"].to<float>());
}

