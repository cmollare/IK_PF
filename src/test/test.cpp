#include <iostream>
#include <vector>
#include "../3DModel/S3DModel.h"
#include "../FileParsers/YamlBodyJoint.h"
#include "../FileParsers/FileParser.h"
#include "../viewer/S3DViewer.h"
#include "../solver/IKSolverPF.h"
#include "../solver/IKSolverPFOrient.h"
#include "../filter/PartFilter.h"
#include "../filter/PartQRSFilter.h"

#define NBMODELS 100

using namespace std;

int main()
{
	YamlBodyJoint ymlBJ("../Model_simple.ymd");//Yaml parser
	ymlBJ.createModel();
	
	FileParser *fileParser = new FileParser("../skel/", "skel_", 795);//Animation file parser
	
	Joint* model = ymlBJ.getModel();//temporary model
	
	S3DViewer viewer;//Declaration of viewer
	viewer.setOptions(true, false, true);
	
	//******************************************
	//*************INITIALISATION***************
	//******************************************
	
	std::vector<S3DModel*> mods;//Initialisations of all models
	for (int i=0 ; i<NBMODELS ; i++)
	{
		mods.push_back(new S3DModel(model, i));
	}
	
	std::vector<std::vector<double> > frame = fileParser->getFirstFrame();
	std::map<std::string, std::string> jtsToPos; //A mettre dans un fichier
	/*
	jtsToPos["Spine"] = "Spine";
	jtsToPos["Head"] = "Head";
	jtsToPos["ShoulderCenter"] = "ShoulderCenter";
	jtsToPos["ShoulderLeft"] = "NULL";
	jtsToPos["ShoulderRight"] = "NULL";
	jtsToPos["ElbowLeft"] = "NULL";
	jtsToPos["ElbowRight"] = "NULL";
	jtsToPos["WristLeft"] = "NULL";
	jtsToPos["WristRight"] = "NULL";
	jtsToPos["HandLeft"] = "HandLeft";
	jtsToPos["HandRight"] = "HandRight";
	jtsToPos["HipCenter"] = "HipCenter";
	jtsToPos["HipLeft"] = "NULL";
	jtsToPos["HipRight"] = "NULL";
	jtsToPos["KneeLeft"] = "NULL";
	jtsToPos["KneeRight"] = "NULL";
	jtsToPos["AnkleLeft"] = "NULL";
	jtsToPos["AnkleRight"] = "NULL";
	jtsToPos["FootLeft"] = "FootLeft";
	jtsToPos["FootRight"] = "FootRight";//*/
	
	/*	
	jtsToPos["Spine"] = "Spine";
	jtsToPos["Head"] = "Head";
	jtsToPos["ShoulderCenter"] = "ShoulderCenter";
	jtsToPos["ShoulderLeft"] = "ShoulderLeft";
	jtsToPos["ShoulderRight"] = "ShoulderRight";
	jtsToPos["ElbowLeft"] = "ElbowLeft";
	jtsToPos["ElbowRight"] = "ElbowRight";
	jtsToPos["WristLeft"] = "WristLeft";
	jtsToPos["WristRight"] = "WristRight";
	jtsToPos["HandLeft"] = "HandLeft";
	jtsToPos["HandRight"] = "HandRight";
	jtsToPos["HipCenter"] = "HipCenter";
	jtsToPos["HipLeft"] = "HipLeft";
	jtsToPos["HipRight"] = "HipRight";
	jtsToPos["KneeLeft"] = "KneeLeft";
	jtsToPos["KneeRight"] = "KneeRight";
	jtsToPos["AnkleLeft"] = "AnkleLeft";
	jtsToPos["AnkleRight"] = "AnkleRight";
	jtsToPos["FootLeft"] = "FootLeft";
	jtsToPos["FootRight"] = "FootRight";//*/
	
	//*	
	jtsToPos["Spine"] = "Spine";
	jtsToPos["Head"] = "Head";
	jtsToPos["ShoulderCenter"] = "ShoulderCenter";
	jtsToPos["ShoulderLeft"] = "ShoulderLeft";
	jtsToPos["ShoulderRight"] = "ShoulderRight";
	jtsToPos["ElbowLeft"] = "ElbowLeft";
	jtsToPos["ElbowRight"] = "ElbowRight";
	jtsToPos["WristLeft"] = "WristLeft";
	jtsToPos["WristRight"] = "WristRight";
	jtsToPos["HipCenter"] = "HipCenter";
	jtsToPos["HipLeft"] = "HipLeft";
	jtsToPos["HipRight"] = "HipRight";
	jtsToPos["KneeLeft"] = "KneeLeft";
	jtsToPos["KneeRight"] = "KneeRight";
	jtsToPos["AnkleLeft"] = "AnkleLeft";
	jtsToPos["AnkleRight"] = "AnkleRight";//*/
	
	
	IKSolverPFOrient iksol(mods, fileParser->getJointNames(), frame);//Declaration of solver
	iksol.mapJointToObs(jtsToPos);
	iksol.initFilter();
	viewer.init();
	viewer.initModels(mods);
	viewer.initObservations(fileParser->getJointNames(), frame);
	iksol.computeLikelihood();
	
	PartFilter filter(mods, fileParser->getJointNames(), frame);
	filter.mapJointToObs(jtsToPos);
	
	//******************************************
	//**********END INITIALISATION**************
	//******************************************
	
	/*std::vector<Eigen::Quaternionf*, Eigen::aligned_allocator<Eigen::Quaternionf*> > vec = mods[0]->getOrientationVec();
	mods[0]->debug();
	for (int i=0 ; i<vec.size() ; i++)
	{
		*vec[i]*=Eigen::Quaternionf(4,2,5,3);
	}
	mods[0]->debug();//*/
	
	//TESTER LE SAMPLING
	
	/*IKSolverPF iksol(mods);
	
	for(int i=0 ; i<1000 ; i++)
	{
		Eigen::Quaternionf mean;
		mean.setIdentity();
		viewer.displaySampling(iksol.sampleQuTEM(mean,1,1,0.1,1));
	}*/
	
	
	
	//viewer.start(); //infinite loop
	bool continuer = true;
	std::string step = "IK";
	while (continuer)
	{
		//frame = fileParser->getNextFrame();//Observation update
		if (step == "IK")
		{
			if (iksol.stepAlt() < 0.60)
			{
				iksol.save();
				step = "InitFilter";
			}
			viewer.update(mods, frame);
			continuer = viewer.isRendering();
		}
		else if (step == "InitFilter")
		{
			filter.initFilter();
			step = "Filter";
			viewer.update(mods, frame);
			continuer = viewer.isRendering();
		}
		else if (step == "Filter")
		{
			frame = fileParser->getNextFrame();//Observation update
			filter.step(frame);
			viewer.update(mods, frame);
			cin;
			continuer = viewer.isRendering();
		}
		else
		{
			cout << "State unknown" << endl;
			continuer = viewer.isRendering();
		}
	}
	
	for (int i=0 ; i<NBMODELS ; i++)
	{
		delete mods[i];
	}
	
	delete fileParser;

	cout << "Program ended successfully !!!" << endl;
	return 0;
}
