#include <iostream>
#include <vector>
#include "../3DModel/S3DModel.h"
#include "../FileParsers/YamlBodyJoint.h"
#include "../FileParsers/FileParser.h"
#include "../viewer/S3DViewer.h"
#include "../solver/IKSolverPF.h"

#define NBMODELS 1

using namespace std;

int main()
{
	YamlBodyJoint ymlBJ("../Model.ymd");//Yaml parser
	ymlBJ.createModel();
	
	FileParser *fileParser = new FileParser("../skel/", "skel_", 795);//Animation file parser
	
	Joint* model = ymlBJ.getModel();//temporary model
	
	S3DViewer viewer;//Declaration of viewer
	
	std::vector<S3DModel*> mods;//Initialisations of all models
	for (int i=0 ; i<NBMODELS ; i++)
	{
		mods.push_back(new S3DModel(model, i));
	}
	
	IKSolverPF iksol(mods);//Declaration of solver
	iksol.initFilter();
	//S3DModel princMod(model);
	viewer.init();
	viewer.initModels(mods);
	viewer.initObservations(fileParser->getJointNames(), fileParser->getFirstFrame());
	
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
	
	
	
	viewer.start(); //infinite loop
	
	for (int i=0 ; i<NBMODELS ; i++)
	{
		delete mods[i];
	}
	
	delete fileParser;

	cout << "Program ended successfully !!!" << endl;
	return 0;
}
