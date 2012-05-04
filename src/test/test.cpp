#include <iostream>
#include <vector>
#include "../3DModel/S3DModel.h"
#include "../FileParsers/YamlBodyJoint.h"
#include "../viewer/S3DViewer.h"

#define NBMODELS 5

using namespace std;

int main()
{
	YamlBodyJoint ymlBJ("../Model.ymd");
	ymlBJ.parseModel();
	ymlBJ.createModel();
	Joint* model = ymlBJ.getModel();
	
	S3DViewer viewer;
	
	vector<S3DModel*> mods;
	for (int i=0 ; i<NBMODELS ; i++)
	{
		mods.push_back(new S3DModel(model, i));
	}
	//S3DModel princMod(model);
	viewer.init();
	viewer.initModels(mods);
	
	viewer.start(); //infinit loop
	
	for (int i=0 ; i<NBMODELS ; i++)
	{
		delete mods[i];
	}

	cout << "Program ended successfuly !!!" << endl;
	return 0;
}
