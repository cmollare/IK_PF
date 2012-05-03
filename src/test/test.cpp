#include <iostream>
#include <vector>
#include "../3DModel/S3DModel.h"
#include "../FileParsers/YamlBodyJoint.h"
#include "../viewer/S3DViewer.h"

using namespace std;

int main()
{
	YamlBodyJoint ymlBJ("../Model.ymd");
	ymlBJ.parseModel();
	ymlBJ.createModel();
	Joint* model = ymlBJ.getModel();
	
	S3DViewer viewer;
	
	/*vector<S3DModel*> mods;
	for (int i=0 ; i<100 ; i++)
	{
		mods.push_back(new S3DModel(model));
	}
	S3DModel princMod(model);*/
	
	/*for (int i=0 ; i<100 ; i++)
	{
		delete mods[i];
	}*/

	cout << "Program ended successfuly !!!" << endl;
	return 0;
}
