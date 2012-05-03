#include <iostream>
#include "../3DModel/S3DModel.h"
#include "../FileParsers/YamlBodyJoint.h"

using namespace std;

int main()
{
	YamlBodyJoint ymlBJ("../Model.ymd");
	ymlBJ.parseModel();
	ymlBJ.createModel();
	Joint* model = ymlBJ.getModel();
	S3DModel princMod(model);
	
	cout << "Hello world !!!" << endl;
	return 0;
}
