#include <iostream>
#include "../3DModel/S3DModel.h"
#include "../FileParsers/YamlBodyJoint.h"

using namespace std;

int main()
{
	YamlBodyJoint ymlBJ("../Model.ymd");
	S3DModel princMod(0);
	
	cout << "Hello world !!!" << endl;
	return 0;
}
