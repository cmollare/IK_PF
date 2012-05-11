#ifndef FILEPARSER_H
#define FILEPARSER_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>

using namespace std;

class FileParser
{
	public:
		FileParser(std::string path, std::string fileBeg, int number);
		void readFile(int number);
		vector<vector<double> > &getFirstFrame();
		vector<vector<double> > &getNextFrame();
		vector<vector<double> > &getCurrentFrame();
		vector<std::string> getJointNames();
		
	private:
		std::string mPath;
		std::string mFileBeg;
		int mMaxFileNumber;
		vector<vector<vector<double> > > mVideoSequence;
		vector<std::string> mJointNames;
		int mNextFrame;
};

#endif
