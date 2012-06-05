#ifndef RESULTPARSER_H
#define RESULTPARSER_H

/*!
 * \file ResultParser.h
 * \brief Parser to save some results.
 */

#include <string>
#include <fstream>

/*!
 * \class ResultParser
 * \brief Class to save some results.
 */
class ResultParser
{
	public:
		ResultParser(std::string path);
		~ResultParser();
};

#endif
