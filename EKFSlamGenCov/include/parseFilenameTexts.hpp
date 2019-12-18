/*
 * EKFSlam.h
 *
 *  Created on: 03 Nov 2018
 *      Author: sholto
 */

#ifndef FILENAME_TEXT_PARSER_H_
#define FILENAME_TEXT_PARSER_H_

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
using namespace std;
using namespace boost::algorithm;


class FilenameTextParser{
public:
	FilenameTextParser(string rgbFilename);
	virtual ~FilenameTextParser();

	bool getNextFilenames(string &rgbImageFilename, string &timestamp, string &depthImageFilename);
	bool getFilenames(string &rgbImageFilename, string &depthImageFilename, string &timestamp, int index);
	bool setIteratorIndex(int index=0);
	string initialRgbImage;

private:
	vector<string> rgbImages, dImages, timestamps;
	int currentIteration;


};

#endif