#ifndef CSV_READER_H
#define CSV_READER_H

#include <string>
#include <vector>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <iostream>
#include <boost/algorithm/string.hpp>

using namespace std;

/*
 * A class to read data from a csv file.
 */
class CSVReader
{
	/**
	 * @brief Name of csv file to read from
	 */
	string file_name;

	/**
	 * @brief Data delimeter in file (probably ",")
	 */
	string delimeter;
 
public:
	/**
	 * @brief Constructor
	 * 
	 * @param filename Name of csv file to read from
	 * @param delim Data delimeter in file (probably ",") 
	 */ 
	CSVReader(string filename, string delim = ",") :
			file_name(filename), delimeter(delim)
	{ }
 
	/**
	 * @brief Reads all rows from csv into vector
	 */
	vector<vector<string> > getData();

};

void writeDataListToCSV(vector<vector<string>> dataList);


#endif