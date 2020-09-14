#include <vector>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "csv_reader.h"
 
using namespace std;
 
// Reads all rows from csv into vector
vector<vector<string> > CSVReader::getData()
{
	// Create file stream
	ifstream csv(file_name);
    if (csv.fail()){
        cout << "couldn't open file" << endl;
    }
	vector<vector<string> > dataList;
 
	string row = "";
	// Loop over all rows in file
	while (getline(csv, row))
	{
		vector<string> data;

		// Split row over delimeter and store
		boost::algorithm::split(data, row, boost::is_any_of(delimeter));
		dataList.push_back(data);
	}

	csv.close();
 
	return dataList;
}