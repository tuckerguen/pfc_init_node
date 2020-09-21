#include <vector>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "csv_reader.h"
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <string>
#include <fstream>

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

void writeDataListToCSV(vector<vector<string>> dataList)
{
    ofstream data_file;
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%X");
    string time = ss.str();
    data_file.open("src/pfc_init/test_results/pfcinit_performance_data_" + time + ".csv");
    
    if (data_file.fail()){
        cout << "couldn't open file" << endl;
    }

    for(int i = 0; i < dataList.size(); i++){
        for(int j = 0; j < dataList.at(i).size(); j++)
        {
            string data_point = dataList.at(i).at(j);
            if(j < dataList.at(i).size()-1)
                data_file << data_point << ", ";
            else 
                data_file << data_point;
        }
        data_file << "\n";
    }
}