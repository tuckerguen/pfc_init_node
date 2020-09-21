#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <string>
#include <fstream>
#include "pfc_initializer_constants.h"
#include "pfc_initializer.h"
#include "pose_helper.h"

using namespace std;

vector<vector<string>> runForPoseAndType(int pose_id, string img_type, bool cheat, bool print, bool thread);
vector<vector<string>> runOnAllData(bool cheat, bool print, bool thread);
void writeDataListToCSV(vector<vector<string>> dataList);
bool processArgBool(string str);

string use_msg_one = "use (run on one img type and pose): ./main <pose_id> <img_type> <cheat(true/false)> <print(true/false)> <thread(t/f)>" ;
string use_msg_all = "use (run on all img types and poses): ./main <cheat(true/false)> <print(true/false)> <thread(t/f)>";
vector<string> csv_key
{ 
    "l_rect.x", "l_rect.y", "r_rect.x", "r_rect.y", "l_scale", "r_scale", 
    "loc.x", "loc.y", "loc.z", "rot_quat.x", "rot_quat.y", "rot_quat.z", "rot_quat.w",
    "rot.roll", "rot.pitch", "rot.yaw", "pos_err", "rot_err"
};

bool processArgBool(string str)
{
    if(str == "false")
        return false;
    else if(str != "true")
    {
        cout << use_msg_one << endl;
        cout <<  use_msg_all<< endl;
    }
    return true;
}

int main(int argc, char** argv)
{ 
    if(argc == 6)
    {
        string cheat_str = argv[3];
        string print_str = argv[4];
        string thread_str = argv[5];
        
        bool print = processArgBool(print_str);
        bool cheat = processArgBool(cheat_str);
        bool thread = processArgBool(thread_str);

        vector<vector<string>> results = runForPoseAndType(stoi(argv[1]), argv[2], cheat, print, thread);
	    vector<vector<string>> data_list;
	    data_list = results;
    }
    else if (argc == 4)
    {
        string cheat_str = argv[1];
        string print_str = argv[2];
        string thread_str = argv[3];
        
        bool print = processArgBool(print_str);
        bool cheat = processArgBool(cheat_str);
        bool thread = processArgBool(thread_str);

        vector<vector<string>> data_list = runOnAllData(cheat, print, thread);
        data_list.insert(data_list.begin(), csv_key);
        writeDataListToCSV(data_list);
    }
    else
    {
        cout << use_msg_one << endl;
        cout << use_msg_all << endl;
        return 0;
    }
}

vector<vector<string>> runForPoseAndType(int pose_id, string img_type, bool cheat, bool print, bool thread)
{
    string left_img_path = "../imgs/raw/" + std::to_string(pose_id) + "_l_c_" + img_type + ".png";
    string right_img_path = "../imgs/raw/" + std::to_string(pose_id) + "_r_c_" + img_type + ".png";

    pfc::match_params params = {
        0, 360, 10, //yaw
        cv::Range(0,90), 10, //pitch
        cv::Range(0,90), 10, //roll
        0.09, 0.18, 0.01, //z
        5, // # candidate points to return
        10, // # points in needle line
    };

    if(cheat)
    {
        // params = {
        //     pfc::min_rot.at(pose_id), 
        //     pfc::max_rot.at(pose_id),
        //     3,
        //     pfc::min_scl.at(pose_id),
        //     pfc::max_scl.at(pose_id),
        //     5,
        //     5
        // };
    }

    PfcInitializer pfc(left_img_path, right_img_path, params);
    pfc.run(print, thread, pose_id);

    // TODO: Uncomment once finished making these functions work with all matches
    // Get results back as vector
    vector<vector<string>> results = pfc.getResultsAsVector(pose_id);
    return results;
}

vector<vector<string>> runOnAllData(bool cheat, bool print, bool thread)
{
    vector<vector<string> > dataList;
    double rot_err_sum=0;
    double pos_err_sum=0;

    for(int pose_id = 0; pose_id < pfc::num_poses; pose_id++){
        for(int j = 0; j < pfc::num_img_types; j++){
            string img_type = pfc::img_types.at(j);
            cout << "Running for: " << pose_id << ", " << img_type << endl;
            vector<vector<string>> data = runForPoseAndType(pose_id, img_type, cheat, print, thread);
            //TODO: Make this work with the multiple candidate points
            // dataList.at(j).push_back(data);
            // pos_err_sum += stod(data.at(16));
            // rot_err_sum += stod(data.at(17));
        }
    }

    double avg_rot_err = rot_err_sum / dataList.size();
    double avg_pos_err = pos_err_sum / dataList.size();
    dataList.push_back(vector<string>{ "avg_pos_err:" , to_string(avg_pos_err)});
    dataList.push_back(vector<string>{ "avg_rot_err", to_string(avg_rot_err)});
   	printf("avg pos error = %f\n", avg_pos_err);
   	printf("avg rot error = %f\n", avg_rot_err);	    
    
    return dataList;
}

void writeDataListToCSV(vector<string> dataList)
{
    ofstream data_file;
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%X");
    string time = ss.str();
    data_file.open("../result_data/pfcinit_performance_data_" + time + ".csv");
    
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