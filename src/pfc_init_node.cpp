#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/ModelStates.h"
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include "pfc_initializer.h"
#include "pose_helper.h"
#include "needle_pose.h"
#include <gazebo_msgs/SetModelState.h>
#include <vector>
#include <chrono>
#include <ctime>
#include <sstream>
#include <random>
#include <fstream>
#include <ros/package.h>

using namespace std;

void writeDataListToCSV(vector<vector<string>> dataList, string time_str) {
	ofstream data_file;

	std::string path = ros::package::getPath("pfc_init");
	data_file.open(path + "/test_results/pfcinit_performance_data_" + time_str + ".csv");

	if (data_file.fail()) {
		cout << "couldn't open file" << endl;
	}

	for (int i = 0; i < dataList.size(); i++) {
		for (int j = 0; j < dataList.at(i).size(); j++) {
			string data_point = dataList.at(i).at(j);
			if (j < dataList.at(i).size() - 1)
				data_file << data_point << ", ";
			else
				data_file << data_point;
		}
		data_file << "\n";
	}
}

cv::Mat l_img;
cv::Mat r_img;
void CB_cam_l(const sensor_msgs::ImageConstPtr &i) {
	try {
		cv::Mat src = cv_bridge::toCvShare(i, sensor_msgs::image_encodings::BGR8)->image;
		l_img = src.clone();
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", i->encoding.c_str());
	}
}
void CB_cam_r(const sensor_msgs::ImageConstPtr &i) {
	try {
		cv::Mat src = cv_bridge::toCvShare(i, sensor_msgs::image_encodings::BGR8)->image;
		r_img = src.clone();
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", i->encoding.c_str());
	}
}

float rng(float min, float max) {
	//https://en.cppreference.com/w/cpp/numeric/random/uniform_int_distribution
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> dist(min, max);
	return dist(gen);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pfc_init_node");

	ros::NodeHandle nh;
	ros::Subscriber l_cam_sub;
	ros::Subscriber r_cam_sub;

	/** Set up subscribers for images */
	l_cam_sub = nh.subscribe("/davinci_endo/left/image_raw", 1, &CB_cam_l);
	r_cam_sub = nh.subscribe("/davinci_endo/right/image_raw", 1, &CB_cam_r);

	/** Get camera intrinsics */
	sensor_msgs::CameraInfo l_inf = *(
		ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/davinci_endo/left/camera_info", nh)
	);
	sensor_msgs::CameraInfo r_inf = *(
		ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/davinci_endo/right/camera_info", nh)
	);
	cv::Mat P_l = cv::Mat(3, 4, CV_64FC1, (void *)l_inf.P.data());
	cv::Mat P_r = cv::Mat(3, 4, CV_64FC1, (void *)r_inf.P.data());

	/* Configure csv key (first row in the csv) */
	int max_cand_pts = 30;
	vector<string> csv_key_base
		{
			"time", "loc_err", "rot_err"
			//, "est_loc_x", "est_loc_y", "est_loc_z", "est_rot_x", "est_rot_y", "est_rot_z", "est_rot_w",
		};
	vector<string> csv_key
		{
			"true_loc_x", "true_loc_y", "true_loc_z", "true_rot_x", "true_rot_y", "true_rot_z", "true_rot_w",
			"yaw_min", "yaw_max", "yaw_inc",
			"roll_min", "roll_max", "roll_inc",
			"pitch_min", "pitch_max", "pitch_inc",
			"z_min", "z_max", "z_inc", "num_cand_pts"
		};
	for (int i = 0; i < max_cand_pts; i++) {
		csv_key.insert(csv_key.end(), csv_key_base.begin(), csv_key_base.end());
	}
	vector<vector<string>> all_results;
	all_results.insert(all_results.begin(), csv_key);

	/* Get current time as string for naming test file */
	auto now = std::chrono::system_clock::now();
	auto in_time_t = std::chrono::system_clock::to_time_t(now);
	std::stringstream ss;
	ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%X");
	string time = ss.str();

	/* Needle position boundaries */
	float x_min = -0.045, x_max = 0.045;
	float y_min = -0.035, y_max = 0.02;
	float z_min = 0.07, z_max = 0.18;

	/* Generate set of random poses */
	int num_poses = 50;
	vector<geometry_msgs::Pose> poses;
	for (int i = 0; i < num_poses; i++) {
		geometry_msgs::Pose test_pose;
		test_pose.position.x = rng(x_min, x_max);
		test_pose.position.y = rng(y_min, y_max);
		test_pose.position.z = rng(z_min, z_max);

		//http://planning.cs.uiuc.edu/node198.html
//		float u = rng(0, 1), v = rng(0, 1), w = rng(0, 1);
//		test_pose.orientation.y = sqrt(1 - u)*cos(2*M_PI*v);
//		test_pose.orientation.z = sqrt(u)*sin(2*M_PI*w);
//		test_pose.orientation.w = sqrt(u)*cos(2*M_PI*w);
//		test_pose.orientation.x = sqrt(1 - u)*sin(2*M_PI*v);

		// For testing only one rotation
		 double radians = pfc::deg2rad * rng(0,360);
		 Eigen::Quaternionf q;
		 q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) //roll
		 	* Eigen::AngleAxisf(radians, Eigen::Vector3f::UnitY()) //pitch
		 	* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()); //yaw
		 test_pose.orientation.x = q.x();
		 test_pose.orientation.y = q.y();
		 test_pose.orientation.z = q.z();
		 test_pose.orientation.w = q.w();

		poses.push_back(test_pose);
	}

	// Loop over all options of some variable we want to test performance of
	// for(int pitch_inc = 5; pitch_inc <= 35; pitch_inc+=5)
	// {
	// 	cout <<pitch_inc << endl;
	// 	if(cand_pts == 6)

	// The number of candidate points to generate
	// NOTE: Should be equal to max_cand_pts unless you're testing variations
	// in candidate point numbers
	int cand_pts = max_cand_pts;

	// Run initialization on all poses
	for (int i = 0; i < num_poses; i++) {
		cout << i << endl;

		// Update needle pose in Gazebo sim
		gazebo_msgs::ModelState modelstate;
		modelstate.model_name = (string)"needle";
		modelstate.reference_frame = (string)"world";
		modelstate.pose = poses.at(i);
		ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
		gazebo_msgs::SetModelState setmodelstate;
		setmodelstate.request.model_state = modelstate;
		client.call(setmodelstate);

		// Get Simulation Needle Pose
		gazebo_msgs::ModelStates model_states = *(
			ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states", nh)
		);

		geometry_msgs::Pose needle_pose;
		for(int n =  0; n < model_states.name.size(); n++){
			if (model_states.name[n] == "needle"){
				needle_pose = model_states.pose[n];
			}
		}

		// Convert geometry_msgs::Pose to NeedlePose
		Eigen::Quaternionf
			q(needle_pose.orientation.w,
			  needle_pose.orientation.x,
			  needle_pose.orientation.y,
			  needle_pose.orientation.z);
		cv::Point3d loc(needle_pose.position.x, needle_pose.position.y, needle_pose.position.z);
		NeedlePose true_pose(loc, q);

		ros::spinOnce();

		/*
		// Display retrieved images
		// cv::namedWindow("l");
		// cv::imshow("l", l_img);
		// cv::waitKey(0);

		// cv::namedWindow("r");
		// cv::imshow("r", r_img);
		// cv::waitKey(0);
		*/

		/* Configure Match Parameters */
		pfc::match_params params = {
			0, 1, 30, //yaw
			0, 360, 30, //pitch
			0, 1, 30, //roll
			0.07, 0.18, 0.01, //z
			cand_pts, // # candidate points to return
			10, // # points in needle line
			P_l,
			P_r
		};

		/** Create and run initializer */
		PfcInitializer pfc_init(P_l, P_r, l_img, r_img, params);
		pfc_init.run(false, true, true_pose);

		// Write test data to csv file
		vector<string> results = pfc_init.getResultsAsVector(true_pose);

		// Add arbitrary large number for all the remaining candidate points not collected for this run
		int diff = 3 * (max_cand_pts - cand_pts);
		vector<string> addtl(diff, "100000");
		results.insert(results.end(), addtl.begin(), addtl.end());
		all_results.push_back(results);
	}
	// }

	writeDataListToCSV(all_results, time);

	return 0;
}

