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

struct pose_bounds {
  float xmin;
  float xmax;
  float ymin;
  float ymax;
  float zmin;
  float zmax;
};

void writeDataListToCSV(vector<vector<string>> dataList, const string &time_str) {
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

vector<vector<string>> create_csv_vec(int num_cand_pts) {
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
	for (int i = 0; i < num_cand_pts; i++) {
		csv_key.insert(csv_key.end(), csv_key_base.begin(), csv_key_base.end());
	}
	vector<vector<string>> all_results;
	all_results.insert(all_results.begin(), csv_key);
	return all_results;
}

string get_time_str() {
	/* Get current time as string for naming test file */
	auto now = std::chrono::system_clock::now();
	auto in_time_t = std::chrono::system_clock::to_time_t(now);
	std::stringstream ss;
	ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%X");
	string time = ss.str();
	return time;
}

/**
 * Generate num_poses needle poses randomly within bounds
 * @param num_poses number poses to generate
 * @param axes vector of logical values for which axes to generate rotations on <roll, pitch, yaw>
 * @return list of geometry_msgs::Pose
 */
vector<geometry_msgs::Pose> generate_poses(int num_poses, pose_bounds pb, vector<bool> axes) {
	vector<geometry_msgs::Pose> poses;
	for (int i = 0; i < num_poses; i++) {
		geometry_msgs::Pose test_pose;
		test_pose.position.x = rng(pb.xmin, pb.xmax);
		test_pose.position.y = rng(pb.ymin, pb.ymax);
//		test_pose.position.z = rng(pb.zmin, pb.zmax);
		test_pose.position.z = 0.12;

		//		//http://planning.cs.uiuc.edu/node198.html
		//		float u = rng(0, 1), v = rng(0, 1), w = rng(0, 1);
		//		test_pose.orientation.y = sqrt(1 - u)*cos(2*M_PI*v);
		//		test_pose.orientation.z = sqrt(u)*sin(2*M_PI*w);
		//		test_pose.orientation.w = sqrt(u)*cos(2*M_PI*w);
		//		test_pose.orientation.x = sqrt(1 - u)*sin(2*M_PI*v);

		// Generate random rpy values from -pi/2 to pi/2
		Eigen::Quaterniond q;
		double roll = rng(-M_PI, M_PI)*axes.at(0);
		double pitch = rng(-M_PI, M_PI)*axes.at(1);
		double yaw = rng(-M_PI, M_PI)*axes.at(2);
		Eigen::Vector3d rpy(roll, pitch, yaw);
		rpy.normalize();

		// Convert to quaternion
		q = Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()) //roll
			*Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) //pitch
			*Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()); //yaw
		test_pose.orientation.x = q.x();
		test_pose.orientation.y = q.y();
		test_pose.orientation.z = q.z();
		test_pose.orientation.w = q.w();

		poses.push_back(test_pose);
	}
	return poses;
}

void set_sim_needle_pose(geometry_msgs::Pose pose, ros::NodeHandle *nh) {
	// Update needle pose in Gazebo sim
	gazebo_msgs::ModelState modelstate;
	modelstate.model_name = (string)"needle";
	modelstate.reference_frame = (string)"world";
	modelstate.pose = pose;
	ros::ServiceClient client = nh->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	gazebo_msgs::SetModelState setmodelstate;
	setmodelstate.request.model_state = modelstate;
	client.call(setmodelstate);
}

geometry_msgs::Pose get_sim_needle_pose(ros::NodeHandle *nh) {
	// Get Simulation Needle Pose
	gazebo_msgs::ModelStates model_states = *(
		ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states", *nh)
	);

	for (int n = 0; n < model_states.name.size(); n++) {
		if (model_states.name[n]=="needle") {
			return model_states.pose[n];
		}
	}
}

void params_to_radians(pfc::match_params *params) {
	params->roll_inc *= pfc::deg2rad;
	params->max_roll *= pfc::deg2rad;
	params->min_roll *= pfc::deg2rad;
	params->pitch_inc *= pfc::deg2rad;
	params->max_pitch *= pfc::deg2rad;
	params->min_pitch *= pfc::deg2rad;
	params->yaw_inc *= pfc::deg2rad;
	params->max_yaw *= pfc::deg2rad;
	params->min_yaw *= pfc::deg2rad;
}

vector<vector<string>> run_on_poses(int max_cand_pts,
									int num_poses,
									vector<geometry_msgs::Pose> poses,
									ros::NodeHandle *nh,
									cv::Mat P_l,
									cv::Mat P_r,
									vector<vector<string>> all_results,
									bool test_candpts,
									bool display_imgs) {
	// Loop over all options of some variable we want to test performance of
	// If not testing variable candidate point numbers
	int start_candpts = 100;
	max_cand_pts = test_candpts ? max_cand_pts : start_candpts;
	for (int cand_pts = start_candpts; cand_pts <= max_cand_pts; cand_pts += 5) {
		cout << cand_pts << endl;

		// Run initialization on all poses
		for (int i = 0; i < num_poses; i++) {
			cout << i << endl;
			set_sim_needle_pose(poses.at(i), nh);
			geometry_msgs::Pose needle_pose = get_sim_needle_pose(nh);


			// Convert geometry_msgs::Pose to NeedlePose
			Eigen::Quaterniond
				q(needle_pose.orientation.w,
				  needle_pose.orientation.x,
				  needle_pose.orientation.y,
				  needle_pose.orientation.z);

			cv::Point3d loc(needle_pose.position.x, needle_pose.position.y, needle_pose.position.z);
			NeedlePose true_pose(loc, q);

			ros::spinOnce();

			if (display_imgs) {
				// Display retrieved images
				cv::namedWindow("l");
				cv::imshow("l", l_img);
				cv::waitKey(0);

				cv::namedWindow("r");
				cv::imshow("r", r_img);
				cv::waitKey(0);
			}

			/* Configure Match Parameters */
			pfc::match_params params = {
				0, 360, 30, //yaw
				0, 360, 30, //pitch
				0, 360, 30, //roll
				0.07, 0.18, 0.01, //z
				cand_pts, // # candidate points to return
				10, // # points in needle line
				P_l,
				P_r
			};
			params_to_radians(&params);

			/** Create and run initializer */
			PfcInitializer pfc_init(P_l, P_r, l_img, r_img, params);
			pfc_init.run(false, true, true_pose);

			// Write test data to csv file
			vector<string> results = pfc_init.getResultsAsVector(true_pose);

			// Add arbitrary large number for all the remaining candidate points not collected for this run
			int diff = 3*(max_cand_pts - cand_pts);
			vector<string> addtl(diff, "100000");
			results.insert(results.end(), addtl.begin(), addtl.end());
			all_results.push_back(results);
		}
	}
	return all_results;
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

	/** Pose Generation Configurations */
	/* Needle position boundaries */
	pose_bounds pb = {
		-0.045, 0.045, //x bounds
		-0.035, 0.02, // y bounds
		0.07, 0.13 // z bounds
	};

	int max_cand_pts = 40; // TODO: Move to input parameter

	/* Configure csv key (first row in the csv) */
	vector<vector<string>> csv_base = create_csv_vec(max_cand_pts);
	string time = get_time_str();

	/* Generate set of random poses */
	int num_poses = 50;
	// roll, pitch, yaw
	vector<bool> axes{true, true, true};
	vector<geometry_msgs::Pose> poses = generate_poses(num_poses, pb, axes);

	// Run and collect data
	vector<vector<string>> all_results = run_on_poses(max_cand_pts, num_poses, poses, &nh, P_l, P_r, csv_base, false, false);

	writeDataListToCSV(all_results, time);

	return 0;
}

