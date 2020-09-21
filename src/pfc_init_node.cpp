#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/ModelStates.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include "pfc_initializer.h"
#include "pose_helper.h"
#include "needle_pose.h"
#include "csv_reader.h"
#include <gazebo_msgs/SetModelState.h>
#include <vector>
#include <chrono>
#include <ctime>
#include <sstream>
#include <cstdlib>

using namespace std;
// void run(PfcInitializer pfc_init, ros::NodeHandle& nh)
// {
// 	// Collect the newest images
// 	cv_bridge::CvImagePtr cv_ptr;
// 	try {
// 		cv_ptr = cv_bridge::toCvCopy(ros::topic::waitForMessage<sensor_msgs::Image>("/davinci_endo/left/image_raw", nh));
// 		pfc_init.l_img.image = cv_ptr->image;
// 	} catch (cv_bridge::Exception &e) {
// 		ROS_ERROR("Could not convert from encoding to 'bgr8' on left cam.");
// 	}
// 	try {
// 		cv_ptr = cv_bridge::toCvCopy(ros::topic::waitForMessage<sensor_msgs::Image>("/davinci_endo/right/image_raw", nh));
// 		pfc_init.r_img.image = cv_ptr->image;
// 	}
// 	catch (cv_bridge::Exception &e) {
// 		ROS_ERROR("Could not convert from encoding to 'bgr8' on right cam.");
// 	}

// 	cv::namedWindow("l");
// 	cv::imshow("l", pfc_init.l_img.image);
// 	cv::waitKey(0);

// 	cv::namedWindow("r");
// 	cv::imshow("r", pfc_init.r_img.image);
// 	cv::waitKey(0);
// }    

cv::Mat l_img;
cv::Mat r_img;
void CB_cam_l(const sensor_msgs::ImageConstPtr& i){
	try{
		cv::Mat src =  cv_bridge::toCvShare(i, sensor_msgs::image_encodings::BGR8)->image;
		l_img = src.clone();
	} catch(cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", i ->encoding.c_str());
	}
}
void CB_cam_r(const sensor_msgs::ImageConstPtr& i){
	try{
		cv::Mat src =  cv_bridge::toCvShare(i, sensor_msgs::image_encodings::BGR8)->image;
		r_img = src.clone();
	} catch(cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", i ->encoding.c_str());
	}
}

float rng(float min, float max)
{
	return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pfc_init_node");

	ros::NodeHandle nh;
	ros::Subscriber l_cam_sub;
	ros::Subscriber r_cam_sub;

	/** Initialize RNG **/
	srand (static_cast <unsigned> (time(0)));

	/** Set up subscribers for images */
	l_cam_sub = nh.subscribe("/davinci_endo/left/image_raw" , 1, &CB_cam_l);
	r_cam_sub = nh.subscribe("/davinci_endo/right/image_raw", 1, &CB_cam_r);

	/** Get camera intrinsics */
	sensor_msgs::CameraInfo l_inf = *(
		ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/davinci_endo/left/camera_info", nh)
	);
	sensor_msgs::CameraInfo r_inf = *(
		ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/davinci_endo/right/camera_info", nh)
	);
	cv::Mat P_l = cv::Mat(3, 4, CV_64FC1, (void *) l_inf.P.data());
    cv::Mat P_r = cv::Mat(3, 4, CV_64FC1, (void *) r_inf.P.data());

	// Configure csv key (first row in the csv)
	int num_cand_pts = 5;
	vector<string> csv_key_base
	{ 
		"time", "loc_err", "rot_err"
		//, "est_loc_x", "est_loc_y", "est_loc_z", "est_rot_x", "est_rot_y", "est_rot_z", "est_rot_w",
	};    
	vector<string> csv_key
	{
		"true_loc_x", "true_loc_y", "true_loc_z", "true_rot_x", "true_rot_y", "true_rot_z", "true_rot_w"		
	};
	for(int i = 0; i < num_cand_pts; i++)
	{
		csv_key.insert(csv_key.end(), csv_key_base.begin(), csv_key_base.end());
	}

	vector<vector<string>> all_results;
	all_results.insert(all_results.begin(), csv_key);

	// Get current time as string for naming test file
	auto now = std::chrono::system_clock::now();
	auto in_time_t = std::chrono::system_clock::to_time_t(now);
	std::stringstream ss;
	ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%X");
	string time = ss.str();

	float x_min = -0.05, x_max = 0.05;
	float y_min = -0.04, y_max = 0.03;
	float z_min = 0.07, z_max = 0.18;


	int num_trials = 10;
	for(int i = 0; i < num_trials; i++)
	{
		/** Set Simulation Needle Pose */
		geometry_msgs::Pose start_pose;
		start_pose.position.x = rng(x_min, x_max);
		start_pose.position.y = rng(y_min, y_max);
		start_pose.position.z = rng(z_min, z_max);\

		//http://planning.cs.uiuc.edu/node198.html
		float u = rng(0,1), v = rng(0,1), w = rng(0,1);
		// start_pose.orientation.y = sqrt(1-u) * cos(2*M_PI*v);
		// start_pose.orientation.z = sqrt(u) * sin(2*M_PI*w);
		// start_pose.orientation.w = sqrt(u) * cos(2*M_PI*w);
		// start_pose.orientation.x = sqrt(1-u) * sin(2*M_PI*v);
		start_pose.orientation.x = 0;
		start_pose.orientation.y = 0;
		start_pose.orientation.z = 0;
		start_pose.orientation.w = 1;

		gazebo_msgs::ModelState modelstate;
		modelstate.model_name = (string) "needle";
		modelstate.reference_frame = (string) "world";
		modelstate.pose = start_pose;
		// modelstate.twist = start_twist;

		ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
		gazebo_msgs::SetModelState setmodelstate;
		setmodelstate.request.model_state = modelstate;
		client.call(setmodelstate);

		/** Get Simulation Needle Pose */
		gazebo_msgs::ModelStates model_states = *(
			ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states", nh)
		);
		geometry_msgs::Pose needle_pose = model_states.pose[2];
		Eigen::Quaternionf q(needle_pose.orientation.w, needle_pose.orientation.x, needle_pose.orientation.y, needle_pose.orientation.z);
		cv::Point3d loc(needle_pose.position.x, needle_pose.position.y, needle_pose.position.z);
		NeedlePose true_pose(loc, q);

		ros::spinOnce();

		// cv::namedWindow("l");
		// cv::imshow("l", l_img);
		// cv::waitKey(0);

		// cv::namedWindow("r");
		// cv::imshow("r", r_img);
		// cv::waitKey(0);

		/** Configure Match Parameters */
		pfc::match_params params = {
			0, 360, 10, //yaw
			cv::Range(0, 1), 15, //pitch
			cv::Range(0,1), 15, //roll
			0.07, 0.18, 0.01, //z
			num_cand_pts, // # candidate points to return
			10, // # points in needle line
			P_l,
			P_r
		};

		/** Create and run initializer */
		PfcInitializer pfc_init(P_l, P_r, l_img, r_img, params);	
		pfc_init.run(true, true, true_pose);

		// Write test data to csv file
		vector<string> results = pfc_init.getResultsAsVector(true_pose);
		all_results.push_back(results);
	}
	
	writeDataListToCSV(all_results);

    return 0;
}

