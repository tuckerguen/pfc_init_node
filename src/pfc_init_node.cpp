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


void run(PfcInitializer pfc_init, ros::NodeHandle& nh)
{
	// Collect the newest images
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(ros::topic::waitForMessage<sensor_msgs::Image>("/davinci_endo/left/image_raw", nh));
		pfc_init.l_img.image = cv_ptr->image;
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from encoding to 'bgr8' on left cam.");
	}
	try {
		cv_ptr = cv_bridge::toCvCopy(ros::topic::waitForMessage<sensor_msgs::Image>("/davinci_endo/right/image_raw", nh));
		pfc_init.r_img.image = cv_ptr->image;
	}
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from encoding to 'bgr8' on right cam.");
	}

	cv::namedWindow("l");
	cv::imshow("l", pfc_init.l_img.image);
	cv::waitKey(0);

	cv::namedWindow("r");
	cv::imshow("r", pfc_init.r_img.image);
	cv::waitKey(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pfc_init_node");

	ros::NodeHandle nh;

    // ros::Subscriber l_cam_sub;
    // ros::Subscriber r_cam_sub;

	//Get camera intrinsics
	sensor_msgs::CameraInfo l_inf = *(
		ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/davinci_endo/left/camera_info", nh)
	);
	sensor_msgs::CameraInfo r_inf = *(
		ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/davinci_endo/right/camera_info", nh)
	);

	gazebo_msgs::ModelStates model_states = *(
		ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states", nh)
	);

	geometry_msgs::Pose needle_pose = model_states.pose[2];
	Eigen::Quaternionf q(needle_pose.orientation.w, needle_pose.orientation.x, needle_pose.orientation.y, needle_pose.orientation.z);
	cout << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << endl;
cv::Point3d loc(needle_pose.position.x, needle_pose.position.y, needle_pose.position.z);
	NeedlePose true_pose(loc, q);

    // Convert camera intrinsics to opencv mats
	cv::Mat P_l = cv::Mat(3, 4, CV_64FC1, (void *) l_inf.P.data());
    cv::Mat P_r = cv::Mat(3, 4, CV_64FC1, (void *) r_inf.P.data());

    // Define image mats
    cv::Mat l_img;
    cv::Mat r_img;

    //Get initial image
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(ros::topic::waitForMessage<sensor_msgs::Image>("/davinci_endo/left/image_raw", nh));
		l_img = cv_ptr->image;
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from encoding to 'bgr8' on left cam.");
	}
	try {
		cv_ptr = cv_bridge::toCvCopy(ros::topic::waitForMessage<sensor_msgs::Image>("/davinci_endo/right/image_raw", nh));
		r_img = cv_ptr->image;
	}
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from encoding to 'bgr8' on right cam.");
	}

	cv::namedWindow("l");
	cv::imshow("l", l_img);
	cv::waitKey(0);

	cv::namedWindow("r");
	cv::imshow("r", r_img);
	cv::waitKey(0);

    pfc::match_params params = {
        0, 360, 10, //yaw
        cv::Range(0,90), 10, //pitch
        cv::Range(0,90), 10, //roll
        0.09, 0.18, 0.01, //z
        5, // # candidate points to return
        10, // # points in needle line
		P_l,
		P_r
    };

	PfcInitializer pfc_init(P_l, P_r, l_img, r_img, params);
	cout << "created pfc_init object" << endl;
	cout << true_pose.location << endl;
	cout << true_pose.getEulerAngleOrientation() << endl;
	// cout << true_pose.getQuaternionOrientation() << endl;
    pfc_init.run(true, true, true_pose);


    return 0;
}
