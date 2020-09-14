#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include "pfc_initializer.h"


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

    ros::Subscriber l_cam_sub;
    ros::Subscriber r_cam_sub;


	//Get camera intrinsics
	sensor_msgs::CameraInfo l_inf = *(
		ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/davinci_endo/left/camera_info", nh)
	);
	sensor_msgs::CameraInfo r_inf = *(
		ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/davinci_endo/right/camera_info", nh)
	);


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
    pfc_init.run(true, true, 0);

    return 0;
}
