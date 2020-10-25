#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/sfm/triangulation.hpp>
#include <eigen3/Eigen/Dense>
#include "pose_helper.h"
#include "needle_pose.h"

//returns 3D location of point given two pixel space points from endoscope stereo camera
cv::Point3d deProjectPoints(const cv::Mat& p_l, const cv::Mat& p_r, const cv::Mat& P_l, const cv::Mat& P_r)
{
    // Create points vector
    vector<cv::Mat> points;
    points.push_back(p_l);
    points.push_back(p_r);

    // Create projection matrices vector
    vector<cv::Mat> projections;
    projections.push_back(P_l);
    projections.push_back(P_r);

    cv::Mat results;
    // Compute 3D location given points and projection matrices
    cv::sfm::triangulatePoints(points, projections, results);
    
    // Format results
    cv::Point3d result;
    result.x = results.at<double>(0);
    result.y = results.at<double>(1);
    result.z = results.at<double>(2);

    return result;
}

// Draws needle origin on image, given match and rotated/scaled template
void drawNeedleOrigin(cv::Mat& img, const cv::Point2d& needle_origin, const cv::Scalar& color){
    // Draw point
    cv::circle(img,
            needle_origin,
            0, color, 1, 8, 0);
}

//Prints and returns location (in mm) and orientation error between given pose and ground truth
vector<double> scorePoseEstimation(NeedlePose est_pose, NeedlePose true_pose, bool print)
{
    //Convert to point3d 
    cv::Point3d true_loc = true_pose.location;
    cv::Point3d result_loc = est_pose.location;

    //Calc euclidean dist between points
    double loc_err = 1000*cv::norm(result_loc - true_loc);

    // Convert to quaternion
    Eigen::Quaterniond true_orientation = true_pose.getQuaternionOrientation();
    Eigen::Quaterniond result_orientation = est_pose.getQuaternionOrientation();
    
    // Calc angle between quaternions in angle-axis representation
    Eigen::Quaterniond qdiff = true_orientation.inverse() * result_orientation;
    double angle_err = 2*atan2(qdiff.vec().norm(), qdiff.w()) * pfc::rad2deg;

    if(print)
    {
        // Format and print results
        printf("(p_err=%fmm, r_err=%fdeg)\n", loc_err, angle_err);
    }

    // Store and return locationa and orientation errors
    vector<double> results;
    results.push_back(loc_err);
    results.push_back(angle_err);
    return results;
}

double constrainAngle(double x, bool is_deg) {
	double min = is_deg ? 180 : M_PI;
	double max = is_deg ? 360 : 2*M_PI;
	x = fmod(x + min, max);
	if (x < 0)
		x += max;
	return x - min;
}

Eigen::Vector3d constrainVector(Eigen::Vector3d v, bool is_deg){
	return Eigen::Vector3d {
		constrainAngle(v.x(), is_deg),
		constrainAngle(v.y(), is_deg),
		constrainAngle(v.z(), is_deg)
	};
}
