#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/sfm/triangulation.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "template_match.h"
#include "pose_helper.h"
#include "csv_reader.h"
#include "needle_pose.h"
#include "pfc_initializer_constants.h"

using namespace std;

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
void drawNeedleOrigin(cv::Mat& img, cv::Point2d needle_origin, cv::Scalar color){
    // Draw point
    cv::circle(img,
            needle_origin,
            0, color, 1, 8, 0);
}

//Prints and returns location and orientation error between given pose and ground truth
vector<double> scorePoseEstimation(NeedlePose est_pose, NeedlePose true_pose, bool print)
{
    //Convert to point3d 
    cv::Point3d true_loc = true_pose.location;
    cv::Point3d result_loc = est_pose.location;
    //Calc euclidean dist between points
    double loc_err = cv::norm(result_loc - true_loc);

    // Convert to quaternion
    Eigen::Quaternionf true_orientation = true_pose.getQuaternionOrientation();
    Eigen::Quaternionf result_orientation = est_pose.getQuaternionOrientation();  

    // Calc angle between quaternions in angle-axis representation
    Eigen::Quaternionf qdiff = true_orientation.inverse() * result_orientation;
    double angle_err = 2*atan2(qdiff.vec().norm(), qdiff.w()) * pfc::rad2deg;

    if(print)
    {
        // Format and print results
        printf("(p_err=%fmm, r_err=%fdeg)\n", 1000*loc_err, angle_err);
    }

    // Store and return locationa and orientation errors
    vector<double> results;
    results.push_back(loc_err);
    results.push_back(angle_err);
    return results;
}

//Returns pose data from ground truth pose csv file for given pose id
NeedlePose readTruePoseFromCSV(int pose_id)
{
	CSVReader reader("../positions/needle_positions.csv");
    // Read all rows into vector
    vector<vector<string> > all_pose_data = reader.getData();
    // Select row for pose by pose_id
    vector<string> pose_data = all_pose_data.at(pose_id);

    NeedlePose pose;

    // store location
    pose.location.x = stod(pose_data.at(1));
    pose.location.y = stod(pose_data.at(2));
    pose.location.z = stod(pose_data.at(3));

    //Store orientation
    Eigen::Quaternionf q;
    q.x() = stod(pose_data.at(4));
    q.y() = stod(pose_data.at(5));
    q.z() = stod(pose_data.at(6));
    q.w() = stod(pose_data.at(7));

    pose.setOrientation(q);

    return pose;
}

