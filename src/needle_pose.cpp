#include <eigen3/Eigen/Dense>
#include <iostream>
#include <utility>
#include "needle_pose.h"
#include "pfc_initializer_constants.h"

using namespace std;

//Return the orientation in quaternion representation
Eigen::Quaternionf NeedlePose::getQuaternionOrientation()
{
    // convert orientation from degrees to radians
    double roll_radians = pfc::deg2rad * orientation.x();
    double pitch_radians = pfc::deg2rad * orientation.y();
    double yaw_radians = pfc::deg2rad * orientation.z();

    // Convert euler angles to quaternion
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(roll_radians, Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(pitch_radians, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(yaw_radians, Eigen::Vector3f::UnitZ());
    
    return q;
}

// Format and print the location and orientation
void NeedlePose::print()
{
    Eigen::Quaternionf q = getQuaternionOrientation();
    printf("(x,y,z)=(%f,%f,%f)\n(r,p,y)=(%f,%f,%f)\n", location.x, location.y, location.z, orientation.x(), orientation.y(), orientation.z());
}

NeedlePose::NeedlePose(const cv::Point3d &location, Eigen::Vector3f orientation) :
        location(location), orientation(std::move(orientation))
{}
