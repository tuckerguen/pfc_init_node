#include <eigen3/Eigen/Dense>
#include <iostream>
#include <utility>
#include "needle_pose.h"
#include "pfc_initializer_constants.h"
#include "pose_helper.h"

using namespace std;

void NeedlePose::setOrientation(Eigen::Vector3d new_orientation) {
	orientation = constrainVector(new_orientation, false);
}

void NeedlePose::setOrientation(Eigen::Quaterniond q) {
	// convert to euler angles
	// NOTE: The first value of eulerAngles() is constrained from [0,PI], not [-PI,PI], so
	// converting rpy to quaternion to rpy may yield different rpy values than the original.
	// The rotation will be the same and the conversion to a quaternion will be the same as well.
	orientation = constrainVector(q.toRotationMatrix().eulerAngles(0, 1, 2), false);
}

Eigen::Vector3d NeedlePose::getEulerAngleOrientation() {
	return constrainVector(orientation, false);
}

//Return the orientation in quaternion representation
Eigen::Quaterniond NeedlePose::getQuaternionOrientation() {
	// Convert euler angles to quaternion
	return Eigen::Quaterniond(
		Eigen::AngleAxisd(orientation.x(), Eigen::Vector3d::UnitX())
		*Eigen::AngleAxisd(orientation.y(), Eigen::Vector3d::UnitY())
		*Eigen::AngleAxisd(orientation.z(), Eigen::Vector3d::UnitZ()));
}

// Format and print the location and orientation
void NeedlePose::print()
{
    Eigen::Quaterniond q = getQuaternionOrientation();
    printf("(x,y,z)=(%f,%f,%f)\n(r,p,y)=(%f,%f,%f)\n", location.x, location.y, location.z, orientation.x(), orientation.y(), orientation.z());
}

NeedlePose::NeedlePose(const cv::Point3d &location, Eigen::Vector3d orientation) :
        location(location), orientation(std::move(orientation))
{}
