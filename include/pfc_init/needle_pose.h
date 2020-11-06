#ifndef NEEDLE_POSE_H
#define NEEDLE_POSE_H

#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>
#include <utility>
#include "pfc_initializer_constants.h"

using namespace std;

/**
 *  @brief Represents location and orientation pose of the needle
 * 
 *  @todo is there a reason we chose to use opencv and eigen as opposed to one or the other?
 */
class NeedlePose {
 private:
  /**
   *  @brief Euler angle orientation of the needle (radians)
   */
  Eigen::Vector3d orientation;

 public:
  double loc_err{};
  double rot_err{};

  bool operator<(const NeedlePose &np) const {
	  return rot_err < np.rot_err;
  }

  /**
   * @brief Cartesian 3D location of the needle (meters)
   */
  cv::Point3d location;

  /**
   * @brief Constructor
   *
   * @param location 3D location (meters)
   * @param orientation Euler angle orientation (radians)
   */
  NeedlePose(const cv::Point3d &location, Eigen::Vector3d orientation);

  NeedlePose(const cv::Point3d &location, const Eigen::Quaterniond &q) :
	  location(location) {
	  setOrientation(q);
  }

  /**
   * @brief Default constructor (location=(0,0,0), orientation=(0,0,0))
   */
  NeedlePose() :
	  location(cv::Point3d(0, 0, 0)), orientation(Eigen::Vector3d(0, 0, 0)) {}

  /**
   * @brief Set orientation
   *
   * @param new_orientation New orientation in euler angle representation (radians)
   */
  void setOrientation(Eigen::Vector3d new_orientation);

  /**
   * @brief Set orientation
   *
   * @param q Orientation in quaternion representation
   */
  void setOrientation(Eigen::Quaterniond q);

  /**
   * @brief Returns orientation in euler angle representation (radians)
   */
  Eigen::Vector3d getEulerAngleOrientation();

  /**
   * @brief Returns orientation in quaternion representation
   */
  Eigen::Quaterniond getQuaternionOrientation();

  /**
   * @brief Prints summary of pose to console
   *
   * @todo maybe change to toString() and allow user to print it
   */
  void print();
};

#endif