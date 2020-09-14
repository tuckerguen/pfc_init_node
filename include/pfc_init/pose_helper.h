#ifndef POSE_HELPER_H
#define POSE_HELPER_H

#include <opencv2/core.hpp>
#include "template_match.h"
#include "needle_template.h"
#include "needle_pose.h"

/**
 * @brief Returns 3D location of point given two pixel space points from endoscope stereo camera
 * 
 * @param p_l Location of point in left image
 * @param p_r Location of point in right image
 */
cv::Point3d deProjectPoints(const cv::Mat& p_l, const cv::Mat& p_r, const cv::Mat& P_l, const cv::Mat& P_r);

/**
 * @brief Returns coordinate location of needle in template after rotation and scaling transformation
 * 
 * @param angle Template was rotated by
 * @param scale Template was scaled to
 * @param templ Template used for match (gives initial origin and template size)
 */
cv::Mat getRotatedOrigin(double angle, double scale, const NeedleTemplate* templ);

/**
 * @brief draws needle origin on image, given match info and rotated/scaled template
 * 
 * @param img Image to draw the origin on
 * @param match Match object used to match template to img
 * @param color Color of the origin point to be drawn
 * @param templ The needle template used in matching
 */
void drawNeedleOrigin(cv::Mat& img, cv::Point2d needle_origin, cv::Scalar color);

/**
 * @brief Prints and returns location and orientation error between given pose and ground truth
 * 
 * @param pose The experimental pose to compare to truth
 * @param pose_id Id number used to lookup ground truth from csv
 * @param print If function should print score to console
 */
vector<double> scorePoseEstimation(NeedlePose pose, int pose_id, bool print);

/**
 * @brief Returns pose data from ground truth pose csv file for given pose id
 * 
 * @param pose_id The id number of the pose data to return
 */
NeedlePose readTruePoseFromCSV(int pose_id);

#endif