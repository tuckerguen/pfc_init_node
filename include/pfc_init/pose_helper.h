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
 * @brief draws needle origin on image, given match info and rotated/scaled template
 * 
 * @param img Image to draw the origin on
 * @param match Match object used to match template to img
 * @param color Color of the origin point to be drawn
 * @param templ The needle template used in matching
 */
void drawNeedleOrigin(cv::Mat& img, const cv::Point2d& needle_origin, const cv::Scalar& color);

/**
 * @brief Prints and returns location and orientation error between given pose and ground truth
 * 
 * @param pose The experimental pose to compare to truth
 * @param pose_id Id number used to lookup ground truth from csv
 * @param print If function should print score to console
 */
vector<double> scorePoseEstimation(NeedlePose est_pose, NeedlePose true_pose, bool print);

#endif