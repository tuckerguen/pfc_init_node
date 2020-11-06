#ifndef MATCHER_H
#define MATCHER_H

#include <future>
#include <sys/sysinfo.h>
#include <queue>
#include <cmath>
#include "needle_image.h"
#include "needle_template.h"
#include "template_match.h"
#include "matcher.h"
#include "pose_helper.h"
#include "pfc_initializer_constants.h"

/**
 * @brief Template match template on base image over range of scales and rotations
 * 
 * @param base_img Image to match template to
 * @param templ Needle template object with template to match with and scale/rotation parameters
 */
vector<TemplateMatch> match(const cv::Mat &base_img, NeedleTemplate templ);

/**
 * @brief Equivalent to match, but scale range is divided amongst a number of parallel threads. 
 * The number of threads is the number of useable cpu cores returned by get_nprocs() from sys/sysinfo.h
 * 
 * @param base_img Image to match template to
 * @param templ Needle template object with template to match with and scale/rotation parameters
 */
vector<TemplateMatch> matchThreaded(const cv::Mat &img, NeedleTemplate templ);

/**
 * @brief Run template match and return a match object storing match details
 * 
 * @param img The base image to match template to
 * @param templ The template to match onto the image
 */
TemplateMatch getMatch(const cv::Mat &img, const cv::Mat &templ);

//vector<TemplateMatch> select_top_matches(vector<TemplateMatch>* matches, int num_cand_pts);

vector<TemplateMatch> collect_top_matches(vector<TemplateMatch> matches, int num_cand_pts);

#endif