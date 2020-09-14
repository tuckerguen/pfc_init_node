#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "needle_image.h"
#include "pfc_initializer_constants.h"

using namespace std;

// TODO: Move to pfc_init_constants
// Canny Filtering Parameters
const cv::Size gauss_filter_size = cv::Size(3,3);
const int low_threshold = 40;
const int max_low_threshold = 120;
const int kernel_size = 3;

// constructor
NeedleImage::NeedleImage(cv::Mat& img)
{
    //Load needle image at path
    raw = img;

    //preprocess
    filterRaw();    
}

//Filters the image for needle
void NeedleImage::filterRaw()
{
    cv::Mat detected_edges;
    // Blur the image before edge detection
    // cv::blur( raw, detected_edges, gauss_filter_size );
    // // Run canny edge detection
    // cv::Canny( detected_edges, image, low_threshold, max_low_threshold, kernel_size);
    cv::Mat img_HSV;
    // convert to HSV
    cv::cvtColor(raw, img_HSV, cv::COLOR_BGR2HSV);
    // filter by HSV values 
    cv::inRange(img_HSV, cv::Scalar(pfc::low_h, pfc::low_s, pfc::low_v), cv::Scalar(pfc::high_h, pfc::high_s, pfc::high_v), image);
    // Display image for debugging
    // cv::namedWindow("edge detected");
    // cv::imshow("edge detected",image);
    // cv::waitKey(0);
}

//TODO: Make this a member function
//rotate an image by angle degrees
void rotate(const cv::Mat &src, cv::Mat &dst, double angle)
{
    // get center of original img
    cv::Point2d center((src.cols - 1) / 2.0, (src.rows - 1) / 2.0);

    // get rotation matrix for rotating the image around its center in pixel coordinates
    cv::Mat rot = getRotationMatrix2D(center, angle, 1.0);
    // calculate dimensions of rotated image, center not relevant
    cv::Rect2d bbox = cv::RotatedRect(cv::Point2d(), src.size(), angle).boundingRect2f();

    // apply translation to rotation matrix to shift center 
    rot.at<double>(0, 2) += bbox.width / 2.0 - src.cols / 2.0;

    rot.at<double>(1, 2) += bbox.height / 2.0 - src.rows / 2.0;
    //apply matrix transformation
    warpAffine(src, dst, rot, bbox.size());
}