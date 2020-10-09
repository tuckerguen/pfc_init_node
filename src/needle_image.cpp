#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "needle_image.h"
#include "pfc_initializer_constants.h"

using namespace std;

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
    // cv::blur( raw, detected_edges, kernel_size );
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