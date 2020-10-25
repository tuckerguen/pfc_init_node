#include <opencv2/core.hpp>
#include <opencv2/cudaimgproc.hpp>
#include "needle_template.h"
#include "pfc_initializer_constants.h"

using namespace std;

void NeedleTemplate::GenerateTemplate(double z, double r, double p, double y)
{
    // Array of needle points
    cv::Point2d needle_arc[resolution + 1];

    // Projection Matrix
    cv::Mat projection = left ? params.P_l : params.P_r;

    // Transformation Matrix
    // Formula for general rotation matrix from: https://en.wikipedia.org/wiki/Rotation_matrix#General_rotations
    cv::Mat transform = (cv::Mat_<double>(4,4) <<
        cos(y)*cos(p), cos(y)*sin(p)*sin(r)-sin(y)*cos(r), cos(y)*sin(p)*cos(r)+sin(y)*sin(r),  0.0,
        sin(y)*cos(p), sin(y)*sin(p)*sin(r)+cos(y)*cos(r), sin(y)*sin(p)*cos(r)-cos(y)*sin(r),  0.0,
        -sin(p),                      cos(p)*sin(r),                        cos(p)*cos(r),        z,
        0.0,                                  0.0,                                  0.0,        1.0);

    // To determine bounding box of needle for cropping
    double leftmost=640, rightmost=0, upmost=480, downmost=0;
    double buffer = 5;

    // Calculate the transformed origin
    cv::Mat needle_arc_origin = (cv::Mat_<double>(4,1) << 0, 0, 0, 1 );
    origin = CalcUVPoint(needle_arc_origin, transform, projection);

    // Calculate transformed arc points
    for(int i = 0; i <= resolution; i++)
    {
        double turn_amt = i * M_PI / resolution;

        cv::Mat needle_arc_pt = (cv::Mat_<double>(4,1) <<
            pfc::radius * cos(turn_amt),
            pfc::radius * sin(turn_amt),
            0.0,
            1.0
        );

        needle_arc[i] = CalcUVPoint(needle_arc_pt, transform, projection);

        // Track the needle boundaries
        if(needle_arc[i].x < leftmost)
            leftmost = needle_arc[i].x;
        if(needle_arc[i].x > rightmost)
            rightmost = needle_arc[i].x;
        if(needle_arc[i].y < upmost)
            upmost = needle_arc[i].y;
        if(needle_arc[i].y > downmost)
            downmost = needle_arc[i].y;
    }

    // Draw into template
    cv::Mat templ(cv::Size(640, 480), CV_8U, cv::Scalar(0));
    for(int i = 1; i <= resolution; i++)
    {
        cv::Point2d p1 = needle_arc[i-1];
        cv::Point2d p2 = needle_arc[i];

        cv::line(templ, p1, p2, cv::Scalar(255), 2, 8, 0);
    }

    // Crop image to only include the needle
    upmost = floor(upmost); leftmost = floor(leftmost); 
    downmost = ceil(downmost);  rightmost = ceil(rightmost);
    cv::Range rows(upmost-buffer, downmost+buffer);
    cv::Range cols(leftmost-buffer, rightmost+buffer);
    
    templ = templ(rows, cols);
    image = templ;
    //Adjust origin to be relative to new size
    origin.x -= leftmost;
    origin.y -= upmost;
    
    // TODO: Better way to make template compatible with edge detection?
    // cv::Mat detected_edges;
    // // Blur the image before edge detection
    // cv::blur( templ, detected_edges,  cv::Size(3,3) );
    // // Run canny edge detection
    // cv::Canny( detected_edges, image, 40, 120, 3);

    // cout << z << ", " << pfc::rad2deg*r << ", " << pfc::rad2deg*p << ", " << pfc::rad2deg*r << endl;
//     cv::namedWindow("templ" + to_string(y)+ to_string(p)+ to_string(r));
//     cv::imshow("templ" + to_string(y)+ to_string(p)+ to_string(r), image);
}

cv::Point2d CalcUVPoint(const cv::Mat& p, const cv::Mat& transform, const cv::Mat& projection)
{
    //Transform based on needle origin location
    cv::Mat needle_arc_pt_tf = transform * p;

    //Transform into u, v
    cv::Mat needle_arc_pt_uv = projection * needle_arc_pt_tf;

    // Divide out homogeneous coordinate
    double u = needle_arc_pt_uv.at<double>(0) / needle_arc_pt_uv.at<double>(2);
    double v = needle_arc_pt_uv.at<double>(1) / needle_arc_pt_uv.at<double>(2);

    return cv::Point2d(u,v);
}