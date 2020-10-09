#ifndef PFCINIT_CONSTANTS_H
#define PFCINIT_CONSTANTS_H

#include <opencv2/core.hpp>

namespace pfc {
    //Degree 2 radians conversion constant
    const double deg2rad = M_PI / 180.0;
    const double rad2deg = 180.0 / M_PI;

    //HSV Filtering Parameters
    const int low_h = 0, high_h = 360;
    const int low_s = 0, high_s = 0;
    const int low_v = 0, high_v = 140;

    //Match drawing constants
    const int line_weight = 1;
    const int line_type = 8;
    const int shift = 0;

    // Template Generation, needle radius
    const double radius = 0.0128;

    // Kernel size for gaussian blur of images
    const auto kernel_size = cv::Size(3, 3);

    struct match_params{
        // Template Match Rotation Parameters
        double min_yaw;
        double max_yaw;
        double yaw_inc;
        double min_pitch;
        double max_pitch;
        double pitch_inc;
        double min_roll;
        double max_roll;
        double roll_inc;

        double min_z;
        double max_z;
        double z_inc; //% scale to increase by on each iteration

        int num_matches; // The top n matches to keep as candidate points
        int resolution; // # points in needle line

        cv::Mat P_l;
        cv::Mat P_r;
    };
}

#endif
