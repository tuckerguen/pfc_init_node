#ifndef PFCINIT_CONSTANTS_H
#define PFCINIT_CONSTANTS_H

#include <opencv2/core.hpp>

namespace pfc {
    //Template parameters
    const std::string templ_path = "../imgs/raw/0_l_c_fatty.png";
    const cv::Rect2i initial_rect(287,205,105,56);
    const cv::Point2d origin(52,4);
    const double initial_rotation = -176.6537;

    // Template Match Rotation Parameters
    const double min_rotation = 170;
    const double max_rotation = 180;   //Max number of degrees to rotate template
    const double rotation_increment = 1; //Number of degrees to rotate template each iteration

    // Template Match Scaling Parameters
    const int min_scale = 96;         //minimum template scale to try to match (in %)
    const int max_scale = 102;        //maximum template scale to try to match (in %)
    const double scale_increment = 3; //% scale to increase by on each iteration

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

    struct match_params{
        // Template Match Rotation Parameters
        // TODO: Change to not use range (is meant to be used as range of matrices)
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


    //Dataset info
    const int num_poses = 10;
    const int num_img_types = 4;
    const std::vector<std::string> img_types = {"fatty", "marked", "red", "tan"};
    const std::vector<double> min_rot = {
        150, 140, 159, 212, 212, 248, 174, 198, 165, 89
    };
    const std::vector<double> max_rot = {
        190, 145, 164, 217, 217, 253, 179, 203, 170, 94
    };
    const std::vector<int> min_scl = {
        80, 98, 135, 105, 90, 90, 190, 107, 139, 116
    };
    const std::vector<int> max_scl = {
        120, 102, 145, 115, 100, 100, 200, 117, 149, 126
    };

}

#endif
