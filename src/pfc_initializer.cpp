#include <string>
#include <iostream>
#include <queue>
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "pose_helper.h"
#include "matcher.h"
#include "pfc_initializer.h"
#include "needle_template.h"
#include "needle_pose.h"

using namespace std;

// Calculates needle pose from images (primary function for use in particle filter)
// Stores pose in this object
void PfcInitializer::run(bool print_results, bool multi_thread, const NeedlePose& true_pose)
{
    // Start timer
    auto t = (double)cv::getTickCount();

    computeNeedlePose(multi_thread);
    
    //Stop timer
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Time: " << t << " s" << endl;
    time = t;

    if(print_results)
    {
        displayResults(true_pose);
    }
}

// Computes pose of needle from the left and right stereo images
void PfcInitializer::computeNeedlePose(bool multi_thread)
{
    // Perform template match on left and right images
    if(multi_thread)
    {
        l_matches = matchThreaded(l_img.image, left_templ);
        r_matches = matchThreaded(r_img.image, right_templ);
    }
    else
    {
        l_matches = match(l_img.image, left_templ);
        r_matches = match(r_img.image, right_templ);
    }

    if(l_matches.size() != r_matches.size())
    {
        std::cerr << "Left and right matches aren't the same length" << endl;
        exit(0);
    }

    // Compute needle positions for all matches
    for(int i = 0; i < l_matches.size(); i++)
    {
        TemplateMatch *match_l = &l_matches.at(i);
        TemplateMatch *match_r = &r_matches.at(i);

        //Initialize left and right needle pixel location vectors
        cv::Mat p_l(2, 1, CV_64FC1);
        cv::Mat p_r(2, 1, CV_64FC1);

        p_l.at<double>(0) = match_l->origin.x;
        p_l.at<double>(1) = match_l->origin.y;
    
        p_r.at<double>(0) = match_r->origin.x;
        p_r.at<double>(1) = match_r->origin.y;
        
        // Get 3D location of needle
        cv::Point3d location = deProjectPoints(p_l, p_r, P_l, P_r);
        // Get Euler angle orientation
        cv::Vec3d left_orientation = match_l->getAngleDegrees();
        cv::Vec3d right_orientation = match_r->getAngleDegrees();

        // Average out orientation values between left and right calculation
        double avg_yaw = (left_orientation[0] + right_orientation[0]) / 2.0;
        double avg_pitch = (left_orientation[1] + right_orientation[1]) / 2.0;
        double avg_roll = (left_orientation[2] + right_orientation[2]) / 2.0;
        
        Eigen::Vector3f orientation(avg_roll, avg_pitch, avg_yaw);

        // Store location/orientation
        poses.emplace_back(location, orientation);
    }
}

void PfcInitializer::displayResults(const NeedlePose& true_pose)
{
    // Draw matches onto images
    for(int i = 0; i < l_matches.size(); i++)
    {
        TemplateMatch match_l = l_matches.at(i);
        TemplateMatch match_r = r_matches.at(i);
        int color_inc = 255/l_matches.size();

//        cv::namedWindow("l", CV_WINDOW_AUTOSIZE);
//        cv::namedWindow("r", CV_WINDOW_AUTOSIZE);
//        imshow("l" + to_string(i), match_l.templ);
//        imshow("r" + to_string(i),match_r.templ);

//        printf("%f, %f == %f\n", match_l.pitch, match_r.pitch, true_pose.getEulerAngleOrientation().y());

//        cv::waitKey(0);
        // match_l.templ.copyTo(l_img.raw.rowRange(match_l.rect.x,match_l.rect.x+match_l.rect.width), l_img.raw.colRange(match_l.rect.y, match_l.rect.y+match_l.rect.height));
        // match_r.templ.copyTo(r_img.raw.rowRange(match_r.rect.x,match_r.rect.x+match_r.rect.width), r_img.raw.colRange(match_r.rect.y, match_r.rect.y+match_r.rect.height));
        match_l.drawOnImage(l_img.raw, cv::Scalar(i*color_inc, 255-i*color_inc, 180));
        match_r.drawOnImage(r_img.raw, cv::Scalar(i*color_inc, 255-i*color_inc, 180));
        drawNeedleOrigin(l_img.raw, match_l.origin, cv::Scalar(0,255,255)); 
        drawNeedleOrigin(r_img.raw, match_r.origin, cv::Scalar(0,255,255)); 
    }

    for(int i = 0; i < poses.size(); i++)
    {
        cout << "Candidate Point " + to_string(i) << endl;
        poses.at(i).print();
        scorePoseEstimation(poses.at(i), true_pose, true);      
        cout << "----------------------------------------------------------------------" << endl;
    }

    cv::namedWindow("left", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("right", cv::WINDOW_AUTOSIZE);
    cv::imshow("left", l_img.raw);
    cv::imshow("right", r_img.raw);
    cv::waitKey(0);

    cv::destroyAllWindows();
}

vector<string> PfcInitializer::getResultsAsVector(NeedlePose true_pose)
{
    vector<string> results;
    // Add true pose
    results.push_back(to_string(true_pose.location.x));
    results.push_back(to_string(true_pose.location.y));
    results.push_back(to_string(true_pose.location.z));
    Eigen::Quaternionf tq = true_pose.getQuaternionOrientation();
    results.push_back(to_string(tq.x()));
    results.push_back(to_string(tq.y()));
    results.push_back(to_string(tq.z()));
    results.push_back(to_string(tq.w()));

    // Add matching parameters
    results.push_back(to_string(left_templ.params.min_yaw));
    results.push_back(to_string(left_templ.params.max_yaw));
    results.push_back(to_string(left_templ.params.yaw_inc));
    results.push_back(to_string(left_templ.params.min_roll));
    results.push_back(to_string(left_templ.params.max_roll));
    results.push_back(to_string(left_templ.params.roll_inc));
    results.push_back(to_string(left_templ.params.min_pitch));
    results.push_back(to_string(left_templ.params.max_pitch));
    results.push_back(to_string(left_templ.params.pitch_inc));
    results.push_back(to_string(left_templ.params.min_z));
    results.push_back(to_string(left_templ.params.max_z));
    results.push_back(to_string(left_templ.params.z_inc));

    // Add number cand pts
    results.push_back(to_string(left_templ.params.num_matches));

    // Score results
    for(auto & p : poses)
    {
		Eigen::Vector3f o = p.getEulerAngleOrientation();
		Eigen::Vector3f t = true_pose.getEulerAngleOrientation();
//		printf("%f, %f, %f == %f, %f, %f\n", o.x(), o.y(), o.z(), t.x(), t.y(), t.z());
        NeedlePose* pose = &p;
        vector<double> score = scorePoseEstimation(*pose, true_pose, false);
        pose->loc_err = score.at(0);
        pose->rot_err = score.at(1);
    }

    // Sort results
    std::sort(poses.begin(), poses.end());

    for(const auto& pose : poses)
    {
        // Add time
        results.push_back(to_string(time));

        results.push_back(to_string(pose.loc_err));
        results.push_back(to_string(pose.rot_err)); 

        // cout << pose.loc_err << ", " << pose.rot_err << endl;
        //Add location guess
        // results.push_back(to_string(pose.location.x));
        // results.push_back(to_string(pose.location.y));
        // results.push_back(to_string(pose.location.z));

        // //Add orientation guess
        // //Quaternion
        // Eigen::Quaternionf q = pose.getQuaternionOrientation();
        // results.push_back(to_string(q.x()));
        // results.push_back(to_string(q.y()));
        // results.push_back(to_string(q.z()));
        // results.push_back(to_string(q.w()));

        //Euler Angles orientation guess
        // Eigen::Vector3f e = pose.getEulerAngleOrientation();
        // results.at(i).push_back(to_string(e.x()));
        // results.at(i).push_back(to_string(e.y()));
        // results.at(i).push_back(to_string(e.z()));

    }

    return results;
}