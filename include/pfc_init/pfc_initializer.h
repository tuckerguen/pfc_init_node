#include "needle_image.h"
#include "needle_template.h"
#include "template_match.h"
#include "needle_pose.h"
#include "pfc_initializer_constants.h"
#include <queue>
#include <vector>

/**
 * @brief Class to localize needle from stereo images
 */
class PfcInitializer
{
private:
    /**
     * @brief Computes pose of needle from the left and right stereo images
     */
    void computeNeedlePose(bool multi_thread);

    /**
     * @brief Prints results of initilization to console
     */
    void displayResults(int pose_id);

public:
    //Camera Projection Matrices
    // Left camera 
    cv::Mat P_l; 
    // Right camera 
    cv::Mat P_r;
    
    /**
     * @brief Left stereo image
     */
    NeedleImage l_img;

    /**
     * @brief Right stereo image
     */
    NeedleImage r_img;

    /**
     * @brief Template used for template matching during localization
     */
    NeedleTemplate left_templ;
    NeedleTemplate right_templ;

    /**
     * @brief Template match data for left image
     */
    vector<TemplateMatch> l_matches;

    /**
     * @brief Template match data for right image
     */
    vector<TemplateMatch> r_matches;

    /**
     * @brief Computed needle pose
     */
    std::vector<NeedlePose> poses;

    /**
     * @brief Constructor
     * TODO: Doc this once we decide on fields
     */
    PfcInitializer(cv::Mat& P_l, cv::Mat& P_r, cv::Mat& l_img, cv::Mat& r_img, pfc::match_params params)
        : P_l(P_l), P_r(P_r), l_img(l_img), r_img(r_img), left_templ(params, true), right_templ(params, false) 
    {}

    /**
     * @brief Runs the initializer, stores the needle pose in initializer
     * 
     * @param print_results If function should print results to console
     * @param multi_thread Use threaded version of match
     */
    void run(bool print_results, bool multi_thread, int pose_id);

    /**
     * @brief Returns the results of initialization as a vector of vectors
     * Each sub vector is a candidate point
     */
    std::vector<std::vector<string>> getResultsAsVector(int pose_id);
};