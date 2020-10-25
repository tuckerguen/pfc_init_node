#ifndef TEMPLATE_MATCH_H
#define TEMPLATE_MATCH_H

#include <opencv2/imgproc.hpp>
#include "pfc_initializer_constants.h"

//A template match
class TemplateMatch 
{
public:
    // We don't really need this since the z location gets computed at the
    // deprojection step. But keep it here for now to compare to the 
    // computed value
    double z;

    double yaw;
    double pitch;
    double roll;
    
    /**
     * @brief Match quality value returned by cv::matchTemplate()
     */    
    double score;


    /**
     * @brief Location of match and template dimensions at match
     */
    cv::Rect rect;

    /**
     * @brief Result image returned by cv::matchTemplate()
     */
    cv::Mat result; 

    /**
     * @brief Template used for match
     */
    cv::Mat templ;

    double ox,oy;
    cv::Point2d origin;

    /**
     * @brief Constructor
     * 
     * @param score Match quality value given to match by cv::matchTemplate()
     */
    TemplateMatch(double z, double yaw, double pitch, double roll, double score) : 
        z(z), yaw(yaw), pitch(pitch), roll(roll), score(score), rect(0,0,0,0), result(), templ() 
    {} 

    /**
     * @brief Constructor
     * 
     * @param score  Match quality value given to match by cv::matchTemplate()
     * @param rect   Rectangle representing bounds of template in matched image
     * @param result Result image returned by cv::matchTemplate()
     * @param templ  Template used in match
     */
    TemplateMatch(double score, cv::Rect2i rect, cv::Mat result, cv::Mat templ) : 
        score(score), rect(rect), result(result), templ(templ) 
    {} 

    /**
     * @brief Default constructor (angle=0, score=-DBL_MAX, scale=1, rect=(0,0,0,0))
     */
    TemplateMatch() : 
        score(-DBL_MAX), z(0.15), rect(0,0,0,0), result() 
    {} 

    // TemplateMatch(TemplateMatch &&) = default;

    /**
     * @brief Returns the match angle in degrees
     */
    cv::Vec3d getAngleDegrees() { return cv::Vec3d(yaw, pitch, roll) * pfc::rad2deg; }
    
    /**
     * @brief Returns the match angle in radians
     */
    cv::Vec3d getAngleRadians() { return cv::Vec3d(yaw, pitch, roll); }

    /**
     * @brief Format and print details of the match
     * 
     * @param name Title to be printed with the match (probably "right" or "left")
     */
    void printMatchSummary(const std::string& name);

    /**
     * @brief Draws the match bounding rectangle on the image
     * 
     * @param img Image to draw the match onto
     * @param color Color of the rectangle
     */
    void drawOnImage(cv::Mat& img, const cv::Scalar& color) const;
};

class TemplateMatchComparator 
{ 
public: 
    int operator() (const TemplateMatch& t1, const TemplateMatch& t2) 
    { 
        return t1.score > t2.score; 
    } 
}; 

#endif