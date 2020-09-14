#ifndef NEEDLE_IMAGE_H
#define NEEDLE_IMAGE_H

#include <opencv2/core.hpp>

/**
 *  @brief Respresents one endoscope image of the needle 
 */
class NeedleImage
{
public:
    /**
     * @brief Original image before preprocessing
     */
    cv::Mat raw;

    /**
     * @brief Image after preprocessing (used for template matching)
     */
    cv::Mat image;  

    /**
     * @brief Path to the image
     */
    std::string path;

    /**
     * @brief Filters needle from the background in the raw image 
     */
    void filterRaw();
    
    /**
     * @brief constructor
     * 
     * @param path Path to the image 
     */
    NeedleImage(cv::Mat& img);

    /**
     * @brief Deconstructor
     */
    ~NeedleImage()
    {};
};

/**
 * @brief Rotates an image counterclockwise by angle degrees
 * 
 * @param src Input image to rotate
 * @param dst Output, rotated image
 * @param angle Number of degrees to rotate
 */
void rotate(const cv::Mat &src, cv::Mat &dst, double angle);

#endif