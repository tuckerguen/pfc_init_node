#ifndef NEEDLE_POSE_H
#define NEEDLE_POSE_H

#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>
#include "pfc_initializer_constants.h"

using namespace std;

/**
 *  @brief Represents location and orientation pose of the needle
 * 
 *  @todo is there a reason we chose to use opencv and eigen as opposed to one or the other?
 */
class NeedlePose 
{
private:
    /**
     *  @brief Euler angle orientation of the needle (degrees)
     */
    Eigen::Vector3f orientation;

public:
    double loc_err;
    double rot_err;

    bool operator< (const NeedlePose &np) const {
        return loc_err < np.loc_err;
    }
    /**
     * @brief Cartesian 3D location of the needle (meters)
     */
    cv::Point3d location;
    
    /**
     * @brief Constructor
     * 
     * @param location 3D location (meters)
     * @param orientation Euler angle orientation (degreees)
     */
    NeedlePose(cv::Point3d location, Eigen::Vector3f orientation) :
        location(location), orientation(orientation) 
    {}

    NeedlePose(cv::Point3d location, Eigen::Quaternionf q) :
        location(location)
    {
        setOrientation(q);
    }

    /**
     * @brief Default constructor (location=(0,0,0), orientation=(0,0,0))
     */
    NeedlePose():
        location(cv::Point3d(0,0,0)), orientation(Eigen::Vector3f(0,0,0))
    {}

    /**
     * @brief Set orientation 
     * 
     * @param new_orientation New orientation in euler angle representation (degrees
     */
    void setOrientation(Eigen::Vector3f new_orientation) { orientation = new_orientation;}
    
    /**
     * @brief Set orientation 
     * 
     * @param q Orientation in quaternion representation
     */
    void setOrientation(Eigen::Quaternionf q) { 
        // convert to euler angles
        orientation = q.toRotationMatrix().eulerAngles(0, 1, 2);
        // convert from radians to degrees
        orientation = orientation * pfc::rad2deg;
    }

    /**
     * @brief Returns orientation in euler angle representation (degrees)
     */
    Eigen::Vector3f getEulerAngleOrientation() { return orientation; };

    /**
     * @brief Returns orientation in quaternion representation
     */
    Eigen::Quaternionf getQuaternionOrientation();

    /**
     * @brief Prints summary of pose to console
     * 
     * @todo maybe change to toString() and allow user to print it
     */
    void print();

    

};

#endif