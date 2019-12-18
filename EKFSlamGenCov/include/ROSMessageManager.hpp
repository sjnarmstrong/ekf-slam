/*
 * EKFSlam.h
 *
 *  Created on: 08 Nov 2018
 *      Author: sholto
 */

#ifndef ROS_MESSAGE_MANAGER_H_
#define ROS_MESSAGE_MANAGER_H_

#include "EKFSlam.hpp"

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <string>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <Eigen/Core>
//#include <Eigen/Dense>

class ROSMessageManager{
public:
    inline ROSMessageManager(int argc, char **argv){
        br = tf::TransformBroadcaster();
    }
    
    tf::Transform currentTransform;
    tf::Quaternion q;
    Eigen::AngleAxisf qeig;
    tf::TransformBroadcaster br;
    std::ofstream output_file;

    void updateStateAndSendMessages(EKFSlam slamAlg, std::string timeStamp);
    void initOutputFile(std::string filename, std::string forTag);
    void writeTransform(std::string timeStamp);
    void closeFile();
};

#endif