/**
  * @file handrotation.cpp
  * transform handmodel to the postion of point cloud.
  * @author Hao Xu
*/


#include "handrotation.h"
#include <iostream>

#include <pcl/pcl_macros.h>
#include <stdio.h>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
#include <ros/ros.h>
#include <string>

// for handmodel
using namespace Eigen;
using namespace KDL;
using namespace std;
//for depth image

using namespace pcl;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;


 handrotation::handrotation(KDL::Frame cartpos, float roll, float pitch, float yaw, float base_x, float base_y, float base_z, Eigen::Matrix4f rotation_t)
{

    Eigen::Affine3f test;
    test = getTransformation (0,0,0,roll, pitch, yaw );
    rotation_t = test.matrix();

    Eigen::Vector3f cartpos_t;
    cartpos_t(0) = cartpos(0,3) + base_x;
    cartpos_t(1) = cartpos(1,3) + base_y;
    cartpos_t(2) = cartpos(2,3) + base_z;

    Eigen::Vector4f t;
    t = cartpos_t.homogeneous();
    t = rotation_t.inverse() * t;
    rotation_t.col(3) = t ;

    //return (rotation_thum4);

}


Eigen::Matrix4f handrotation::gethandrotation(KDL::Frame cartpos, float roll, float pitch, float yaw, float base_x, float base_y, float base_z)
{
    Eigen::Matrix4f rotation_t;
    //handrotation( cartpos,roll, pitch, yaw,  base_x, base_y, base_z, rotation_t);
    Eigen::Affine3f test;
    test = getTransformation (0,0,0,roll, pitch, yaw );
    //return test.inverse();
    rotation_t = test.matrix();

    Eigen::Vector3f cartpos_t;
    cartpos_t(0) = cartpos(0,3) + base_x;
    cartpos_t(1) = cartpos(1,3) + base_y;
    cartpos_t(2) = cartpos(2,3) + base_z;

    Eigen::Vector4f t;
    t = cartpos_t.homogeneous();
    t = rotation_t.inverse() * t;
    rotation_t.col(3) = t ;


    return(rotation_t);

}
