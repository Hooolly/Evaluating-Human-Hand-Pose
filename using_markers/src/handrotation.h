/**
  * @file handrotation.h
  * transform handmodel to the postion of point cloud.
  * @author Hao Xu
*/



#ifndef HANDROTATION_H

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>


#include <Eigen/Core>
#include <Eigen/Dense>
//for the transformation
#include <Eigen/Geometry>

#include <math.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


// for depth image
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>


#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#define HANDROTATION_H

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

class handrotation
{

    KDL::Frame cartpos;
    float roll;
    float pitch;
    float yaw;
    float base_x;
    float base_y;
    float base_z;
    Eigen::Matrix4f rotation_t;

public:

    handrotation (KDL::Frame cartpos, float roll, float pitch, float yaw, float base_x, float base_y, float base_z, Eigen::Matrix4f rotation_t);

    Eigen::Matrix4f gethandrotation (Frame ,  float , float , float, float, float, float );


};

#endif // HANDROTATION_H


