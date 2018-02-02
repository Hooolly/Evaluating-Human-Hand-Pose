#include "lossfunc.h"


#include <iostream>

#include <pcl/pcl_macros.h>
#include <iostream>

#include <pcl/pcl_macros.h>
#include <stdio.h>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <pcl/registration/eigen.h>
//#include <pcl/registration/distances.h>
//#include <vector>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
// for handmodel
using namespace Eigen;
using namespace KDL;
using namespace std;
//for depth image

using namespace pcl;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
//using namespace distances;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Eigen::Map<Eigen::Vector3f> Vector3fMap;

LossFunc::LossFunc (Eigen::MatrixXf Hand_model, CloudPtr roiCloud)
{};

float LossFunc::getLossvalue(Eigen::MatrixXf Hand_model, CloudPtr roiCloud)
{
    int nr = roiCloud->points.size();
    Eigen::MatrixXf sphere(48,3);
    Eigen::VectorXf r(48,1);
    sphere.col(0) = Hand_model.col(0);
    sphere.col(1) = Hand_model.col(1);
    sphere.col(2) = Hand_model.col(2);
    r.col(0) = Hand_model.col(3);
    float loss_value = 0;
     //pcl::getMaxDistance (roiCloud, const Eigen::Vector4f &pivot_pt, Eigen::Vector4f &max_pt)



     for (unsigned int i=0;i<nr-1;i++){
        float min_dist = FLT_MAX;
        //pcl::Vector3fMapConst sample = roiCloud->points[i].getVector3fMap ();

        Eigen::Vector3f sample =roiCloud->points.at(i).getVector3fMap();

        for (unsigned int j=0;j<47;j++){


            int min_idx = -1;
            float dist;
            float dist_abs;


            Eigen::Vector3f temp = sphere.row(j);


            dist = (temp - sample).norm()-r(j);
            dist_abs = abs((temp - sample).norm()-r(j));

            if (dist < min_dist ){

              min_idx = int (j);
              min_dist = dist;
            }


        loss_value = loss_value + min_dist;
}

        return (loss_value);

}
}
