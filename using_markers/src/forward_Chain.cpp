// for handmodel
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

#include "handrotation.h"
#include "lossfunc.h"


//#include "MultMatrix.hpp"
// for handmodel
using namespace Eigen;
using namespace KDL;
using namespace std;
//for depth image

using namespace pcl;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
//using namespace handrotation;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;



// the ROS publishers

ros::Publisher pub_roiCloud;
ros::Publisher pub_rawDepthImage;

int main( int argc, char** argv )
{

    // Initialize ROS
    ros::init (argc, argv, "model");
    ros::NodeHandle n;




    //init publishers
    pub_roiCloud = n.advertise<sensor_msgs::PointCloud2>("/dhri/roiCloud", 100);
    pub_rawDepthImage = n.advertise<sensor_msgs::Image>("/dhri/depthImage", 100);


    //=====================read in depth image===============================================================
    string filepath;
    ros::param::get("/filepath", filepath);   //get "filepath"
    Mat depthImage = imread(filepath, -1);  //readin depth iamge from "filepath"



    //=====================find out region of interest on the depth image based on the 3D coordinate and a radius=====================
    float fx,fy,cx,cy,height,width;
    ros::param::get("/fx", fx); ros::param::get("/fy", fy); ros::param::get("/cx", cx); ros::param::get("/cy", cy);
    ros::param::get("/height", height); ros::param::get("/width", width);

    Eigen::Vector3f roiCenter3d(-0.12f, -0.0f, 0.78f);    //crop center position (3d position in [m])
    int uCord = int(floor(fx/roiCenter3d(2) * roiCenter3d(0) + cx));   //correspoinding u Pixel coordinate
    int vCord = int(floor(fy/roiCenter3d(2) * roiCenter3d(1) + cy));   //correspoinding v Pixel coordinate

    float cropRangeMeter = 0.15f;                    //crop radius in [m]
    int cropRangePixel = int(floor(fx/roiCenter3d(2)*cropRangeMeter)); //corresponding crop size in [pixels] (related to depth)
    int startU, startV, endU, endV;    //crop starting and ending coordinate [pixel]
    startU = min(max(uCord-cropRangePixel,0),639);
    endU = min(max(uCord+cropRangePixel,0),639);
    startV = min(max(vCord-cropRangePixel,0),479);
    endV = min(max(vCord+cropRangePixel,0),479);

    //cout<<startU<<" "<<startV<<" "<<endU<<" "<<endV<<endl;



    //=====================convert ROI pixels to point cloud===============================================================

    CloudPtr roiCloud(new Cloud());
    for (size_t i=startU; i<endU; i++){
        for(size_t j=startV; j<endV; j++){
            PointT pt;
            Vec3b intensity = depthImage.at<Vec3b>(j, i);
            pt.z = float(intensity.val[0] + intensity.val[1]*256)/1000.f ;  //[m]
            if (pt.z!=pt.z || pt.z<0.50 || fabs(pt.z-roiCenter3d(2)) > cropRangeMeter)
                continue;
            pt.x = (float(i) - cx) * pt.z *(1.0/ fx); //[m]
            pt.y = (float(j) - cy) * pt.z *(1.0/ fy);
            roiCloud->points.push_back(pt);
        }
    }

    cout<<"cloud size: "<<roiCloud->points.size()<<endl;


    //=====================make ros messages of the image and the cropped point cloud==============================================================
    sensor_msgs::PointCloud2 roiCloud_output;
    pcl::toROSMsg(*roiCloud, roiCloud_output);
   // cout<<"cloud : "<< roiCloud_output<<endl;
    roiCloud_output.header.frame_id="camera";
    roiCloud_output.header.stamp = ros::Time::now();

    sensor_msgs::Image msg = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", depthImage).toImageMsg();
    msg.header.stamp = ros::Time::now();

    //=====================find out region of interest on the depth image based on the 3D coordinate and a radius=====================
    float ori_x, ori_y, ori_z, roll, pitch, yaw;
    ros::param::get("/ori_x", ori_x); ros::param::get("/ori_y", ori_y); ros::param::get("/ori_z", ori_z);
    ros::param::get("/roll", roll); ros::param::get("/pitch", pitch); ros::param::get("/yaw", yaw);


    // thumb---------------------------------------------
    float thumb_1, thumb_2, thumb_3, thumb_4, thumb_5;
    ros::param::get("/thumb_1", thumb_1); ros::param::get("/thumb_2", thumb_2);
    ros::param::get("/thumb_3", thumb_3); ros::param::get("/thumb_4", thumb_4);
    ros::param::get("/thumb_5", thumb_5);

    // index---------------------------------------------
    float index_1, index_2, index_3, index_4, index_5;
    ros::param::get("/index_1", index_1); ros::param::get("/index_2", index_2);
    ros::param::get("/index_3", index_3); ros::param::get("/index_4", index_4);
    ros::param::get("/index_5", index_5);

    // middle---------------------------------------------
    float middle_1, middle_2, middle_3, middle_4, middle_5;
    ros::param::get("/middle_1", middle_1); ros::param::get("/middle_2", middle_2);
    ros::param::get("/middle_3", middle_3); ros::param::get("/middle_4", middle_4);
    ros::param::get("/middle_5", middle_5);

    // ring---------------------------------------------
    float ring_1, ring_2, ring_3, ring_4, ring_5;
    ros::param::get("/ring_1", ring_1); ros::param::get("/ring_2", ring_2);
    ros::param::get("/ring_3", ring_3); ros::param::get("/ring_4", ring_4);
    ros::param::get("/ring_5", ring_5);

    // little---------------------------------------------
    float little_1, little_2, little_3, little_4, little_5;
    ros::param::get("/little_1", little_1); ros::param::get("/little_2", little_2);
    ros::param::get("/little_3", little_3); ros::param::get("/little_4", little_4);
    ros::param::get("/little_5", little_5);

    //  adjust paramerer a==============================
    float adj;
    ros::param::get("/adj", adj);

    // define the base position of each finger============
    Eigen::MatrixXd base_position( 5, 3);
    base_position << -0.017 , -0.03052 , 0.1031  ,
                     0.0072 , -0.02199 , 0.11196 ,
                     0.03077 , -0.0185 , 0.1088 ,
                     0.0498 , -0.01457 , 0.09657 ,
                     0.0206 , -0.0284 , 0.0229 ;

    //init publishers=======================================
   ros::Publisher pub_sphereArray = n.advertise<visualization_msgs::MarkerArray>("/dhri/sphereArray", 100);


    //static Frame 	DH (double a, double alpha, double d, double theta)
    //thumb-------------------------------------------------------------
    KDL::Chain chain_thu1;
       chain_thu1.addSegment(Segment(Joint(Joint::RotX),Frame::DH(0.0, -M_PI_2 , 0.0, -(M_PI_2)/6)));
       chain_thu1.addSegment(Segment(Joint(Joint::RotX),Frame::DH(0.061, M_PI_2, 0.0, -(M_PI_2)/3)));
       chain_thu1.addSegment(Segment(Joint(Joint::RotY),Frame::DH(0.0, -M_PI_2, 0.0, 0.0)));
       chain_thu1.addSegment(Segment(Joint(Joint::RotY),Frame::DH(0.0461,0.0,0.0, -(M_PI_2)/3)));
       chain_thu1.addSegment(Segment(Joint(Joint::RotX),Frame::DH(0.0237, 0.0, 0.0, -(M_PI_2)/3)));

       // Create solver based on kinematic chain
       ChainFkSolverPos_recursive fksolver_thu1 = ChainFkSolverPos_recursive(chain_thu1);

    //----------------------------------------------------------------
    KDL::Chain chain_thu2;
        chain_thu2.addSegment(Segment(Joint(Joint::RotX),Frame::DH(0.0, -M_PI_2 , 0.0, -(M_PI_2)/6)));
        chain_thu2.addSegment(Segment(Joint(Joint::RotX),Frame::DH(0.061, M_PI_2, 0.0, -(M_PI_2)/3)));
        chain_thu2.addSegment(Segment(Joint(Joint::RotY),Frame::DH(0.0, -M_PI_2, 0.0, 0.0)));
        chain_thu2.addSegment(Segment(Joint(Joint::RotY),Frame::DH(0.0461,0.0,0.0, -(M_PI_2)/3)));


        // Create solver based on kinematic chain
        ChainFkSolverPos_recursive fksolver_thu2 = ChainFkSolverPos_recursive(chain_thu2);

    //----------------------------------------------------------------
    KDL::Chain chain_thu3;
        chain_thu3.addSegment(Segment(Joint(Joint::RotX),Frame::DH(0.0, -M_PI_2 , 0.0, -(M_PI_2)/6)));
        chain_thu3.addSegment(Segment(Joint(Joint::RotX),Frame::DH(0.061, M_PI_2, 0.0, -(M_PI_2)/3)));



        // Create solver based on kinematic chain
        ChainFkSolverPos_recursive fksolver_thu3 = ChainFkSolverPos_recursive(chain_thu3);



    //----------------------------------------------------------------
    KDL::Chain chain_thu4;
    chain_thu4.addSegment(Segment(Joint(Joint::RotX),Frame::DH(0.0, -M_PI_2 , 0.0, -(M_PI_2)/6)));

        // Create solver based on kinematic chain
        ChainFkSolverPos_recursive fksolver_thu4 = ChainFkSolverPos_recursive(chain_thu4);


    // Index-----------------------------------------------------------------------------------
     KDL::Chain chain_ind1;
        chain_ind1.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));
        chain_ind1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, M_PI_2, 0.0, -M_PI_2)));
        chain_ind1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0547, 0.0, 0.0, 0)));
        chain_ind1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0361,0.0,0.0,0.0)));
        chain_ind1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0144, 0.0, 0.0, 0.0)));

     // Create solver based on kinematic chain
     ChainFkSolverPos_recursive fksolver_ind1 = ChainFkSolverPos_recursive(chain_ind1);

     //----------------------------------------------
     KDL::Chain chain_ind2;
        chain_ind2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));
        chain_ind2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, M_PI_2, 0.0, -M_PI_2)));
        chain_ind2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0547, 0.0, 0.0, 0)));
        chain_ind2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0361,0.0,0.0,0.0)));
        //chain_ind2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0144, 0.0, 0.0, 0.0)));


     // Create solver based on kinematic chain
     ChainFkSolverPos_recursive fksolver_ind2 = ChainFkSolverPos_recursive(chain_ind2);

     //------------------------------
     KDL::Chain chain_ind3;
        chain_ind3.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));
        chain_ind3.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, M_PI_2, 0.0, -M_PI_2)));
        chain_ind3.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0547, 0.0, 0.0, 0)));

     // Create solver based on kinematic chain
     ChainFkSolverPos_recursive fksolver_ind3 = ChainFkSolverPos_recursive(chain_ind3);

     //------------------------------
     KDL::Chain chain_ind4;
        chain_ind4.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));
          //chain_ind4.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, M_PI_2, 0.0, -M_PI_2)));
          //chain_ind4.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0547, 0.0, 0.0, 0)));
          //chain_ind4.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0361,0.0,0.0,0.0)));
          //chain_ind4.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0144, 0.0, 0.0, 0.0)));




          // Create solver based on kinematic chain
          ChainFkSolverPos_recursive fksolver_ind4 = ChainFkSolverPos_recursive(chain_ind4);


    // Middle-----------------------------------------------------------------
    KDL::Chain chain_mid1;
        chain_mid1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));
        chain_mid1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, M_PI_2, 0.0, -M_PI_2)));
        chain_mid1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0512, 0.0, 0.0, 0)));
        chain_mid1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0382,0.0,0.0,0.0)));
        chain_mid1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0191, 0.0, 0.0, 0.0)));



          // Create solver based on kinematic chain
          ChainFkSolverPos_recursive fksolver_mid1 = ChainFkSolverPos_recursive(chain_mid1);

    //----------------------------------------------------------------------
    KDL::Chain chain_mid2;
        chain_mid2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));
        chain_mid2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, M_PI_2, 0.0, -M_PI_2)));
        chain_mid2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0512, 0.0, 0.0, 0)));
        chain_mid2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0382,0.0,0.0,0.0)));
        //chain_mid2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0191, 0.0, 0.0, 0.0)));

        // Create solver based on kinematic chain
        ChainFkSolverPos_recursive fksolver_mid2 = ChainFkSolverPos_recursive(chain_mid2);

    //----------------------------------------------------------------------
    KDL::Chain chain_mid3;
        chain_mid3.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));
        chain_mid3.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, M_PI_2, 0.0, -M_PI_2)));
        chain_mid3.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0512, 0.0, 0.0, 0)));


         // Create solver based on kinematic chain
         ChainFkSolverPos_recursive fksolver_mid3 = ChainFkSolverPos_recursive(chain_mid3);


    //----------------------------------------------------------------------
    KDL::Chain chain_mid4;
        chain_mid4.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));


        // Create solver based on kinematic chain
        ChainFkSolverPos_recursive fksolver_mid4 = ChainFkSolverPos_recursive(chain_mid4);

// Ring -------------------------------------------------------------------------
       KDL::Chain chain_rin1;
        chain_rin1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));
        chain_rin1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, M_PI_2, 0.0, -M_PI_2)));
        chain_rin1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0392, 0.0, 0.0, 0)));
        chain_rin1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0338,0.0,0.0,0.0)));
        chain_rin1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0170, 0.0, 0.0, 0.0)));

    // Create solver based on kinematic chain
       ChainFkSolverPos_recursive fksolver_rin1 = ChainFkSolverPos_recursive(chain_rin1);
 //--------------------------------------------------------

       KDL::Chain chain_rin2;
        chain_rin2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));
        chain_rin2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, M_PI_2, 0.0, -M_PI_2)));
        chain_rin2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0392, 0.0, 0.0, 0)));
        chain_rin2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0338,0.0,0.0,0.0)));
       //chain_rin2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0170, 0.0, 0.0, 0.0)));


       // Create solver based on kinematic chain
       ChainFkSolverPos_recursive fksolver_rin2 = ChainFkSolverPos_recursive(chain_rin2);

//--------------------------------------------------------

    KDL::Chain chain_rin3;
        chain_rin3.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));
        chain_rin3.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, M_PI_2, 0.0, -M_PI_2)));
        chain_rin3.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0392, 0.0, 0.0, 0)));

        // Create solver based on kinematic chain
        ChainFkSolverPos_recursive fksolver_rin3 = ChainFkSolverPos_recursive(chain_rin3);

//--------------------------------------------------------

    KDL::Chain chain_rin4;
        chain_rin4.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));


        // Create solver based on kinematic chain
        ChainFkSolverPos_recursive fksolver_rin4 = ChainFkSolverPos_recursive(chain_rin4);


// Little-----------------------------------------------------------
        KDL::Chain chain_lit1;
            chain_lit1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));
            chain_lit1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, M_PI_2, 0.0, -M_PI_2)));
            chain_lit1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0310, 0.0, 0.0, 0.0)));
            chain_lit1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0288, 0.0,0.0,0.0)));
            chain_lit1.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0129, 0.0, 0.0, 0.0)));

            // Create solver based on kinematic chain
            ChainFkSolverPos_recursive fksolver_lit1 = ChainFkSolverPos_recursive(chain_lit1);

//-----------------------------------------------------------------------------

        KDL::Chain chain_lit2;
            chain_lit2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));
            chain_lit2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, M_PI_2, 0.0, -M_PI_2)));
            chain_lit2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0310, 0.0, 0.0, 0.0)));
            chain_lit2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0288, 0.0,0.0,0.0)));
            //chain_lit2.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0129, 0.0, 0.0, 0.0)));


            // Create solver based on kinematic chain
            ChainFkSolverPos_recursive fksolver_lit2 = ChainFkSolverPos_recursive(chain_lit2);

//-----------------------------------------------------------------------------

        KDL::Chain chain_lit3;
            chain_lit3.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));
            chain_lit3.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, M_PI_2, 0.0, -M_PI_2)));
            chain_lit3.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0310, 0.0, 0.0, 0.0)));


            // Create solver based on kinematic chain
            ChainFkSolverPos_recursive fksolver_lit3 = ChainFkSolverPos_recursive(chain_lit3);

//--------------------------------------------------------------------------------
        KDL::Chain chain_lit4;
        chain_lit4.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(0.0, -M_PI_2, 0.0, M_PI)));

        // Create solver based on kinematic chain
        ChainFkSolverPos_recursive fksolver_lit4 = ChainFkSolverPos_recursive(chain_lit4);


// thumb------------------------------------------
  // Create the frame that will contain the results
  //static Rotation RotX (double angle);
  //declare member function

    Rotation R_thum = Rotation::EulerZYX(-0.0, 0.0, M_PI);
    //Rotation R_thum = Rotation::RotX(-M_PI);
    KDL::Vector V_thum(0.0206, -0.0284, 0.0229);
    //Frame (const Rotation &R, const Vector &V)
    Frame cartpos_thu1 ( R_thum, V_thum);

    // Calculate forward position kinematics for thumb
    unsigned int num1 = chain_thu1.getNrOfJoints();
    KDL::JntArray joint_thumb = JntArray(num1);

    joint_thumb(0) = double(thumb_1);
    joint_thumb(1) = double(thumb_2);
    joint_thumb(2) = double(thumb_3);
    joint_thumb(3) = double(thumb_4);
    joint_thumb(4) = double(thumb_5);

    KDL::JntArray jointpositions_thum = JntArray(num1);
    for(unsigned int i=0;i<num1;i++){
        switch (i) {
        case 0:
            jointpositions_thum(i) = joint_thumb(i);

        case 1:
            jointpositions_thum(i) = joint_thumb(i);

        case 2:
            jointpositions_thum(i) = joint_thumb(i);

        case 3:
            jointpositions_thum(i) = joint_thumb(i);

        case 4:
            jointpositions_thum(i) = joint_thumb(i);

        }
    }

    fksolver_thu1.JntToCart(jointpositions_thum,cartpos_thu1);




    //std::cout << rotation_thu1 << std::endl;
    // Create the frame that will contain the results
      Frame cartpos_thu2 ( R_thum, V_thum);
      // Calculate forward position kinematics for thumb
      unsigned int num2 = chain_thu2.getNrOfJoints();
      KDL::JntArray jointpositions_thum2 = JntArray(num2);
      for(unsigned int i=0;i<num2;i++){
          switch (i) {
          case 0:
              jointpositions_thum2(i) = joint_thumb(i);

          case 1:
              jointpositions_thum2(i) = joint_thumb(i);

          case 2:
              jointpositions_thum2(i) = joint_thumb(i);

          case 3:
              jointpositions_thum2(i) = joint_thumb(i);

          //case 4:
  //            jointpositions_thum(i) = joint_thumb(i);

          }
      }
      fksolver_thu2.JntToCart(jointpositions_thum2,cartpos_thu2);


      // Create the frame that will contain the results
        KDL::Frame cartpos_thu3(R_thum, V_thum);

        unsigned int num3 = chain_thu3.getNrOfJoints();
        KDL::JntArray jointpositions_thum3 = JntArray(num3);
        for(unsigned int i=0;i<num3;i++){
            switch (i) {
            case 0:
                jointpositions_thum3(i) = joint_thumb(i);

            case 1:
                jointpositions_thum3(i) = joint_thumb(i);



            }
        }
        fksolver_thu3.JntToCart(jointpositions_thum3, cartpos_thu3);




        // Create the frame that will contain the results
          KDL::Frame cartpos_thu4(R_thum, V_thum);

          unsigned int num4 = chain_thu3.getNrOfJoints();
          KDL::JntArray jointpositions_thum4 = JntArray(num4);
          for(unsigned int i=0;i<num4;i++){
              switch (i) {
              case 0:
                  jointpositions_thum4(i) = joint_thumb(i);

              }
          }




          fksolver_thu4.JntToCart(jointpositions_thum4,cartpos_thu4);




//index ------------------------------------------------------------

          // Create the frame that will contain the result
            //KDL::Frame cartpos_ind1;
            Rotation R_ind = Rotation::RotZ(double(0.0678));
            //Rotation R_thum = Rotation::RotX(-M_PI);
            KDL::Vector V_ind(0.0498, -0.01457, 0.09657);
            //Frame (const Rotation &R, const Vector &V)
            Frame cartpos_ind1 ( R_ind, V_ind);
//--------------------------------------------------------
            unsigned int ind_num1 = chain_ind1.getNrOfJoints();

             KDL::JntArray joint_ind1 = JntArray(ind_num1);

             joint_ind1(0) = double(index_1);
             joint_ind1(1) = double(index_2);
             joint_ind1(2) = double(index_3);
             joint_ind1(3) = double(index_4);
             joint_ind1(4) = double(index_5);

             KDL::JntArray jointpositions_ind1 = JntArray(ind_num1);
             for(unsigned int i=0;i<num1;i++){
                 switch (i) {
                 case 0:
                     jointpositions_ind1(i) = joint_ind1(i);

                 case 1:
                     jointpositions_ind1(i) = joint_ind1(i);

                 case 2:
                     jointpositions_ind1(i) = joint_ind1(i);

                 case 3:
                     jointpositions_ind1(i) = joint_ind1(i);

                 case 4:
                     jointpositions_ind1(i) = joint_ind1(i);

                 }
             }

            fksolver_ind1.JntToCart(jointpositions_ind1,cartpos_ind1);




            // Create the frame that will contain the results
              KDL::Frame cartpos_ind2 ( R_ind, V_ind);

              // Calculate forward position kinematics for index
              //bool kinematics_status;

              unsigned int ind_num2 = chain_ind2.getNrOfJoints();
              KDL::JntArray jointpositions_ind2 = JntArray(ind_num2);
              for(unsigned int i=0;i<num2;i++){
                  switch (i) {
                  case 0:
                      jointpositions_ind2(i) = joint_ind1(i);

                  case 1:
                      jointpositions_ind2(i) = joint_ind1(i);

                  case 2:
                      jointpositions_ind2(i) = joint_ind1(i);

                  case 3:
                      jointpositions_ind2(i) = joint_ind1(i);

                  //case 4:
                     // jointpositions_ind2(i) = joint_ind1(i);

                  }
              }
              fksolver_ind2.JntToCart(jointpositions_ind2,cartpos_ind2);




              // Create the frame that will contain the results
                KDL::Frame cartpos_ind3( R_ind, V_ind);

                unsigned int ind_num3 = chain_ind3.getNrOfJoints();
                KDL::JntArray jointpositions_ind3 = JntArray(ind_num3);
                for(unsigned int i=0;i<num3;i++){
                    switch (i) {
                    case 0:
                        jointpositions_ind3(i) = joint_ind1(i);

                    case 1:
                        jointpositions_ind3(i) = joint_ind1(i);

                    case 2:
                        jointpositions_ind3(i) = joint_ind1(i);

                    case 3:
                        jointpositions_ind3(i) = joint_ind1(i);

                    case 4:
                       jointpositions_ind3(i) = joint_ind1(i);

                    }
                }
                fksolver_ind3.JntToCart(jointpositions_ind3,cartpos_ind3);

                // Create the frame that will contain the results
                  KDL::Frame cartpos_ind4( R_ind, V_ind);


                  unsigned int ind_num4 = chain_ind4.getNrOfJoints();
                  KDL::JntArray jointpositions_ind4 = JntArray(ind_num4);
                  for(unsigned int i=0;i<num4;i++){
                      switch (i) {
                      case 0:
                          jointpositions_ind4(i) = joint_ind1(i);



                      }
                  }
                  fksolver_ind4.JntToCart(jointpositions_ind4,cartpos_ind4);
//midle-------------------------------------------------------------
                  // Create the frame that will contain the results
                    //KDL::Frame cartpos_mid1;
                  Rotation R_mid = Rotation::RotZ(double(0.2206));
                  KDL::Vector V_mid(0.03077, -0.0185, 0.1088);
                  //Frame (const Rotation &R, const Vector &V)
                  Frame cartpos_mid1 ( R_mid, V_mid);

                  KDL::JntArray joint_mid1 = JntArray(ind_num1);

                   joint_mid1(0) = double(middle_1);
                   joint_mid1(1) = double(middle_2);
                   joint_mid1(2) = double(middle_3);
                   joint_mid1(3) = double(middle_4);
                   joint_mid1(4) = double(middle_5);

                   KDL::JntArray jointpositions_mid1 = JntArray(ind_num1);
                   for(unsigned int i=0;i<num1;i++){
                       switch (i) {
                       case 0:
                           jointpositions_mid1(i) = joint_mid1(i);

                       case 1:
                           jointpositions_mid1(i) = joint_mid1(i);

                       case 2:
                           jointpositions_mid1(i) = joint_mid1(i);

                       case 3:
                           jointpositions_mid1(i) = joint_mid1(i);

                       case 4:
                           jointpositions_mid1(i) = joint_mid1(i);

                       }
                   }

                    fksolver_mid1.JntToCart(jointpositions_mid1,cartpos_mid1);


                    // Create the frame that will contain the results
                      KDL::Frame cartpos_mid2( R_mid, V_mid);

                      KDL::JntArray jointpositions_mid2 = JntArray(ind_num2);
                      for(unsigned int i=0;i<num2;i++){
                          switch (i) {
                          case 0:
                              jointpositions_mid2(i) = joint_mid1(i);

                          case 1:
                              jointpositions_mid2(i) = joint_mid1(i);

                          case 2:
                              jointpositions_mid2(i) = joint_mid1(i);

                          case 3:
                              jointpositions_mid2(i) = joint_mid1(i);

                         // case 4:
                              //jointpositions_mid2(i) = joint_mid1(i);

                          }
                      }
                      fksolver_mid2.JntToCart(jointpositions_mid2,cartpos_mid2);


                      // Create the frame that will contain the results
                        KDL::Frame cartpos_mid3( R_mid, V_mid);

                        KDL::JntArray jointpositions_mid3 = JntArray(ind_num3);
                        for(unsigned int i=0;i<num3;i++){
                            switch (i) {
                            case 0:
                                jointpositions_mid3(i) = joint_mid1(i);

                            case 1:
                                jointpositions_mid3(i) = joint_mid1(i);

                            case 2:
                                jointpositions_mid3(i) = joint_mid1(i);

                            //case 3:
                                //jointpositions_mid3(i) = joint_mid1(i);

                           // case 4:
                                //jointpositions_mid3(i) = joint_mid1(i);

                            }
                        }
                        fksolver_mid3.JntToCart(jointpositions_mid3,cartpos_mid3);

                        // Create the frame that will contain the results
                          KDL::Frame cartpos_mid4( R_mid, V_mid);

                          KDL::JntArray jointpositions_mid4 = JntArray(ind_num4);
                          for(unsigned int i=0;i<num4;i++){
                              switch (i) {
                              case 0:
                                  jointpositions_mid4(i) = joint_mid1(i);

                             // case 1:
                                 // jointpositions_mid4(i) = joint_mid1(i);

                              //case 2:
                                 // jointpositions_mid4(i) = joint_mid1(i);

                              //case 3:
                                  //jointpositions_mid4(i) = joint_mid1(i);

                             // case 4:
                                  //jointpositions_mid4(i) = joint_mid1(i);

                              }
                          }
                          fksolver_mid4.JntToCart(jointpositions_mid4,cartpos_mid4);

// ring------------------------------------------
                          // Create the frame that will contain the results
                            //KDL::Frame cartpos_rin1;
                          Rotation R_rin = Rotation::RotZ(double(0.2626));
                          KDL::Vector V_rin(0.0072, -0.02199, 0.11196);
                          //Frame (const Rotation &R, const Vector &V)
                          Frame cartpos_rin1 ( R_rin, V_rin);

                          KDL::JntArray joint_rin1 = JntArray(ind_num1);

                          joint_rin1(0) = double(ring_1);
                          joint_rin1(1) = double(ring_2);
                          joint_rin1(2) = double(ring_3);
                          joint_rin1(3) = double(ring_4);
                          joint_rin1(4) = double(ring_5);

                          KDL::JntArray jointpositions_rin1 = JntArray(ind_num1);
                          for(unsigned int i=0;i<num1;i++){
                              switch (i) {
                              case 0:
                                  jointpositions_rin1(i) = joint_rin1(i);

                              case 1:
                                  jointpositions_rin1(i) = joint_rin1(i);

                              case 2:
                                  jointpositions_rin1(i) = joint_rin1(i);

                              case 3:
                                  jointpositions_rin1(i) = joint_rin1(i);

                              case 4:
                                  jointpositions_rin1(i) = joint_rin1(i);

                              }
                          }

                            fksolver_rin1.JntToCart(jointpositions_rin1,cartpos_rin1);
                            // Create the frame that will contain the results
                              KDL::Frame cartpos_rin2( R_rin, V_rin);




                              KDL::JntArray jointpositions_rin2 = JntArray(ind_num2);
                              for(unsigned int i=0;i<num2;i++){
                                  switch (i) {
                                  case 0:
                                      jointpositions_rin2(i) = joint_rin1(i);

                                  case 1:
                                      jointpositions_rin2(i) = joint_rin1(i);

                                  case 2:
                                      jointpositions_rin2(i) = joint_rin1(i);

                                  case 3:
                                      jointpositions_rin2(i) = joint_rin1(i);

                                  case 4:
                                      jointpositions_rin2(i) = joint_rin1(i);

                                  }
                              }

                              fksolver_rin2.JntToCart(jointpositions_rin2,cartpos_rin2);
                              // Create the frame that will contain the results
                                KDL::Frame cartpos_rin3( R_rin, V_rin);

                                KDL::JntArray jointpositions_rin3 = JntArray(ind_num3);
                                for(unsigned int i=0;i<num3;i++){
                                    switch (i) {
                                    case 0:
                                        jointpositions_rin3(i) = joint_rin1(i);

                                    case 1:
                                        jointpositions_rin3(i) = joint_rin1(i);

                                    case 2:
                                        jointpositions_rin3(i) = joint_rin1(i);

                                    case 3:
                                        jointpositions_rin3(i) = joint_rin1(i);

                                    case 4:
                                        jointpositions_rin3(i) = joint_rin1(i);

                                    }
                                }
                                fksolver_rin3.JntToCart(jointpositions_rin3,cartpos_rin3);

                                // Create the frame that will contain the results
                                  KDL::Frame cartpos_rin4( R_rin, V_rin);

                                  KDL::JntArray jointpositions_rin4 = JntArray(ind_num4);
                                  for(unsigned int i=0;i<num4;i++){
                                      switch (i) {
                                      case 0:
                                          jointpositions_rin4(i) = joint_rin1(i);

                                      case 1:
                                          jointpositions_rin4(i) = joint_rin1(i);

                                      case 2:
                                          jointpositions_rin4(i) = joint_rin1(i);

                                      case 3:
                                          jointpositions_rin4(i) = joint_rin1(i);

                                      case 4:
                                          jointpositions_rin4(i) = joint_rin1(i);

                                      }
                                  }
                                  fksolver_rin4.JntToCart(jointpositions_rin4,cartpos_rin4);

// little------------------------------------------
  // Create the frame that will contain the results

    Rotation R_lit = Rotation::RotZ(double(0.2699));
    KDL::Vector V_lit(-0.017, -0.03052, 0.1031);
    //Frame (const Rotation &R, const Vector &V)
    Frame cartpos_lit1 ( R_lit, V_lit);
    KDL::JntArray joint_lit1 = JntArray(ind_num1);

         joint_lit1(0) = double(little_1);
         joint_lit1(1) = double(little_2);
         joint_lit1(2) = double(little_3);
         joint_lit1(3) = double(little_4);
         joint_lit1(4) = double(little_5);

         KDL::JntArray jointpositions_lit1 = JntArray(ind_num1);
             for(unsigned int i=0;i<num1;i++){
                switch (i) {
                   case 0:
                        jointpositions_lit1(i) = joint_lit1(i);

                   case 1:
                        jointpositions_lit1(i) = joint_lit1(i);

                   case 2:
                        jointpositions_lit1(i) = joint_lit1(i);

                   case 3:
                        jointpositions_lit1(i) = joint_lit1(i);

                   case 4:
                        jointpositions_lit1(i) = joint_lit1(i);

                }
             }
    fksolver_lit1.JntToCart(jointpositions_lit1,cartpos_lit1);
    // Create the frame that will contain the results
      KDL::Frame cartpos_lit2( R_lit, V_lit);


      KDL::JntArray jointpositions_lit2 = JntArray(ind_num2);
      for(unsigned int i=0;i<num2;i++){
          switch (i) {
          case 0:
              jointpositions_lit2(i) = joint_lit1(i);

          case 1:
              jointpositions_lit2(i) = joint_lit1(i);

          case 2:
              jointpositions_lit2(i) = joint_lit1(i);

          case 3:
              jointpositions_lit2(i) = joint_lit1(i);

          case 4:
              jointpositions_lit2(i) = joint_lit1(i);

          }
      }

      fksolver_lit2.JntToCart(jointpositions_lit2,cartpos_lit2);
      // Create the frame that will contain the results
        KDL::Frame cartpos_lit3( R_lit, V_lit);

        KDL::JntArray jointpositions_lit3 = JntArray(ind_num3);
        for(unsigned int i=0;i<num3;i++){
            switch (i) {
            case 0:
                jointpositions_lit3(i) = joint_lit1(i);

            case 1:
                jointpositions_lit3(i) = joint_lit1(i);

            case 2:
                jointpositions_lit3(i) = joint_lit1(i);

            case 3:
                jointpositions_lit3(i) = joint_lit1(i);

            case 4:
                jointpositions_lit3(i) = joint_lit1(i);

            }
        }
        fksolver_lit3.JntToCart(jointpositions_lit3,cartpos_lit3);

        // Create the frame that will contain the results
          KDL::Frame cartpos_lit4( R_lit, V_lit);

          KDL::JntArray jointpositions_lit4 = JntArray(ind_num4);
          for(unsigned int i=0;i<num4;i++){
              switch (i) {
              case 0:
                  jointpositions_lit4(i) = joint_lit1(i);

              case 1:
                  jointpositions_lit4(i) = joint_lit1(i);

              case 2:
                  jointpositions_lit4(i) = joint_lit1(i);

              case 3:
                  jointpositions_lit4(i) = joint_lit1(i);

              case 4:
                  jointpositions_lit4(i) = joint_lit1(i);

              }
          }
          fksolver_lit4.JntToCart(jointpositions_lit4,cartpos_lit4);



//=================make an example sphereArray message for rviz==========================
    visualization_msgs::MarkerArray sphereArray;
//spehere array

    Eigen::MatrixXf thum1_model(2,4);
    Eigen::MatrixXf thum2_model(2,4);
    Eigen::MatrixXf thum3_model(2,4);
    Eigen::MatrixXf thum4_model(2,4);


    //-------------
    Eigen::MatrixXf ind1_model(2,4);
    Eigen::MatrixXf ind2_model(2,4);
    Eigen::MatrixXf ind3_model(2,4);
    Eigen::MatrixXf ind4_model(4,4);
    //----------------
    Eigen::MatrixXf mid1_model(2,4);
    Eigen::MatrixXf mid2_model(2,4);
    Eigen::MatrixXf mid3_model(2,4);
    Eigen::MatrixXf mid4_model(4,4);
    //-----------
    Eigen::MatrixXf rin1_model(2,4);
    Eigen::MatrixXf rin2_model(2,4);
    Eigen::MatrixXf rin3_model(2,4);
    Eigen::MatrixXf rin4_model(4,4);
    //-------------------
    Eigen::MatrixXf lit1_model(2,4);
    Eigen::MatrixXf lit2_model(2,4);
    Eigen::MatrixXf lit3_model(2,4);
    Eigen::MatrixXf lit4_model(4,4);

for  (unsigned int i=1;i<20;i++){
    KDL::Frame cartpos;

    switch(i)
    {

   case 1:
       {
        //cartpos = cartpos_thu1;
        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        Eigen::Matrix4f rotation_thu1 = np.gethandrotation(cartpos_thu1, roll, pitch, yaw, float(base_position(4,0)), float(base_position(4,1)), float(base_position(4,2)));
        Eigen::Matrix4f rotation_thu2 = np.gethandrotation(cartpos_thu2, roll, pitch, yaw, float(base_position(4,0)), float(base_position(4,1)), float(base_position(4,2)));
        //std::cout <<rotation_thu2<< std::endl;

for (int j = 0; j<2; j++) {

            //std::cout << j << std::endl;
            visualization_msgs::Marker sphere_thumb_1;
            sphere_thumb_1.header.frame_id = "camera";
            sphere_thumb_1.type = visualization_msgs::Marker::SPHERE;
            sphere_thumb_1.header.stamp = ros::Time::now();
            float sphereRadius = 0.019f+0.001f*j;
            sphere_thumb_1.scale.x = sphereRadius;
            sphere_thumb_1.scale.y = sphereRadius;
            sphere_thumb_1.scale.z = sphereRadius;

            sphere_thumb_1.pose.position.x = (double)rotation_thu1(0,3) - ((double)rotation_thu1(0,3) - (double)rotation_thu2(0,3))/2*j + ori_x ;
            sphere_thumb_1.pose.position.y = (double)rotation_thu1(1,3) - ((double)rotation_thu1(1,3) - (double)rotation_thu2(1,3))/2*j + ori_y ;
            sphere_thumb_1.pose.position.z = (double)rotation_thu1(2,3) - ((double)rotation_thu1(2,3) - (double)rotation_thu2(2,3))/2*j + ori_z ;

            sphere_thumb_1.color.r = 1.0f;
            sphere_thumb_1.color.g = 0.0f;
            sphere_thumb_1.color.b = 0.0f;
            sphere_thumb_1.color.a = 1.0;

        //pushback the pehres to the sphereArray
            sphereArray.markers.push_back(sphere_thumb_1);
            thum1_model.row(j) << sphere_thumb_1.pose.position.x, sphere_thumb_1.pose.position.y, sphere_thumb_1.pose.position.z, sphereRadius;


        }
            //std::cout<<thum1_model<<std::endl;
    }
        case 2:
    {


        Eigen::Matrix4f test1;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test1;
        handrotation np(cartpos_test1, roll_ang, pitch_ang, yaw_ang, x, y, z, test1);

        Eigen::Matrix4f rotation_thu2 = np.gethandrotation(cartpos_thu2, roll, pitch, yaw, float(base_position(4,0)), float(base_position(4,1)), float(base_position(4,2)));
        Eigen::Matrix4f rotation_thu3 = np.gethandrotation(cartpos_thu3, roll, pitch, yaw, float(base_position(4,0)), float(base_position(4,1)), float(base_position(4,2)));
        //std::cout <<rotation_thu2<< std::endl;

        for (int j = 0; j<2; j++) {
                visualization_msgs::Marker sphere_thumb_2;
                sphere_thumb_2.header.frame_id = "camera";
                sphere_thumb_2.type = visualization_msgs::Marker::SPHERE;
                sphere_thumb_2.header.stamp = ros::Time::now();
                float sphereRadius = 0.028f+0.003f*j;
                sphere_thumb_2.scale.x = sphereRadius;
                sphere_thumb_2.scale.y = sphereRadius;
                sphere_thumb_2.scale.z = sphereRadius;


                sphere_thumb_2.pose.position.x = (double)rotation_thu2(0,3) - ((double)rotation_thu2(0,3) - (double)rotation_thu3(0,3))/2*j + ori_x;
                sphere_thumb_2.pose.position.y = (double)rotation_thu2(1,3) - ((double)rotation_thu2(1,3) - (double)rotation_thu3(1,3))/2*j + ori_y;
                sphere_thumb_2.pose.position.z = (double)rotation_thu2(2,3) - ((double)rotation_thu2(2,3) - (double)rotation_thu3(2,3))/2*j + ori_z;


                sphere_thumb_2.color.r = 0.0f;
                sphere_thumb_2.color.g = 1.0f;
                sphere_thumb_2.color.b = 0.0f;
                sphere_thumb_2.color.a = 1.0;

            //pushback the pehres to the sphereArray
                sphereArray.markers.push_back(sphere_thumb_2);
                thum2_model.row(j) << sphere_thumb_2.pose.position.x, sphere_thumb_2.pose.position.y, sphere_thumb_2.pose.position.z, sphereRadius;




        }
}



   case 3:
    {
       // cartpos = cartpos_thu3;
        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        Eigen::Matrix4f rotation_thu3 = np.gethandrotation(cartpos_thu3, roll, pitch, yaw, float(base_position(4,0)), float(base_position(4,1)), float(base_position(4,2)));
        Eigen::Matrix4f rotation_thu4 = np.gethandrotation(cartpos_thu4, roll, pitch, yaw, float(base_position(4,0)), float(base_position(4,1)), float(base_position(4,2)));

        for (int j = 0; j<2; j++) {
          visualization_msgs::Marker sphere_thumb_3;
          sphere_thumb_3.header.frame_id = "camera";
          sphere_thumb_3.type = visualization_msgs::Marker::SPHERE;
          sphere_thumb_3.header.stamp = ros::Time::now();
          float sphereRadius = 0.035f+0.002f*j;
          sphere_thumb_3.scale.x = sphereRadius;
          sphere_thumb_3.scale.y = sphereRadius;
          sphere_thumb_3.scale.z = sphereRadius;


          sphere_thumb_3.pose.position.x = (double)rotation_thu3(0,3) - ((double)rotation_thu3(0,3) - (double)rotation_thu4(0,3))/2*j + ori_x;
          sphere_thumb_3.pose.position.y = (double)rotation_thu3(1,3) - ((double)rotation_thu3(1,3) - (double)rotation_thu4(1,3))/2*j + ori_y;
          sphere_thumb_3.pose.position.z = (double)rotation_thu3(2,3) - ((double)rotation_thu3(2,3) - (double)rotation_thu4(2,3))/2*j + ori_z;


          sphere_thumb_3.color.r = 0.0f;
          sphere_thumb_3.color.g = 0.0f;
          sphere_thumb_3.color.b = 1.0f;
          sphere_thumb_3.color.a = 1.0;

      //pushback the pehres to the sphereArray
          sphereArray.markers.push_back(sphere_thumb_3);
          thum3_model.row(j) << sphere_thumb_3.pose.position.x, sphere_thumb_3.pose.position.y, sphere_thumb_3.pose.position.z, sphereRadius;

        }
      }
        //std::cout << cartpos_thu3 <<std::endl;
    case 4:
     {

        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

       // Eigen::Matrix4f rotation_thu3 = np.gethandrotation(cartpos_thu3, roll, pitch, yaw, float(base_position(4,0)), float(base_position(4,1)), float(base_position(4,2)));
        Eigen::Matrix4f rotation_thu4 = np.gethandrotation(cartpos_thu4, roll, pitch, yaw, float(base_position(4,0)), float(base_position(4,1)), float(base_position(4,2)));
        for (int n = 1; n<3; n++) {
          visualization_msgs::Marker sphere_thumb_4;
          sphere_thumb_4.header.frame_id = "camera";
          sphere_thumb_4.type = visualization_msgs::Marker::SPHERE;
          sphere_thumb_4.header.stamp = ros::Time::now();
          float sphereRadius = 0.026f;
          sphere_thumb_4.scale.x = sphereRadius;
          sphere_thumb_4.scale.y = sphereRadius;
          sphere_thumb_4.scale.z = sphereRadius;

          sphere_thumb_4.pose.position.x =  (double)rotation_thu4(0,3)*n/2 + ori_x;
          sphere_thumb_4.pose.position.y =  (double)rotation_thu4(1,3)*n/2 + ori_y;
          sphere_thumb_4.pose.position.z =  ((double)rotation_thu4(2,3)+ adj)*n/2 + ori_z - adj;

          sphere_thumb_4.color.r = 1.0f;
          sphere_thumb_4.color.g = 0.5f;
          sphere_thumb_4.color.b = 0.5f;
          sphere_thumb_4.color.a = 1.0;

      //pushback the pehres to the sphereArray
          sphereArray.markers.push_back(sphere_thumb_4);
          thum4_model.row(n-1) << sphere_thumb_4.pose.position.x, sphere_thumb_4.pose.position.y, sphere_thumb_4.pose.position.z, sphereRadius;
}
      }

//index --------------------------------------------------
    case 5:
     {

       // cartpos = cartpos_ind1;
        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        Eigen::Matrix4f rotation_ind1 = np.gethandrotation(cartpos_ind1, roll, pitch, yaw, float(base_position(3,0)), float(base_position(3,1)), float(base_position(3,2)));
        Eigen::Matrix4f rotation_ind2 = np.gethandrotation(cartpos_ind2, roll, pitch, yaw, float(base_position(3,0)), float(base_position(3,1)), float(base_position(3,2)));


        for (int j = 0; j<2; j++) {
          visualization_msgs::Marker sphere_ind_1;
          sphere_ind_1.header.frame_id = "camera";
          sphere_ind_1.type = visualization_msgs::Marker::SPHERE;
          sphere_ind_1.header.stamp = ros::Time::now();
          float sphereRadius = 0.018f+0.002*j;
          sphere_ind_1.scale.x = sphereRadius;
          sphere_ind_1.scale.y = sphereRadius;
          sphere_ind_1.scale.z = sphereRadius;

          sphere_ind_1.pose.position.x = (double)rotation_ind1(0,3) - ((double)rotation_ind1(0,3) - (double)rotation_ind2(0,3))/2*j + ori_x;
          sphere_ind_1.pose.position.y = (double)rotation_ind1(1,3) - ((double)rotation_ind1(1,3) - (double)rotation_ind2(1,3))/2*j + ori_y;
          sphere_ind_1.pose.position.z = (double)rotation_ind1(2,3) - ((double)rotation_ind1(2,3) - (double)rotation_ind2(2,3))/2*j + ori_z;

          sphere_ind_1.color.r = 1.0f;
          sphere_ind_1.color.g = 0.0f;
          sphere_ind_1.color.b = 0.0f;
          sphere_ind_1.color.a = 1.0;

      //pushback the pehres to the sphereArray
          sphereArray.markers.push_back(sphere_ind_1);
          ind1_model.row(j) << sphere_ind_1.pose.position.x, sphere_ind_1.pose.position.y, sphere_ind_1.pose.position.z, sphereRadius;

      }
}
    case 6:
     { //cartpos = cartpos_ind2;
        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        Eigen::Matrix4f rotation_ind2 = np.gethandrotation(cartpos_ind2, roll, pitch, yaw, float(base_position(3,0)), float(base_position(3,1)), float(base_position(3,2)));
        Eigen::Matrix4f rotation_ind3 = np.gethandrotation(cartpos_ind3, roll, pitch, yaw, float(base_position(3,0)), float(base_position(3,1)), float(base_position(3,2)));
        for (int j = 0; j<2; j++) {
          visualization_msgs::Marker sphere_ind_2;
          sphere_ind_2.header.frame_id = "camera";
          sphere_ind_2.type = visualization_msgs::Marker::SPHERE;
          sphere_ind_2.header.stamp = ros::Time::now();
          float sphereRadius = 0.024f+0.004*j;
          sphere_ind_2.scale.x = sphereRadius;
          sphere_ind_2.scale.y = sphereRadius;
          sphere_ind_2.scale.z = sphereRadius;

          sphere_ind_2.pose.position.x = (double)rotation_ind2(0,3) - ((double)rotation_ind2(0,3) - (double)rotation_ind3(0,3))/2*j + ori_x;
          sphere_ind_2.pose.position.y = (double)rotation_ind2(1,3) - ((double)rotation_ind2(1,3) - (double)rotation_ind3(1,3))/2*j + ori_y;
          sphere_ind_2.pose.position.z = (double)rotation_ind2(2,3) - ((double)rotation_ind2(2,3) - (double)rotation_ind3(2,3))/2*j + ori_z;

          sphere_ind_2.color.r = 0.0f;
          sphere_ind_2.color.g = 1.0f;
          sphere_ind_2.color.b = 0.0f;
          sphere_ind_2.color.a = 1.0;

      //pushback the pehres to the sphereArray
          sphereArray.markers.push_back(sphere_ind_2);
          ind2_model.row(j) << sphere_ind_2.pose.position.x, sphere_ind_2.pose.position.y, sphere_ind_2.pose.position.z, sphereRadius;
      }
}
    case 7:
     {// cartpos = cartpos_ind3;

        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        Eigen::Matrix4f rotation_ind3 = np.gethandrotation(cartpos_ind3, roll, pitch, yaw, float(base_position(3,0)), float(base_position(3,1)), float(base_position(3,2)));
        Eigen::Matrix4f rotation_ind4 = np.gethandrotation(cartpos_ind4, roll, pitch, yaw, float(base_position(3,0)), float(base_position(3,1)), float(base_position(3,2)));

        for (int j = 0; j<2; j++) {
          visualization_msgs::Marker sphere_ind_3;
          sphere_ind_3.header.frame_id = "camera";
          sphere_ind_3.type = visualization_msgs::Marker::SPHERE;
          sphere_ind_3.header.stamp = ros::Time::now();
          float sphereRadius = 0.029f+0.003*j;
          sphere_ind_3.scale.x = sphereRadius;
          sphere_ind_3.scale.y = sphereRadius;
          sphere_ind_3.scale.z = sphereRadius;

          sphere_ind_3.pose.position.x = (double)rotation_ind3(0,3) - ((double)rotation_ind3(0,3) - (double)rotation_ind4(0,3))/2*j + ori_x;
          sphere_ind_3.pose.position.y = (double)rotation_ind3(1,3) - ((double)rotation_ind3(1,3) - (double)rotation_ind4(1,3))/2*j + ori_y;
          sphere_ind_3.pose.position.z = (double)rotation_ind3(2,3) - ((double)rotation_ind3(2,3) - (double)rotation_ind4(2,3))/2*j + ori_z;

          sphere_ind_3.color.r = 0.0f;
          sphere_ind_3.color.g = 0.0f;
          sphere_ind_3.color.b = 1.0f;
          sphere_ind_3.color.a = 1.0;

      //pushback the pehres to the sphereArray
          sphereArray.markers.push_back(sphere_ind_3);
          ind3_model.row(j) << sphere_ind_3.pose.position.x, sphere_ind_3.pose.position.y, sphere_ind_3.pose.position.z, sphereRadius;
      }\

}
    case 8:
     { //cartpos = cartpos_ind4;

        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        //Eigen::Matrix4f rotation_ind3 = np.gethandrotation(cartpos_ind3, roll, pitch, yaw, float(base_position(3,0)), float(base_position(3,1)), float(base_position(3,2)));
        Eigen::Matrix4f rotation_ind4 = np.gethandrotation(cartpos_ind4, roll, pitch, yaw, float(base_position(3,0)), float(base_position(3,1)), float(base_position(3,2)));



        for  (unsigned int n=1; n<5; n++) {
          visualization_msgs::Marker sphere_ind_4;
          sphere_ind_4.header.frame_id = "camera";
          sphere_ind_4.type = visualization_msgs::Marker::SPHERE;
          sphere_ind_4.header.stamp = ros::Time::now();
          float sphereRadius = 0.024f+0.002*n;
          sphere_ind_4.scale.x = sphereRadius;
          sphere_ind_4.scale.y = sphereRadius;
          sphere_ind_4.scale.z = sphereRadius;

          sphere_ind_4.pose.position.x = (double)rotation_ind4(0,3) *n/4 + ori_x;
          sphere_ind_4.pose.position.y = ((double)rotation_ind4(1,3)  ) *n/4 + ori_y ;
          sphere_ind_4.pose.position.z = ((double)rotation_ind4(2,3) + adj) *n/4 + ori_z - adj ;

          sphere_ind_4.color.r = 1.0f;
          sphere_ind_4.color.g = 0.5f;
          sphere_ind_4.color.b = 0.5f;
          sphere_ind_4.color.a = 1.0;

      //pushback the pehres to the sphereArray
          sphereArray.markers.push_back(sphere_ind_4);
          ind4_model.row(n-1) << sphere_ind_4.pose.position.x, sphere_ind_4.pose.position.y, sphere_ind_4.pose.position.z, sphereRadius;
      }
}
  // middle--------------------------------------------------

   case 9:
     { //cartpos = cartpos_mid1;

        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        Eigen::Matrix4f rotation_mid1 = np.gethandrotation(cartpos_mid1, roll, pitch, yaw, float(base_position(2,0)), float(base_position(2,1)), float(base_position(2,2)));
        Eigen::Matrix4f rotation_mid2 = np.gethandrotation(cartpos_mid2, roll, pitch, yaw, float(base_position(2,0)), float(base_position(2,1)), float(base_position(2,2)));

        for (int j = 0; j<2; j++) {
          visualization_msgs::Marker sphere_mid_1;
          sphere_mid_1.header.frame_id = "camera";
          sphere_mid_1.type = visualization_msgs::Marker::SPHERE;
          sphere_mid_1.header.stamp = ros::Time::now();
          float sphereRadius = 0.019f;
          sphere_mid_1.scale.x = sphereRadius;
          sphere_mid_1.scale.y = sphereRadius;
          sphere_mid_1.scale.z = sphereRadius;

          sphere_mid_1.pose.position.x = (double)rotation_mid1(0,3) - ((double)rotation_mid1(0,3) - (double)rotation_mid2(0,3))/2*j + ori_x;
          sphere_mid_1.pose.position.y = (double)rotation_mid1(1,3) - ((double)rotation_mid1(1,3) - (double)rotation_mid2(1,3))/2*j + ori_y;
          sphere_mid_1.pose.position.z = (double)rotation_mid1(2,3) - ((double)rotation_mid1(2,3) - (double)rotation_mid2(2,3))/2*j + ori_z;


          sphere_mid_1.color.r = 1.0f;
          sphere_mid_1.color.g = 0.0f;
          sphere_mid_1.color.b = 0.0f;
          sphere_mid_1.color.a = 1.0;

      //pushback the pehres to the sphereArray
          sphereArray.markers.push_back(sphere_mid_1);
          mid1_model.row(j) << sphere_mid_1.pose.position.x, sphere_mid_1.pose.position.y, sphere_mid_1.pose.position.z, sphereRadius;
      }
}
    case 10:
     { //cartpos = cartpos_mid2;


        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        Eigen::Matrix4f rotation_mid2 = np.gethandrotation(cartpos_mid2, roll, pitch, yaw, float(base_position(2,0)), float(base_position(2,1)), float(base_position(2,2)));
        Eigen::Matrix4f rotation_mid3 = np.gethandrotation(cartpos_mid3, roll, pitch, yaw, float(base_position(2,0)), float(base_position(2,1)), float(base_position(2,2)));
        for (int j = 0; j<2; j++) {
          visualization_msgs::Marker sphere_mid_2;
          sphere_mid_2.header.frame_id = "camera";
          sphere_mid_2.type = visualization_msgs::Marker::SPHERE;
          sphere_mid_2.header.stamp = ros::Time::now();
          float sphereRadius = 0.024f+0.002*j;
          sphere_mid_2.scale.x = sphereRadius;
          sphere_mid_2.scale.y = sphereRadius;
          sphere_mid_2.scale.z = sphereRadius;


          sphere_mid_2.pose.position.x = (double)rotation_mid2(0,3) - ((double)rotation_mid2(0,3) - (double)rotation_mid3(0,3))/2*j + ori_x;
          sphere_mid_2.pose.position.y = (double)rotation_mid2(1,3) - ((double)rotation_mid2(1,3) - (double)rotation_mid3(1,3))/2*j + ori_y;
          sphere_mid_2.pose.position.z = (double)rotation_mid2(2,3) - ((double)rotation_mid2(2,3) - (double)rotation_mid3(2,3))/2*j + ori_z;

          sphere_mid_2.color.r = 0.0f;
          sphere_mid_2.color.g = 1.0f;
          sphere_mid_2.color.b = 0.0f;
          sphere_mid_2.color.a = 1.0;

      //pushback the pehres to the sphereArray
          sphereArray.markers.push_back(sphere_mid_2);
          mid2_model.row(j) << sphere_mid_2.pose.position.x, sphere_mid_2.pose.position.y, sphere_mid_2.pose.position.z, sphereRadius;
      }
}
    case 11:
     { //cartpos = cartpos_mid3;

        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        Eigen::Matrix4f rotation_mid3 = np.gethandrotation(cartpos_mid3, roll, pitch, yaw, float(base_position(2,0)), float(base_position(2,1)), float(base_position(2,2)));
        Eigen::Matrix4f rotation_mid4 = np.gethandrotation(cartpos_mid4, roll, pitch, yaw, float(base_position(2,0)), float(base_position(2,1)), float(base_position(2,2)));

        for (int j = 0; j<2; j++) {
          visualization_msgs::Marker sphere_mid_3;
          sphere_mid_3.header.frame_id = "camera";
          sphere_mid_3.type = visualization_msgs::Marker::SPHERE;
          sphere_mid_3.header.stamp = ros::Time::now();
          float sphereRadius = 0.028f+0.003*j;
          sphere_mid_3.scale.x = sphereRadius;
          sphere_mid_3.scale.y = sphereRadius;
          sphere_mid_3.scale.z = sphereRadius;

          sphere_mid_3.pose.position.x = (double)rotation_mid3(0,3) - ((double)rotation_mid3(0,3) - (double)rotation_mid4(0,3))/2*j + ori_x;
          sphere_mid_3.pose.position.y = (double)rotation_mid3(1,3) - ((double)rotation_mid3(1,3) - (double)rotation_mid4(1,3))/2*j + ori_y;
          sphere_mid_3.pose.position.z = (double)rotation_mid3(2,3) - ((double)rotation_mid3(2,3) - (double)rotation_mid4(2,3))/2*j + ori_z;

          sphere_mid_3.color.r = 0.0f;
          sphere_mid_3.color.g = 0.0f;
          sphere_mid_3.color.b = 1.0f;
          sphere_mid_3.color.a = 1.0;

      //pushback the pehres to the sphereArray
          sphereArray.markers.push_back(sphere_mid_3);
          mid3_model.row(j) << sphere_mid_3.pose.position.x, sphere_mid_3.pose.position.y, sphere_mid_3.pose.position.z, sphereRadius;
      }
}

  case 12:
     { //cartpos = cartpos_mid4;

        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        //Eigen::Matrix4f rotation_mid2 = np.gethandrotation(cartpos_mid2, roll, pitch, yaw, float(base_position(2,0)), float(base_position(2,1)), float(base_position(2,2)));
        Eigen::Matrix4f rotation_mid4 = np.gethandrotation(cartpos_mid4, roll, pitch, yaw, float(base_position(2,0)), float(base_position(2,1)), float(base_position(2,2)));

        for  (unsigned int n=1; n<5; n++) {
          visualization_msgs::Marker sphere_mid_4;
          sphere_mid_4.header.frame_id = "camera";
          sphere_mid_4.type = visualization_msgs::Marker::SPHERE;
          sphere_mid_4.header.stamp = ros::Time::now();
          float sphereRadius = 0.027f+0.002*n;
          sphere_mid_4.scale.x = sphereRadius;
          sphere_mid_4.scale.y = sphereRadius;
          sphere_mid_4.scale.z = sphereRadius;

          sphere_mid_4.pose.position.x = (double)rotation_mid4(0,3)*n/4 + ori_x;
          sphere_mid_4.pose.position.y = (double)rotation_mid4(1,3)*n/4 + ori_y;
          sphere_mid_4.pose.position.z = ((double)rotation_mid4(2,3)+ adj) *n/4 + ori_z - adj;
          sphere_mid_4.color.r = 1.0f;
          sphere_mid_4.color.g = 0.5f;
          sphere_mid_4.color.b = 0.5f;
          sphere_mid_4.color.a = 1.0;

      //pushback the pehres to the sphereArray
          sphereArray.markers.push_back(sphere_mid_4);
          mid4_model.row(n-1) << sphere_mid_4.pose.position.x, sphere_mid_4.pose.position.y, sphere_mid_4.pose.position.z, sphereRadius;
      }
}
// ring--------------------------------------------------

          case 13:
           { //cartpos = cartpos_rin1;

        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        Eigen::Matrix4f rotation_rin1 = np.gethandrotation(cartpos_rin1, roll, pitch, yaw, float(base_position(1,0)), float(base_position(1,1)), float(base_position(1,2)));
        Eigen::Matrix4f rotation_rin2 = np.gethandrotation(cartpos_rin2, roll, pitch, yaw, float(base_position(1,0)), float(base_position(1,1)), float(base_position(1,2)));

        for (int j = 0; j<2; j++) {
                visualization_msgs::Marker sphere_rin_1;
                sphere_rin_1.header.frame_id = "camera";
                sphere_rin_1.type = visualization_msgs::Marker::SPHERE;
                sphere_rin_1.header.stamp = ros::Time::now();
                float sphereRadius = 0.018f+0.002*j;
                sphere_rin_1.scale.x = sphereRadius;
                sphere_rin_1.scale.y = sphereRadius;
                sphere_rin_1.scale.z = sphereRadius;

                sphere_rin_1.pose.position.x = (double)rotation_rin1(0,3) - ((double)rotation_rin1(0,3) - (double)rotation_rin2(0,3))/2*j + ori_x;
                sphere_rin_1.pose.position.y = (double)rotation_rin1(1,3) - ((double)rotation_rin1(1,3) - (double)rotation_rin2(1,3))/2*j + ori_y;
                sphere_rin_1.pose.position.z = (double)rotation_rin1(2,3) - ((double)rotation_rin1(2,3) - (double)rotation_rin2(2,3))/2*j + ori_z;

                sphere_rin_1.color.r = 1.0f;
                sphere_rin_1.color.g = 0.0f;
                sphere_rin_1.color.b = 0.0f;
                sphere_rin_1.color.a = 1.0;

            //pushback the pehres to the sphereArray
                sphereArray.markers.push_back(sphere_rin_1);
                 rin1_model.row(j) << sphere_rin_1.pose.position.x, sphere_rin_1.pose.position.y, sphere_rin_1.pose.position.z, sphereRadius;
            }
    }
          case 14:
           {

        //cartpos = cartpos_rin2;

        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        Eigen::Matrix4f rotation_rin2 = np.gethandrotation(cartpos_rin2, roll, pitch, yaw, float(base_position(1,0)), float(base_position(1,1)), float(base_position(1,2)));
        Eigen::Matrix4f rotation_rin3 = np.gethandrotation(cartpos_rin3 , roll, pitch, yaw, float(base_position(1,0)), float(base_position(1,1)), float(base_position(1,2)));
        for (int j = 0; j<2; j++) {
                visualization_msgs::Marker sphere_rin_2;
                sphere_rin_2.header.frame_id = "camera";
                sphere_rin_2.type = visualization_msgs::Marker::SPHERE;
                sphere_rin_2.header.stamp = ros::Time::now();
                float sphereRadius = 0.024f+0.002*j;
                sphere_rin_2.scale.x = sphereRadius;
                sphere_rin_2.scale.y = sphereRadius;
                sphere_rin_2.scale.z = sphereRadius;

                sphere_rin_2.pose.position.x = (double)rotation_rin2(0,3) - ((double)rotation_rin2(0,3) - (double)rotation_rin3(0,3))/2*j + ori_x;
                sphere_rin_2.pose.position.y = (double)rotation_rin2(1,3) - ((double)rotation_rin2(1,3) - (double)rotation_rin3(1,3))/2*j + ori_y;
                sphere_rin_2.pose.position.z = (double)rotation_rin2(2,3) - ((double)rotation_rin2(2,3) - (double)rotation_rin3(2,3))/2*j + ori_z;

                sphere_rin_2.color.r = 0.0f;
                sphere_rin_2.color.g = 1.0f;
                sphere_rin_2.color.b = 0.0f;
                sphere_rin_2.color.a = 1.0;

            //pushback the pehres to the sphereArray
                sphereArray.markers.push_back(sphere_rin_2);
                rin2_model.row(j) << sphere_rin_2.pose.position.x, sphere_rin_2.pose.position.y, sphere_rin_2.pose.position.z, sphereRadius;
            }
}
          case 15:
           { //cartpos = cartpos_rin3;

        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        Eigen::Matrix4f rotation_rin3 = np.gethandrotation(cartpos_rin3, roll, pitch, yaw, float(base_position(1,0)), float(base_position(1,1)), float(base_position(1,2)));
        Eigen::Matrix4f rotation_rin4 = np.gethandrotation(cartpos_rin4 , roll, pitch, yaw, float(base_position(1,0)), float(base_position(1,1)), float(base_position(1,2)));
        for (int j = 0; j<2; j++) {
                visualization_msgs::Marker sphere_rin_3;
                sphere_rin_3.header.frame_id = "camera";
                sphere_rin_3.type = visualization_msgs::Marker::SPHERE;
                sphere_rin_3.header.stamp = ros::Time::now();
                float sphereRadius = 0.027f+0.002*j;
                sphere_rin_3.scale.x = sphereRadius;
                sphere_rin_3.scale.y = sphereRadius;
                sphere_rin_3.scale.z = sphereRadius;

                sphere_rin_3.pose.position.x = (double)rotation_rin3(0,3) - ((double)rotation_rin3(0,3) - (double)rotation_rin4(0,3))/2*j + ori_x;
                sphere_rin_3.pose.position.y = (double)rotation_rin3(1,3) - ((double)rotation_rin3(1,3) - (double)rotation_rin4(1,3))/2*j + ori_y;
                sphere_rin_3.pose.position.z = (double)rotation_rin3(2,3) - ((double)rotation_rin3(2,3) - (double)rotation_rin4(2,3))/2*j + ori_z;

                sphere_rin_3.color.r = 0.0f;
                sphere_rin_3.color.g = 0.0f;
                sphere_rin_3.color.b = 1.0f;
                sphere_rin_3.color.a = 1.0;

            //pushback the pehres to the sphereArray
                sphereArray.markers.push_back(sphere_rin_3);
                rin3_model.row(j) << sphere_rin_3.pose.position.x, sphere_rin_3.pose.position.y, sphere_rin_3.pose.position.z, sphereRadius;
            }
}

          case 16:
           {

        //cartpos = cartpos_rin4;
        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

       // Eigen::Matrix4f rotation_rin3 = np.gethandrotation(cartpos_rin3, roll, pitch, yaw, float(base_position(1,0)), float(base_position(1,1)), float(base_position(1,2)));
        Eigen::Matrix4f rotation_rin4 = np.gethandrotation(cartpos_rin4 , roll, pitch, yaw, float(base_position(1,0)), float(base_position(1,1)), float(base_position(1,2)));


        for  (unsigned int n=1; n<5; n++) {
                visualization_msgs::Marker sphere_rin_4;
                sphere_rin_4.header.frame_id = "camera";
                sphere_rin_4.type = visualization_msgs::Marker::SPHERE;
                sphere_rin_4.header.stamp = ros::Time::now();
                float sphereRadius = 0.027f+0.001*n;
                sphere_rin_4.scale.x = sphereRadius;
                sphere_rin_4.scale.y = sphereRadius;
                sphere_rin_4.scale.z = sphereRadius;

                sphere_rin_4.pose.position.x = (double)rotation_rin4(0,3)*n/4 + ori_x;
                sphere_rin_4.pose.position.y = (double)rotation_rin4(1,3)*n/4 + ori_y;
                sphere_rin_4.pose.position.z = ((double)rotation_rin4(2,3)+ adj) *n/4 + ori_z - adj;

                sphere_rin_4.color.r = 1.0f;
                sphere_rin_4.color.g = 0.5f;
                sphere_rin_4.color.b = 0.5f;
                sphere_rin_4.color.a = 1.0;

            //pushback the pehres to the sphereArray
                sphereArray.markers.push_back(sphere_rin_4);
                rin4_model.row(n-1) << sphere_rin_4.pose.position.x, sphere_rin_4.pose.position.y, sphere_rin_4.pose.position.z, sphereRadius;
            }
}
        // little--------------------------------------------------

                  case 17:
                   { //cartpos = cartpos_lit1;

        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        Eigen::Matrix4f rotation_lit1 = np.gethandrotation(cartpos_lit1, roll, pitch, yaw, float(base_position(0,0)), float(base_position(0,1)), float(base_position(0,2)));
        Eigen::Matrix4f rotation_lit2 = np.gethandrotation(cartpos_lit2, roll, pitch, yaw, float(base_position(0,0)), float(base_position(0,1)), float(base_position(0,2)));

        for (int j = 0; j<2; j++) {
                        visualization_msgs::Marker sphere_lit_1;
                        sphere_lit_1.header.frame_id = "camera";
                        sphere_lit_1.type = visualization_msgs::Marker::SPHERE;
                        sphere_lit_1.header.stamp = ros::Time::now();
                        float sphereRadius = 0.018f+0.001*j;
                        sphere_lit_1.scale.x = sphereRadius;
                        sphere_lit_1.scale.y = sphereRadius;
                        sphere_lit_1.scale.z = sphereRadius;

                        sphere_lit_1.pose.position.x = (double)rotation_lit1(0,3) - ((double)rotation_lit1(0,3) - (double)rotation_lit2(0,3))/2*j + ori_x;
                        sphere_lit_1.pose.position.y = (double)rotation_lit1(1,3) - ((double)rotation_lit1(1,3) - (double)rotation_lit2(1,3))/2*j + ori_y;
                        sphere_lit_1.pose.position.z = (double)rotation_lit1(2,3) - ((double)rotation_lit1(2,3) - (double)rotation_lit2(2,3))/2*j + ori_z;

                        sphere_lit_1.color.r = 1.0f;
                        sphere_lit_1.color.g = 0.0f;
                        sphere_lit_1.color.b = 0.0f;
                        sphere_lit_1.color.a = 1.0;

                    //pushback the pehres to the sphereArray
                        sphereArray.markers.push_back(sphere_lit_1);
                        lit1_model.row(j) << sphere_lit_1.pose.position.x, sphere_lit_1.pose.position.y, sphere_lit_1.pose.position.z, sphereRadius;
                    }
}
                  case 18:
                   {

        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        Eigen::Matrix4f rotation_lit3 = np.gethandrotation(cartpos_lit3, roll, pitch, yaw, float(base_position(0,0)), float(base_position(0,1)), float(base_position(0,2)));
        Eigen::Matrix4f rotation_lit2 = np.gethandrotation(cartpos_lit2, roll, pitch, yaw, float(base_position(0,0)), float(base_position(0,1)), float(base_position(0,2)));
for (int j = 0; j<2; j++) {
                        visualization_msgs::Marker sphere_lit_2;
                        sphere_lit_2.header.frame_id = "camera";
                        sphere_lit_2.type = visualization_msgs::Marker::SPHERE;
                        sphere_lit_2.header.stamp = ros::Time::now();
                        float sphereRadius = 0.024f+0.002*j;
                        sphere_lit_2.scale.x = sphereRadius;
                        sphere_lit_2.scale.y = sphereRadius;
                        sphere_lit_2.scale.z = sphereRadius;

                        sphere_lit_2.pose.position.x = (double)rotation_lit2(0,3) - ((double)rotation_lit2(0,3) - (double)rotation_lit3(0,3))/2*j + ori_x;
                        sphere_lit_2.pose.position.y = (double)rotation_lit2(1,3) - ((double)rotation_lit2(1,3) - (double)rotation_lit3(1,3))/2*j + ori_y;
                        sphere_lit_2.pose.position.z = (double)rotation_lit2(2,3) - ((double)rotation_lit2(2,3) - (double)rotation_lit3(2,3))/2*j + ori_z;

                        sphere_lit_2.color.r = 0.0f;
                        sphere_lit_2.color.g = 1.0f;
                        sphere_lit_2.color.b = 0.0f;
                        sphere_lit_2.color.a = 1.0;

                    //pushback the pehres to the sphereArray
                        sphereArray.markers.push_back(sphere_lit_2);
                        lit2_model.row(j) << sphere_lit_2.pose.position.x, sphere_lit_2.pose.position.y, sphere_lit_2.pose.position.z, sphereRadius;
                    }
    }
                  case 19:
                   {

        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

        Eigen::Matrix4f rotation_lit3 = np.gethandrotation(cartpos_lit3, roll, pitch, yaw, float(base_position(0,0)), float(base_position(0,1)), float(base_position(0,2)));
        Eigen::Matrix4f rotation_lit4 = np.gethandrotation(cartpos_lit4, roll, pitch, yaw, float(base_position(0,0)), float(base_position(0,1)), float(base_position(0,2)));
for (int j = 0; j<2; j++) {
                        visualization_msgs::Marker sphere_lit_3;
                        sphere_lit_3.header.frame_id = "camera";
                        sphere_lit_3.type = visualization_msgs::Marker::SPHERE;
                        sphere_lit_3.header.stamp = ros::Time::now();
                        float sphereRadius = 0.026f+0.002*j;
                        sphere_lit_3.scale.x = sphereRadius;
                        sphere_lit_3.scale.y = sphereRadius;
                        sphere_lit_3.scale.z = sphereRadius;


                        sphere_lit_3.pose.position.x = (double)rotation_lit3(0,3) - ((double)rotation_lit3(0,3) - (double)rotation_lit4(0,3))/2*j + ori_x;
                        sphere_lit_3.pose.position.y = (double)rotation_lit3(1,3) - ((double)rotation_lit3(1,3) - (double)rotation_lit4(1,3))/2*j + ori_y;
                        sphere_lit_3.pose.position.z = (double)rotation_lit3(2,3) - ((double)rotation_lit3(2,3) - (double)rotation_lit4(2,3))/2*j + ori_z;

                        sphere_lit_3.color.r = 0.0f;
                        sphere_lit_3.color.g = 0.0f;
                        sphere_lit_3.color.b = 1.0f;
                        sphere_lit_3.color.a = 1.0;

                    //pushback the pehres to the sphereArray
                        sphereArray.markers.push_back(sphere_lit_3);
                        lit3_model.row(j) << sphere_lit_3.pose.position.x, sphere_lit_3.pose.position.y, sphere_lit_3.pose.position.z, sphereRadius;
                    }

    }
                  case 20:
                   {
        Eigen::Matrix4f test;
        float x,y,z;
        float roll_ang, pitch_ang, yaw_ang;
        KDL::Frame cartpos_test;
        handrotation np(cartpos_test, roll_ang, pitch_ang, yaw_ang, x, y, z, test);

       // Eigen::Matrix4f rotation_lit3 = np.gethandrotation(cartpos_lit3, roll, pitch, yaw, float(base_position(0,0)), float(base_position(0,1)), float(base_position(0,2)));
        Eigen::Matrix4f rotation_lit4 = np.gethandrotation(cartpos_lit4, roll, pitch, yaw, float(base_position(0,0)), float(base_position(0,1)), float(base_position(0,2)));


        for  (unsigned int n=1; n<5; n++) {
                        visualization_msgs::Marker sphere_lit_4;
                        sphere_lit_4.header.frame_id = "camera";
                        sphere_lit_4.type = visualization_msgs::Marker::SPHERE;
                        sphere_lit_4.header.stamp = ros::Time::now();
                        float sphereRadius = 0.028f+0.001*n;
                        sphere_lit_4.scale.x = sphereRadius;
                        sphere_lit_4.scale.y = sphereRadius;
                        sphere_lit_4.scale.z = sphereRadius;

                        sphere_lit_4.pose.position.x = (double)rotation_lit4(0,3)*n/4 + ori_x;
                        sphere_lit_4.pose.position.y = (double)rotation_lit4(1,3)*n/4 + ori_y;
                        sphere_lit_4.pose.position.z = ((double)rotation_lit4(2,3)+ adj) *n/4 + ori_z - adj;

                        sphere_lit_4.color.r = 1.0f;
                        sphere_lit_4.color.g = 0.5f;
                        sphere_lit_4.color.b = 0.5f;
                        sphere_lit_4.color.a = 1.0;

                    //pushback the pehres to the sphereArray
                        sphereArray.markers.push_back(sphere_lit_4);
                        lit4_model.row(n-1) << sphere_lit_4.pose.position.x, sphere_lit_4.pose.position.y, sphere_lit_4.pose.position.z, sphereRadius;

                    }
}
    }


}
Eigen::MatrixXf Hand_model(48,4);
                Hand_model.row(0) = thum1_model.row(0);
                Hand_model.row(1) = thum1_model.row(1);
                Hand_model.row(2) = thum2_model.row(0);
                Hand_model.row(3) = thum2_model.row(1);
                Hand_model.row(4) = thum3_model.row(0);
                Hand_model.row(5) = thum3_model.row(1);
                Hand_model.row(6) = thum4_model.row(0);
                Hand_model.row(7) = thum4_model.row(1);
                //ind-----------
                Hand_model.row(8) = ind1_model.row(0);
                Hand_model.row(9) = ind1_model.row(1);
                Hand_model.row(10) = ind2_model.row(0);
                Hand_model.row(11) = ind2_model.row(1);
                Hand_model.row(12) = ind3_model.row(0);
                Hand_model.row(13) = ind3_model.row(1);
                Hand_model.row(14) = ind4_model.row(0);
                Hand_model.row(15) = ind4_model.row(1);
                Hand_model.row(16) = ind4_model.row(2);
                Hand_model.row(17) = ind4_model.row(3);
                //mid----------
                Hand_model.row(18) = mid1_model.row(0);
                Hand_model.row(19) = mid1_model.row(1);
                Hand_model.row(20) = mid2_model.row(0);
                Hand_model.row(21) = mid2_model.row(1);
                Hand_model.row(22) = mid3_model.row(0);
                Hand_model.row(23) = mid3_model.row(1);
                Hand_model.row(24) = mid4_model.row(0);
                Hand_model.row(25) = mid4_model.row(1);
                Hand_model.row(26) = mid4_model.row(2);
                Hand_model.row(27) = mid4_model.row(3);
                //ring---------
                Hand_model.row(28) = rin1_model.row(0);
                Hand_model.row(29) = rin1_model.row(1);
                Hand_model.row(30) = rin2_model.row(0);
                Hand_model.row(31) = rin2_model.row(1);
                Hand_model.row(32) = rin3_model.row(0);
                Hand_model.row(33) = rin3_model.row(1);
                Hand_model.row(34) = rin4_model.row(0);
                Hand_model.row(35) = rin4_model.row(1);
                Hand_model.row(36) = rin4_model.row(2);
                Hand_model.row(37) = rin4_model.row(3);
                //lit--------------
                Hand_model.row(38) = lit1_model.row(0);
                Hand_model.row(39) = lit1_model.row(1);
                Hand_model.row(40) = lit2_model.row(0);
                Hand_model.row(41) = lit2_model.row(1);
                Hand_model.row(42) = lit3_model.row(0);
                Hand_model.row(43) = lit3_model.row(1);
                Hand_model.row(44) = lit4_model.row(0);
                Hand_model.row(45) = lit4_model.row(1);
                Hand_model.row(46) = lit4_model.row(2);
                Hand_model.row(47) = lit4_model.row(3);
                //std::cout<<Hand_model<<std::endl;
                //std::cout<<thum1_model<<std::endl;

                LossFunc loss(Hand_model, roiCloud);
                  float loss_total;
                  loss_total = loss.getLossvalue(Hand_model, roiCloud);
                  std::cout<<loss_total<<std::endl;


    for (size_t i=0; i<sphereArray.markers.size(); i++){
        sphereArray.markers[i].id = i;
    }
    int a = sphereArray.markers.size();
    ros::Rate r(10);
    //std::cout << a <<std::endl;
    //publish the differetn ROS messages
    while (ros::ok()){
        pub_sphereArray.publish(sphereArray);
        pub_roiCloud.publish (roiCloud_output);
        pub_rawDepthImage.publish(msg);



    }


}




