#include <ros/ros.h>

#include <iostream>
#include <string>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "model");
    ros::NodeHandle n;
    ros::Rate r(1);

    //init publishers
    ros::Publisher model_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);


    //=================make an example sphereArray message for rviz==========================
    visualization_msgs::MarkerArray sphereArray;

    //first spehere
    visualization_msgs::Marker sphereMarker1;
    sphereMarker1.header.frame_id = "camera";
    sphereMarker1.type = visualization_msgs::Marker::SPHERE;
    sphereMarker1.header.stamp = ros::Time::now();
    float sphereRadius = 0.05f;
    sphereMarker1.scale.x = sphereRadius;
    sphereMarker1.scale.y = sphereRadius;
    sphereMarker1.scale.z = sphereRadius;

    sphereMarker1.pose.position.x = 0.1;
    sphereMarker1.pose.position.y = 0;
    sphereMarker1.pose.position.z = 0;

    sphereMarker1.color.r = 0.0f;
    sphereMarker1.color.g = 1.0f;
    sphereMarker1.color.b = 0.0f;
    sphereMarker1.color.a = 1.0;

    //second sphere
    visualization_msgs::Marker sphereMarker2;
    sphereMarker2.header.frame_id = "camera";
    sphereMarker2.type = visualization_msgs::Marker::SPHERE;
    sphereMarker2.header.stamp = ros::Time::now();
    sphereRadius = 0.1f;
    sphereMarker2.scale.x = sphereRadius;
    sphereMarker2.scale.y = sphereRadius;
    sphereMarker2.scale.z = sphereRadius;
    sphereMarker2.pose.position.x = 0.3;
    sphereMarker2.pose.position.y = 0.0;
    sphereMarker2.pose.position.z = 0.0;
    sphereMarker2.color.r = 1.0f;
    sphereMarker2.color.g = 0.0f;
    sphereMarker2.color.b = 0.0f;
    sphereMarker2.color.a = 1.0;



    //pushback the pehres to the sphereArray
    sphereArray.markers.push_back(sphereMarker1);
    sphereArray.markers.push_back(sphereMarker2);
    for (size_t i=0; i<sphereArray.markers.size(); i++){
        sphereArray.markers[i].id = i;
    }

    ros::Rate r(30);

    //publish the differetn ROS messages
    while (ros::ok()){
        
        model_pub.publish(sphereArray);
        r.sleep();
    }




    return 0;
