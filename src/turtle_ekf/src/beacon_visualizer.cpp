/**
 * \Description:
 * this node is used to publish the location for the beacon.
 * It publishes to:
 * - /visu_beacon: will be used in RVIZ for visualization.
 * Parameter:
 * - R, G, B: to set the color of the beacon inside the RVIZ.
 * - X, Y, Z: to set the location of the beacon inside the environment.
 *
 * SERRANO&ALI_ECN_M1_2017
 */


//Cpp
#include <math.h>

//ROS
#include "ros/ros.h"

//ROS msgs
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "ecn_turtle_path_publiser");
    ROS_INFO("ecn_turtle_path_publiser connected to roscore");
    ros::NodeHandle nh, nh_("~");  //ROS Handler - global, local namespace.

    double R, G, B;
    nh.param("R", R, 1.0);
    nh.param("G", G, 1.0);
    nh.param("B", B, 0.5);
    double beacon_x, beacon_y, beacon_z;
    nh.param("X", beacon_x, 0.0);
    nh.param("Y", beacon_y, 0.0);
    nh.param("Z", beacon_z, 0.0);

    //Subscriptions and publishings
    ros::Publisher  beaconPub = nh.advertise<visualization_msgs::Marker>("visu_beacon",1) ;

    // beacon as a sphere
    visualization_msgs::Marker beacon;
    beacon.header.frame_id = "/map";
    beacon.header.stamp = ros::Time();
    beacon.ns = "beacon";
    beacon.id = 1;
    beacon.type = beacon.SPHERE ;
    beacon.action = beacon.ADD;
    beacon.pose.position.x = beacon_x ;
    beacon.pose.position.y = beacon_y ;
    beacon.pose.position.z = beacon_z ;
    beacon.pose.orientation.x = 0.0 ;
    beacon.pose.orientation.y = 0.0 ;
    beacon.pose.orientation.z = 0.0 ;
    beacon.pose.orientation.w = 1.0 ;

    beacon.scale.x = 0.45;
    beacon.scale.y = 0.45;
    beacon.scale.z = 0.45;
    beacon.color.a = 1; // Don't forget to set the alpha!
    beacon.color.r = R ;
    beacon.color.g = G ;
    beacon.color.b = B ;

    ros::Rate rate(10);
    while (ros::ok()) {

        ros::spinOnce();

        beaconPub.publish(beacon) ;

        rate.sleep();
    }

    ROS_INFO("ROS-Node Terminated\n");
}
