/**
 * \Description:
 * this node is used to publish three markers of type circle, radius of each one is corresponding to one distance between the robot and the beacon.
 * It subscribes to:
 * - /beacon_distance
 * It publishes to:
 * - /visu_circle
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

double circleRadius ;
double angleRes = 5*M_PI/180.0 ;  // 5°
bool radiusAvailable ;

// Callback functions..
void getRadiusCallback( std_msgs::Float64 radius_msg ) {
    radiusAvailable = true;
    circleRadius = radius_msg.data / 100;
}

int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "ecn_turtle_path_publiser");
    ROS_INFO("ecn_turtle_path_publiser connected to roscore");
    ros::NodeHandle nh, nh_("~");  //ROS Handler - global, local namespace.

    double scaling;
    nh_.param("scaling", scaling, 1.0);
    double R, G, B;
    nh.param("R", R, 1.0);
    nh.param("G", G, 1.0);
    nh.param("B", B, 0.5);
    double beacon_x, beacon_y, beacon_z;
    nh.param("X", beacon_x, 0.0);
    nh.param("Y", beacon_y, 0.0);
    nh.param("Z", beacon_z, 0.0);

    //Subscriptions and publishings
    ros::Subscriber radiusSub = nh.subscribe<std_msgs::Float64>("beacon_distance", 1, getRadiusCallback);
    ros::Publisher  circlePub = nh.advertise<visualization_msgs::Marker>("visu_circle",1);

    // beacon diameter depending on measure of camera
    visualization_msgs::Marker circle;
    circle.header.frame_id = "/map";
    circle.header.stamp = ros::Time();
    circle.ns = "beacon_diameter";
    circle.id = 1;
    circle.type = circle.LINE_STRIP ;
    circle.action = circle.ADD;
    circle.pose.position.x = beacon_x ;
    circle.pose.position.y = beacon_y ;
    circle.pose.position.z = beacon_z ;

    circle.pose.orientation.x = 0.0 ;
    circle.pose.orientation.y = 0.0 ;
    circle.pose.orientation.z = 0.0 ;
    circle.pose.orientation.w = 1.0 ;

    circle.scale.x = 0.1;
    circle.scale.y = 0.1;
    circle.scale.z = 0.1;
    circle.color.a = 0.5; // Don't forget to set the alpha!
    circle.color.r = R ;
    circle.color.g = G ;
    circle.color.b = B ;

    ros::Rate rate(10);
    while (ros::ok()) {

        ros::spinOnce();

        // Publishing the corresponding rviz marker
        circle.points.clear() ;
        circle.header.stamp = ros::Time::now();

        for( unsigned i = 0 ; i <= 2*M_PI/angleRes ; i++ ) {
            geometry_msgs::Point p ;
            double angle = i*angleRes ;
            p.x  = scaling * circleRadius * cos(angle) ;
            p.y  = scaling * circleRadius * sin(angle) ;
            p.z  = 0.0                     ;
            circle.points.push_back(p)     ;
        }

        circlePub.publish(circle) ;

        rate.sleep();
    }

    ROS_INFO("ROS-Node Terminated\n");
}
