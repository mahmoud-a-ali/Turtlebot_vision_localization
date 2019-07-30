
/**
 * \Description:
 * this node is used to reorder the distances to beacon to follow standard order" blue - red - green "
 * It subscribes to:
 * - /blue/beacon_distance
 * - /red/beacon_distance
 * - /green/beacon_distance
 * It publishes to:
 * - /beacon_distances
 *
 * ALI&SERRANO_ECN_M1_2017
 */

//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <math.h>

//OpenCV
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>

//ROS
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
//#include <sensor_msgs>

// Include here the ".h" files corresponding to the topic type you use.
// ...

using namespace std;

// You may have a number of globals here.
bool blue_update = false;
bool red_update = false;
bool green_update = false;
float blue_dist, red_dist, green_dist;

// Callback functions...
void getBlueDistanceCallback(std_msgs::Float64 dist)
{
    blue_update = true;
    blue_dist = dist.data;
}

void getRedDistanceCallback(std_msgs::Float64 dist)
{
    red_update = true;
    red_dist = dist.data;
}

void getGreenDistanceCallback(std_msgs::Float64 dist)
{
    green_update = true;
    green_dist = dist.data;
}

// Auxiliary functions
//...

int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "blob_dist_interface");

    // Define your node handles
    ros::NodeHandle nh, nh_loc("~");

    // Declare your subscribers
    ros::Subscriber blue_sub = nh.subscribe<std_msgs::Float64>("/blue/beacon_distance", 1, getBlueDistanceCallback);
    ros::Subscriber red_sub = nh.subscribe<std_msgs::Float64>("/red/beacon_distance", 1, getRedDistanceCallback);
    ros::Subscriber green_sub = nh.subscribe<std_msgs::Float64>("/green/beacon_distance", 1, getGreenDistanceCallback);

    // Declare you publishers and service servers
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float64MultiArray>("beacon_distances", 1);

    // node initilization
    // ...

    // rate
    ros::Rate rate(30); // Hz
    while (ros::ok()){
        ros::spinOnce();

        // Your node's code goes here.
        //if ( blue_update && red_update && green_update ) {
        // set-up and send message
        std_msgs::Float64MultiArray dist_msg;
        dist_msg.data.push_back( 0.9*blue_dist );
        dist_msg.data.push_back( 0.9*red_dist );
        dist_msg.data.push_back( 0.9*green_dist );
        dist_pub.publish( dist_msg );

        rate.sleep();
    }
}
