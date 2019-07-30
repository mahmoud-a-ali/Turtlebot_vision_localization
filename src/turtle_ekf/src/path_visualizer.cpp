
/**
 * \Description:
 * this node is used to publish the pose obtained by the EKF to a marker that can be visualized using RVIZ.
 * It subscribes to:
 * - /pose: that publised by the EKF node.
 * It publishes to:
 * - /visu_path: will be used for visualization in RVIZ.
 *
 * SERRANO&ALI_ECN_M1_2017
 */

//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
using Eigen::MatrixXd;

//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic type you use.
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <turtle_ekf/Pose2DWithCovariance.h>
#include <turtle_ekf/StampedDistances.h>

//#include <geometry_msgs/PoseWithCovariance.h>
//#include <std_msgs/Float32.h>

// global variables...
turtle_ekf::Pose2DWithCovariance poseWCov;
bool is_pose_available= false;

// Callback functions...
void pose_Callback( turtle_ekf::Pose2DWithCovariance odom2D_msg ) {
    is_pose_available= true;
    poseWCov = odom2D_msg;
}


int main (int argc, char** argv){

    //ROS Initialization
    ros::init(argc, argv, "ekf_node");

    // Define your node handles
    ros::NodeHandle nh, nh_loc("~");

    // Read the node parameters if any
    double R, G, B;
    nh_loc.param("R", R, 1.0);
    nh_loc.param("G", G, 1.0);
    nh_loc.param("B", B, 0.5);

    // Declare your node's subscriptions and service clients
    ros::Subscriber poseWCov_sub   = nh.subscribe<turtle_ekf::Pose2DWithCovariance>("pose", 1, pose_Callback) ;

    // Declare you publishers and service servers
    ros::Publisher  pathPub = nh.advertise<visualization_msgs::Marker>("visu_path", 1);

    // beacon diameter depending on measure of camera
    visualization_msgs::Marker pose_path;
    pose_path.header.frame_id = "/map";
    pose_path.header.stamp = ros::Time();
    pose_path.ns = "path_visualizer";
    pose_path.id = 1;
    pose_path.type = pose_path.LINE_STRIP ;
    pose_path.action = pose_path.ADD;
    pose_path.pose.position.x = 1 ;
    pose_path.pose.position.y = 1 ;
    pose_path.pose.position.z = 0 ;

    pose_path.pose.orientation.x = 0.0 ;
    pose_path.pose.orientation.y = 0.0 ;
    pose_path.pose.orientation.z = 0.0 ;
    pose_path.pose.orientation.w = 1.0 ;

    pose_path.scale.x = 0.02;
    pose_path.scale.y = 0.02;
    pose_path.scale.z = 0.02;
    pose_path.color.a = 1; // Don't forget to set the alpha!
    pose_path.color.r = R ;
    pose_path.color.g = G ;
    pose_path.color.b = B ;

    ros::Rate rate(10);
    while (ros::ok()) {

        ros::spinOnce();

        if (!is_pose_available) {
            ROS_INFO("Waiting for pose with covariance...");
            rate.sleep();
            continue;
        }

        // Publishing the corresponding rviz marker
        pose_path.header.stamp = ros::Time::now();
        geometry_msgs::Point p;
        p.x = poseWCov.pose.x - 1;
        p.y = poseWCov.pose.y - 1;
        p.z = 0.0;
        pose_path.points.push_back(p);

        pathPub.publish(pose_path);

        rate.sleep();
    }

    ROS_INFO("ROS-Node Terminated\n");
}
