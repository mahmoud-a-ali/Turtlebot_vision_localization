/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include "tf/transform_broadcaster.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <turtle_ekf/Pose2DWithCovariance.h>

turtle_ekf::Pose2DWithCovariance robot_pose;
bool robot_pose_available = false;

class TransformSender
{
public:
    ros::NodeHandle node_;
    //constructor
    TransformSender(double x, double y, double z, double yaw, double pitch, double roll, ros::Time time, const std::string& frame_id, const std::string& child_frame_id)
    {
        tf::Quaternion q;
        q.setRPY(roll, pitch,yaw);
        transform_ = tf::StampedTransform(tf::Transform(q, tf::Vector3(x,y,z)), time, frame_id, child_frame_id );
    };
    TransformSender(double x, double y, double z, double qx, double qy, double qz, double qw, ros::Time time, const std::string& frame_id, const std::string& child_frame_id) :
        transform_(tf::Transform(tf::Quaternion(qx,qy,qz,qw), tf::Vector3(x,y,z)), time, frame_id, child_frame_id){};
    //Clean up ros connections
    ~TransformSender() { }

    //A pointer to the rosTFServer class
    tf::TransformBroadcaster broadcaster;

    // A function to call to send data periodically
    void send (ros::Time time) {
        transform_.stamp_ = time;
        broadcaster.sendTransform(transform_);
    };

private:
    tf::StampedTransform transform_;

};

// Callback functions
void getKeyCallback(std_msgs::Int16 key_msg)
{
    //key_msg.data == 43 ? x += 0.25: x -= 0.25;
}

void getPoseCallback(turtle_ekf::Pose2DWithCovariance pose_msg)
{
    robot_pose_available = true;
    robot_pose = pose_msg;
}

int main(int argc, char ** argv)
{
    //Initialize ROS
    ros::init(argc, argv,"my_static_transform_publisher", ros::init_options::AnonymousName);

    ros::NodeHandle nh, nh_("~");  //ROS Handler - global, local namespace.

    // Subscribers
    ros::Subscriber key_sub  = nh.subscribe<std_msgs::Int16>("/key_typed", 1, getKeyCallback);
    //ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose2D>("/robot_pose", 1, getPoseCallback);
    ros::Subscriber pose_sub = nh.subscribe<turtle_ekf::Pose2DWithCovariance>("/robot_pose", 1, getPoseCallback);

    int period = 50; // 20 Hz
    ros::Duration sleeper(period/1000.0);

    if (argc < 3)
        ROS_FATAL("Specify frame_id and child_frame_id");

    TransformSender tf_sender(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              ros::Time() + sleeper, //Future dating to allow slower sending w/o timeout
                              argv[1], argv[2]);

    while(tf_sender.node_.ok())
    {
        ros::spinOnce();

        if (!robot_pose_available) {
            sleeper.sleep();
            continue;
        }
        // Node code goes here
        TransformSender tf_sender(robot_pose.pose.x,
                                  robot_pose.pose.y,
                                  0.0,
                                  robot_pose.pose.theta,
                                  0.0, 0.0,
                                  ros::Time() + sleeper, //Future dating to allow slower sending w/o timeout
                                  argv[1], argv[2]);
        tf_sender.send(ros::Time::now() + sleeper);
        ROS_DEBUG("Sending transform from %s with parent %s\n", argv[1], argv[2]);

        sleeper.sleep();
    }

    return 0;

};
