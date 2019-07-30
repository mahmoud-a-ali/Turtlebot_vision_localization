
/**
 * \Description:
 * this node is used to rectify the image published by one camera.
 * It subscribes to:
 * - /camera info: this topic contains the information of the camera "interinsic paramters and distortion coefficients" required to the rectification process.
 * - /image raw; this topic contains the image itself that will be rectified.
 * It publishes to:
 * - /image rect: it publishes the rectified image through this topic.
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

// OpenCV
#include <opencv2/highgui/highgui.hpp>

//ROS
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>

// Include here the ".h" files corresponding to the topic type you use.
// ...

// You may have a number of globals here.
cv::Mat image_copy, placeholder;
bool is_image_available = false;

sensor_msgs::CameraInfo cam_info;
bool cam_info_available;

image_transport::Publisher im_pub;

// Callback functions...
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    is_image_available = true;
    try{
        //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        image_copy = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::waitKey(30);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    if (cam_info_available) {
        // setup matrices to use in undistort function
        cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                camera_matrix.at<double>(i, j) = cam_info.K[3*i + j];
        cv::Mat distortion = cv::Mat::eye(1, 5, CV_64F);
        for (int i = 0; i < 5; i++)
            distortion.at<double>(0, i) = cam_info.D[i];

        cv::Mat temp = image_copy.clone();
        cv::undistort(temp, image_copy, camera_matrix, distortion);

        sensor_msgs::ImagePtr im_msg;
        im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_copy).toImageMsg();
        im_pub.publish(im_msg);
    }

}

void camInfoCallback(sensor_msgs::CameraInfo info_msg)
{
    cam_info_available = true;
    cam_info = info_msg;
}

int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "image_rectifier");

    // Define your node handles
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Read the node parameters if any
    // ...

    // Declare your node's subscriptions and service clients
    image_transport::Subscriber im_subs = it.subscribe("image_raw", 1, imageCallback);
    ros::Subscriber info_sub            = nh.subscribe<sensor_msgs::CameraInfo>("camera_info", 1, camInfoCallback);

    // Declare you publishers and service servers
    im_pub = it.advertise("image_rect", 1);

    // rate
    ros::Rate rate(30);
    while (ros::ok()){
        ros::spinOnce();

        // Node code here

        rate.sleep();
    }
}
