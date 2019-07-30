/**
 * \Description:
 * this node is used to combine the 5 images from the 5 different cameras -transmitted by OCCAM 60 Camera- to obtain a single panoramic image.
 * It subscribes to:
 * - /camera0/image rect: the rectified image obtained from camera0.
 * - /camera1/image rect: the rectified image obtained from camera1.
 * - /camera2/image rect: the rectified image obtained from camera2.
 * - /camera3/image rect: the rectified image obtained from camera3.
 * - /camera4/image rect: the rectified image obtained from camera4.
 * It publishes to:
 * - /image tiles: the tiles image obtained by combing the 5 images.
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

// Include here the ".h" files corresponding to the topic type you use.
// ...

// You may have a number of globals here.
cv::Mat image0, image1, image2, image3, image4;
bool is_image0_available = false;
bool is_image1_available = false;
bool is_image2_available = false;
bool is_image3_available = false;
bool is_image4_available = false;

image_transport::Publisher im_pub;

// Callback functions...
void img0Callback(const sensor_msgs::ImageConstPtr& msg)
{
    is_image0_available = true;
    try{
        image0 = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::waitKey(30);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void img1Callback(const sensor_msgs::ImageConstPtr& msg)
{
    is_image1_available = true;
    try{
        image1 = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::waitKey(30);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void img2Callback(const sensor_msgs::ImageConstPtr& msg)
{
    is_image2_available = true;
    try{
        image2 = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::waitKey(30);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void img3Callback(const sensor_msgs::ImageConstPtr& msg)
{
    is_image3_available = true;
    try{
        image3 = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::waitKey(30);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void img4Callback(const sensor_msgs::ImageConstPtr& msg)
{
    is_image4_available = true;
    try{
        image4 = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::waitKey(30);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "image_tiles");

    // Define your node handles
    ros::NodeHandle nh, nh_("~");
    image_transport::ImageTransport it(nh);

    // Read the node parameters if any
    int img_width, img_height, n_imgs;
    nh_.param("img_width", img_width, 752);
    nh_.param("img_height", img_height, 480);
    nh_.param("n_imgs", n_imgs, 5);

    // Declare node's subscriptions and service clients
    image_transport::Subscriber im0_subs = it.subscribe("/camera0/image_rect", 1, img0Callback);
    image_transport::Subscriber im1_subs = it.subscribe("/camera1/image_rect", 1, img1Callback);
    image_transport::Subscriber im2_subs = it.subscribe("/camera2/image_rect", 1, img2Callback);
    image_transport::Subscriber im3_subs = it.subscribe("/camera3/image_rect", 1, img3Callback);
    image_transport::Subscriber im4_subs = it.subscribe("/camera4/image_rect", 1, img4Callback);

    // Declare publishers and service servers
    im_pub = it.advertise("image_tiles", 1);

    // rate
    ros::Rate rate(40);
    while (ros::ok()){
        ros::spinOnce();

        // Node code here
        if ( !is_image0_available || !is_image1_available || !is_image2_available ||
             !is_image3_available || !is_image4_available ) {
            ROS_INFO("Waiting for all images");
            rate.sleep();
            continue;
        }

        std::vector<cv::Mat> images {image0, image1, image2, image3, image4};
        cv::Mat img_tiles(img_height, img_width * n_imgs, CV_8UC3);
        for(int i = 0; i < n_imgs; i++) {
            cv::Rect roi(i*img_width, 0, img_width, img_height);
            images[i].copyTo( img_tiles(roi) );
        }

        sensor_msgs::ImagePtr im_msg;
        im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_tiles).toImageMsg();
        im_pub.publish(im_msg);

        rate.sleep();
    }
}
