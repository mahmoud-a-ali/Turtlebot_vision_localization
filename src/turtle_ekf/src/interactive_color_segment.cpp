/**
 * \Description:
 * this node is used to set Min and Max thresholds for HSV color space using interactive mode with track bars, to give reliable detection for the beacons.
 *
 * ALI&SERRANO_ECN_M1_2017
 */

//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//ROS
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs>

// Include here the ".h" files corresponding to the topic type you use.
// ...

using namespace cv;
using namespace std;

// You may have a number of globals here.
cv::Mat image_copy, placeholder;
bool is_image_available = false;

// global variable to keep track of
bool show = false;

// declare local variables
int HMin, SMin, VMin;
int HMax, SMax, VMax;
Scalar minHSV, maxHSV;

Mat imageHSV, maskHSV, resultHSV;

// Callback functions...
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    is_image_available = true;
    try{
        cv::imshow("original", cv_bridge::toCvShare(msg, "bgr8")->image);
        image_copy = cv_bridge::toCvCopy(msg, "bgr8")->image;
        //image_copy = ~image_copy;
        cv::waitKey(30);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

// Create a callback for event on trackbars
void onTrackbarActivity(int pos, void* userdata){
    // Just update the global variable that there is an event
    show = true;
    return;
}

void interactiveThresholds() {
    show = false;

    // Get values from the BGR trackbar
    minHSV = Scalar(HMin, SMin, VMin);
    maxHSV = Scalar(HMax, SMax, VMax);

    // Convert the BGR image to other color spaces
    cvtColor(image_copy, imageHSV, COLOR_BGR2HSV);

    // Create the mask using the min and max values obtained from trackbar and apply bitwise and operation to get the results
    inRange(imageHSV, minHSV, maxHSV, maskHSV);
    resultHSV = Mat::zeros(image_copy.rows, image_copy.cols, CV_8UC3);
    bitwise_and(image_copy, image_copy, resultHSV, maskHSV);

    imshow("SelectHSV",resultHSV);
}

int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "interactive_color_segment");

    // Define your node handles
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Read the node parameters if any
    // ...

    // Declare your node's subscriptions and service clients
    image_transport::Subscriber im_subs = it.subscribe("image", 1, imageCallback);

    // Declare you publishers and service servers
    image_transport::Publisher im_pub = it.advertise("th_image", 1);
    image_transport::Publisher mask_pub = it.advertise("mask", 1);

    // creating windows to display images
    cv::namedWindow("original");
    namedWindow("SelectHSV", WINDOW_NORMAL);
    cv::startWindowThread();

    // image resize width and height
    int offset = 300;

    // position on the screen where the windows start
    int initialX = 50;
    int	initialY = 50;

    // moving the windows to stack them horizontally
    // moveWindow("original", initialX, initialY);
    moveWindow("SelectHSV", initialX, initialY);

    // Blue beacon thresholding
    HMin = 102; HMax = 115; SMin = 130; SMax = 250; VMin = 37; VMax = 235;

    // creating trackbars to get values for HSV
    createTrackbar("HMin", "SelectHSV", &HMin, 180, onTrackbarActivity);
    createTrackbar("HMax", "SelectHSV", &HMax, 180, onTrackbarActivity);
    createTrackbar("SMin", "SelectHSV", &SMin, 255, onTrackbarActivity);
    createTrackbar("SMax", "SelectHSV", &SMax, 255, onTrackbarActivity);
    createTrackbar("VMin", "SelectHSV", &VMin, 255, onTrackbarActivity);
    createTrackbar("VMax", "SelectHSV", &VMax, 255, onTrackbarActivity);

    // show all images initially
    Mat empty = Mat::zeros(480, 640, CV_8UC3);
    imshow("SelectHSV", empty);

    // initialize image message
    sensor_msgs::ImagePtr im_msg;
    sensor_msgs::ImagePtr mask_msg;

    // rate
    ros::Rate rate(50);
    while (ros::ok()){
        ros::spinOnce();

        // Your node's code goes here.
        if( !is_image_available ){
            ROS_INFO("Waiting for image.") ;
            rate.sleep() ;
            continue ;
        }

        //if (show)
        interactiveThresholds();

        im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resultHSV).toImageMsg();
        im_pub.publish(im_msg);

        mask_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", maskHSV).toImageMsg();
        mask_pub.publish(mask_msg);

        rate.sleep();
    }
}
