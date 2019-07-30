/**
 * \Description:
 * this node is used to detect the beacons in the tiled image using "SimpleBlobDetector".
 * It subscribes to:
 * - /camera0/camera_info: this topic contains the information of camera0.
 * - /camera1/camera_info: this topic contains the information of camera1.
 * - /camera2/camera_info: this topic contains the information of camera2.
 * - /camera3/camera_info: this topic contains the information of camera3.
 * - /camera4/camera_info: this topic contains the information of camera4.
 * - /image: the image at which we want to check if there is a beacon or not. this topic will be remapped to another topic contains a real image to be checked, often it is remapped to image tiles.
 * It publishes to:
 * - /beacon distance: the distance to the detected beacon, it is often remapped to represent a specific beacon "e.g /blue/beacon distance".
 * Parameters:
 * - invert: boolean variable to determine if the image should be inverted or not.
 * - HMin: Minimum threshold for Hue channel to be used in the inRange function
 * - HMax: Maximum threshold for Hue channel to be used in the inRange function.
 * - SMin: Minimum threshold for Saturation channel to be used in the inRange function.
 * - SMax: Maximum threshold for Saturation channel to be used in the inRange function.
 * - VMin: Minimum threshold for Value channel to be used in the inRange function.
 * - VMax: Maximum threshold for Value channel to be used in the inRange function.
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
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

//ROS
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs>

// Include here the ".h" files corresponding to the topic type you use.
// ...

using namespace cv;
using namespace std;

// You may have a number of globals here.
cv::Mat image_copy,  morph_im;
bool is_image_available = false;

sensor_msgs::CameraInfo cam0_info;
sensor_msgs::CameraInfo cam1_info;
sensor_msgs::CameraInfo cam2_info;
sensor_msgs::CameraInfo cam3_info;
sensor_msgs::CameraInfo cam4_info;

bool cam0_info_available;
bool cam1_info_available;
bool cam2_info_available;
bool cam3_info_available;
bool cam4_info_available;

// global variable to keep track of
bool invert_im;

// declare local variables
int HMin, SMin, VMin;
int HMax, SMax, VMax;
Scalar minHSV, maxHSV;
Mat imageHSV, maskHSV, resultHSV;

// globals for blob detector
std::vector<KeyPoint> keypoints;
Mat im_with_keypoints, morph_with_keypoints;

// global publisher
ros::Publisher dist_pub;
double Focal;

// Callback functions...
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    is_image_available = true;
    try{
        // cv::imshow("original", cv_bridge::toCvShare(msg, "bgr8")->image);
        image_copy = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::waitKey(30);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void cam0InfoCallback(sensor_msgs::CameraInfo info_msg)
{
    cam0_info_available = true;
    cam0_info = info_msg;
}

void cam1InfoCallback(sensor_msgs::CameraInfo info_msg)
{
    cam1_info_available = true;
    cam1_info = info_msg;
}

void cam2InfoCallback(sensor_msgs::CameraInfo info_msg)
{
    cam2_info_available = true;
    cam2_info = info_msg;
}

void cam3InfoCallback(sensor_msgs::CameraInfo info_msg)
{
    cam3_info_available = true;
    cam3_info = info_msg;
}

void cam4InfoCallback(sensor_msgs::CameraInfo info_msg)
{
    cam4_info_available = true;
    cam4_info = info_msg;
}

// Auxiliary functions...
void HSVThreshold()
{
    // Get values from the BGR trackbar
    minHSV = Scalar(HMin, SMin, VMin);
    maxHSV = Scalar(HMax, SMax, VMax);

    // Convert the BGR image to other color spaces
    if (invert_im)
        cvtColor(~image_copy, imageHSV, COLOR_BGR2HSV);
    else
        cvtColor(image_copy, imageHSV, COLOR_BGR2HSV);
    // Create the mask using the min and max values obtained from trackbar and apply bitwise and operation to get the results
    inRange(imageHSV, minHSV, maxHSV, maskHSV);
    //resultHSV = Mat::zeros(image_copy.rows, image_copy.cols, CV_8UC3);
    //bitwise_and(image_copy, image_copy, resultHSV, maskHSV);
}

void writeText(string text, Mat img, Point2f pt)
{
    int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 0.8;
    int thickness = 2;

    int baseline = 0;
    Size textSize = getTextSize(text, fontFace,
                                fontScale, thickness, &baseline);
    baseline += thickness;

    // center the text
    Point textOrg((pt.x - textSize.width/2),
                  (pt.y + textSize.height/2));

    //    // draw the box
    //    rectangle(img, textOrg + Point(0, baseline),
    //              textOrg + Point(textSize.width, -textSize.height),
    //              Scalar(0,0,255));
    //    // ... and the baseline first
    //    line(img, textOrg + Point(0, thickness),
    //         textOrg + Point(textSize.width, thickness),
    //         Scalar(0, 0, 255));

    // then put the text itself
    putText(img, text, textOrg, fontFace, fontScale,
            Scalar::all(255), thickness, 8);
}

void publish_blob_dist()
{
    KeyPoint max_keypoint(Point2f(1,1), 1);
    for (auto keypoint : keypoints)
        if (keypoint.size > max_keypoint.size) max_keypoint = keypoint;

    int cam = (int) max_keypoint.pt.x / cam0_info.width;
    float dist2obj, offset = 0;

    switch (cam) {
    case 0: dist2obj = (cam0_info.K[0] * 45) / max_keypoint.size + offset; break;
    case 1: dist2obj = (cam1_info.K[0] * 45) / max_keypoint.size + offset; break;
    case 2: dist2obj = (cam2_info.K[0] * 45) / max_keypoint.size + offset; break;
    case 3: dist2obj = (cam3_info.K[0] * 45) / max_keypoint.size + offset; break;
    case 4: dist2obj = (cam4_info.K[0] * 45) / max_keypoint.size + offset; break;
    default: break;
    }

    float focal;
    switch (cam) {
    case 0: focal = cam0_info.K[0]; break;
    case 1: focal = cam1_info.K[0]; break;
    case 2: focal = cam2_info.K[0]; break;
    case 3: focal = cam3_info.K[0]; break;
    case 4: focal = cam4_info.K[0]; break;
    default: break;
    }

    writeText(std::to_string(dist2obj) + " cm", im_with_keypoints, max_keypoint.pt);
    writeText("f = " + std::to_string(focal), im_with_keypoints,
              Point2f(cam0_info.width*cam + 140, cam0_info.height - 30));
    writeText("P = " + std::to_string(max_keypoint.size), im_with_keypoints,
              Point2f(cam0_info.width*cam + 140, cam0_info.height - 60));


    std_msgs::Float64 dist_msg;
    dist_msg.data = dist2obj;
    dist_pub.publish( dist_msg );
}

int main (int argc, char** argv)
{
    //ROS Initialization
    ros::init(argc, argv, "interactive_color_segment");

    // Define your node handles
    ros::NodeHandle nh, nh_loc("~");
    image_transport::ImageTransport it(nh);

    // Read the node parameters if any
    // nh.param("Focal", Focal, 475.55); // improvement because of varying focal length for each lens inside the OCCAM omni camera
    nh_loc.param("HMin", HMin, 0);
    nh_loc.param("HMax", HMax, 180);
    nh_loc.param("SMin", SMin, 0);
    nh_loc.param("SMax", SMax, 255);
    nh_loc.param("VMin", VMin, 0);
    nh_loc.param("VMax", VMax, 255);

    // For red detection mainly
    nh_loc.param("invert", invert_im, false);

    // Declare your node's subscriptions and service clients
    image_transport::Subscriber im_subs = it.subscribe("image", 1, imageCallback);
    ros::Subscriber info_sub1           = nh.subscribe<sensor_msgs::CameraInfo>("/camera0/camera_info", 1, cam0InfoCallback);
    ros::Subscriber info_sub2           = nh.subscribe<sensor_msgs::CameraInfo>("/camera1/camera_info", 1, cam1InfoCallback);
    ros::Subscriber info_sub3           = nh.subscribe<sensor_msgs::CameraInfo>("/camera2/camera_info", 1, cam2InfoCallback);
    ros::Subscriber info_sub4           = nh.subscribe<sensor_msgs::CameraInfo>("/camera3/camera_info", 1, cam3InfoCallback);
    ros::Subscriber info_sub5           = nh.subscribe<sensor_msgs::CameraInfo>("/camera4/camera_info", 1, cam4InfoCallback);

    // Declare you publishers and service servers
    dist_pub = nh.advertise<std_msgs::Float64>("beacon_distance", 1);

    // node initilization
    // creating windows to display images
    //namedWindow("HSV thresholding", WINDOW_NORMAL);
    //namedWindow("morphology", WINDOW_NORMAL);
    namedWindow("keypoints", WINDOW_NORMAL);
    cv::startWindowThread();

    // moving the windows to stack them horizontally
    int offset = 300;
    int initialX = 50;
    int initialY = 50;
    //moveWindow("HSV thresholding", initialX, initialY);
    //moveWindow("morphology", initialX + 2 * (offset + 5), initialY);
    moveWindow("keypoints", initialX + 4 * (offset + 5), initialY);

    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Filter by Color
    params.filterByColor = true;
    params.blobColor = 255;

    // Change thresholds
    params.thresholdStep = 80;
    params.minThreshold = 20;
    params.maxThreshold = 120;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 300;
    params.maxArea = 160000;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.7;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.8;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.5;

    // Set up the detector with parameters.
#if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2
    SimpleBlobDetector detector(params);
#else
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
#endif

    /// Default morphology params
    int morph_elem = 2;     // ellipse
    int morph_size = 5;     // 2(morph_size) + 1
    int morph_operator = 0; // opening

    // rate
    ros::Rate rate(50);
    while (ros::ok()){
        ros::spinOnce();

        // Your node's code goes here.
        if ( !is_image_available || !cam0_info_available || !cam1_info_available ||
             !cam2_info_available || !cam3_info_available || !cam4_info_available) {
            ROS_INFO("Waiting for image.") ;
            rate.sleep() ;
            continue ;
        }

        ros::Time begin = ros::Time::now();

        // Threshold for specific color
        HSVThreshold(); // maskHSV and resultHSV


        // Since MORPH_X : 2,3,4,5 and 6
        int operation = morph_operator + 2;
        Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
        /// Apply the specified morphology operation
        morphologyEx( maskHSV, morph_im, operation, element );


        // Detect blobs.
#if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2
        detector.detect(morph_im, keypoints);
        for ( auto &keypoint : keypoints )
            keypoint.size *= 2;
#else
        detector->detect(maskHSV, keypoints);
#endif

        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        //drawKeypoints( morph_im, keypoints, morph_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        drawKeypoints( image_copy, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

        // Triangle similarity -> Focal_length = (distance2object * pixel_width) / actual_width
        if (keypoints.size() > 0) {
            publish_blob_dist();
            //writeText(std::to_string(keypoints[0].size), im_with_keypoints, Point2f(120, 20));
        }

        ros::Duration imProc_duration = ros::Time::now() - begin;
        writeText(std::to_string(imProc_duration.toSec()) + " s", im_with_keypoints, Point2f(120, 20));


        // Show blobs
        //imshow("HSV thresholding", resultHSV);
        //imshow("morphology", morph_with_keypoints);
        imshow("keypoints", im_with_keypoints);

        rate.sleep();
    }
}
