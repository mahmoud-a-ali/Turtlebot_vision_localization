/**
 * \Description:
 * this node is used to estimate the absoulte location(x,y) of the robot using Trilteration.
 * It subscribe to:
 * - /beacon_distances
 * It publishes to:
 * - /pose
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
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
using Eigen::MatrixXd;

//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic type you use.
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/Float64MultiArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

//#include <geometry_msgs/PoseWithCovariance.h>
//#include <std_msgs/Float32.h>

// global variables...
std_msgs::Float64MultiArray dist;
bool  is_dist_available=false;


void msurmnt_Callback ( std_msgs::Float64MultiArray  dist_msg) {
    is_dist_available= true;
    dist = dist_msg;
}

int circle_circle_intersection(double x0, double y0, double r0,
                               double x1, double y1, double r1,
                               double &xi, double &yi,
                               double &xi_prime, double &yi_prime)
{
    double a, dx, dy, d, h, rx, ry;
    double x2, y2;

    // dx and dy are the vertical and horizontal distances between the circle centers.
    dx = x1 - x0;
    dy = y1 - y0;

    // Determine the straight-line distance between the centers.
    //d = sqrt((dy*dy) + (dx*dx));
    d = hypot(dx,dy); // Suggested by Keith Briggs

    // Check for solvability.
    if (d > (r0 + r1)) /* no solution. circles do not intersect. */
        return 0;
    if (d < fabs(r0 - r1)) /* no solution. one circle is contained in the other */
        return 0;


    /* 'point 2' is the point where the line through the circle
   * intersection points crosses the line between the circle centers.
   */

    /* Determine the distance from point 0 to point 2. */
    a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

    /* Determine the coordinates of point 2. */
    x2 = x0 + (dx * a/d);
    y2 = y0 + (dy * a/d);

    // Determine the distance from point 2 to either of the intersection points.
    h = sqrt((r0*r0) - (a*a));

    // Now determine the offsets of the intersection points from point 2.
    rx = -dy * (h/d);
    ry = dx * (h/d);

    /* Determine the absolute intersection points. */
    xi = x2 + rx;
    xi_prime = x2 - rx;
    yi = y2 + ry;
    yi_prime = y2 - ry;

    return 1;
}

void select_point(double p1x, double p1y, double p2x, double p2y, double bx, double by, double &px, double &py, double d){
    double t1= fabs (sqrt( (bx- p1x)*(bx-p1x) + (by-p1y)*(by-p1y)) );
    double t2= fabs (sqrt( (bx-p2x)*(bx-p2x) + (by-p2y)*(by-p2y)) );
    if ( fabs(d-t1) < fabs(d-t2) )
       { px= p1x;  py=p1y; }
    else
       { px= p2x;  py=p2y; }
}

void incircle(double p1x, double p1y, double p2x, double p2y, double p3x, double p3y, double &xc, double &yc){
    double a1 =fabs (sqrt( (p3x-p2x)*(p3x-p2x) + (p3y-p2y)*(p3y-p2y)) );
    double a2 =fabs (sqrt( (p1x-p3x)*(p1x-p3x) + (p1y-p3y)*(p1y-p3y)) );
    double a3 =fabs (sqrt( (p1x-p2x)*(p1x-p2x) + (p1y-p2y)*(p1y-p2y)) );
    xc= ( a1*p1x + a2*p2x + a3*p3x )/(a1+a2+a3);
    yc= ( a1*p1y + a2*p2y + a3*p3y )/(a1+a2+a3);
}


int main (int argc, char** argv){

    //ROS Initialization
    ros::init(argc, argv, "ekf_node");

    // Define your node handles
    ros::NodeHandle nh_loc("~"), nh_glob;

    // Read the node parameters if any

    // Declare your node's subscriptions and service clients
    ros::Subscriber msurmnt_sub = nh_glob.subscribe<std_msgs::Float64MultiArray>("/beacon_distances",1, msurmnt_Callback) ;

    // Declare you publishers and service servers
    ros::Publisher  pose_pub = nh_glob.advertise<geometry_msgs::Pose2D>("/pose",1) ;

    // node initilization

    //geometry_msgs::Point B1, B2, B3;
    //B1.x=0;  B1.y=10; B2.x=10;  B2.y=0; B3.x=20; B3.y=20;

    //double x1=0, x2=0, x3=10, y1=5, y2=10, y3=10, x, y, d1, d2, d3;
    double d1, d2, d3, xc, yc;
    ros::Rate rate(10);

    while (ros::ok()){

        ros::spinOnce();
                if( ! is_dist_available ){
                    ROS_INFO("Waiting for odom/dist.") ;
                    rate.sleep() ;
                    continue ;
                }

        // measurement part:

        // Actual measurement  Y
        d1= dist.data[0];
        d2= dist.data[1];
        d3= dist.data[2];

        //double d1=5, d2=5, d3=1;
        double b1x=0, b1y=0, b2x=0, b2y=190, b3x=350, b3y=0;
        double ip12x1, ip12y1, ip12x2, ip12y2,   ip13x1, ip13y1, ip13x2, ip13y2,   ip23x1, ip23y1, ip23x2, ip23y2;

        int t12= circle_circle_intersection(b1x, b1y, d1, b2x, b2y, d2, ip12x1, ip12y1, ip12x2, ip12y2); //12
        int t13= circle_circle_intersection(b1x, b1y, d1, b3x, b3y, d3, ip13x1, ip13y1, ip13x2, ip13y2); //13
        int t23= circle_circle_intersection(b2x, b2y, d2, b3x, b3y, d3, ip23x1, ip23y1, ip23x2, ip23y2); //23

        if ( t12 && t13 && t23){
            //std::cout<< "point1: (" << ip1x << ", " << ip1y << ")" << ")"<< std::endl;
            //std::cout<< "point2: (" << ip2x << ", " << ip2y << ")" << ")"<< std::endl;
            double ip1x, ip1y, ip2x, ip2y, ip3x, ip3y;

            select_point(ip12x1, ip12y1, ip12x2, ip12y2, b3x, b3y, ip1x, ip1y, d3 );
            select_point(ip13x1, ip13y1, ip13x2, ip13y2, b2x, b2y, ip2x, ip2y, d2 );
            select_point(ip23x1, ip23y1, ip23x2, ip23y2, b1x, b1y, ip3x, ip3y, d1 );

            std::cout<< "point1: (" << ip1x << ", " << ip1y << ")" << ")"<< std::endl;
            std::cout<< "point2: (" << ip2x << ", " << ip2y << ")" << ")"<< std::endl;
            std::cout<< "point3: (" << ip3x << ", " << ip3y << ")" << ")"<< std::endl;

            incircle(ip1x, ip1y, ip2x, ip2y, ip3x, ip3y, xc, yc);
            std::cout<< "center: (" << xc << ", " << yc << ")" << ")"<< std::endl;


        }
        else
            std::cout<<"no common intersection"<<std::endl;



//        float c1= (d1*d1) - (x1*x1) - (y1*y1);
//        float c2= (d2*d2) - (x2*x2) - (y2*y2);
//        float c3= (d3*d3) - (x3*x3) - (y3*y3);

//        x= ( ((c1-c2)*(y3-y1)) - ((c1-c3)*(y2-y1)) ) / ( 2*( ( (x2-x1)*(y3-y1) ) - ((x3-x1)*(y2-y1)) ));
//        y= ( c1 - c2 - (2*x*(x2-x1)) ) / (2 * (y2-y1) );
        // measurement covariance Q_gamma
        //Q_gamma(0,0)= .1;  Q_gamma(1,1)= .1;  Q_gamma(2,2)= .1;

        // Estimated_covariance_est: Pk+1/k+1 = ( I - Kk*Ck ) * Pk+1/k
        //P= ( I - K*C ) * P;

        // publish the final pose ........................................................................................................
        geometry_msgs::Pose2D pose;
        pose.x = xc;
        pose.y = yc;
        pose.theta =  0;
        pose_pub.publish(pose);



        rate.sleep();
    }


}
