/**
 * \Description:
 * this node is used to implement the Extended kalman Filter equations, its input are the prediction and the measurement, and its output is final estimation.
 * It subscribes to:
 * - /odom2D: this topic contains the 2D pose (x,y,theta) of the turtlrbot and their covariance.
 * - /beacon distances : this topic contains three distance from the robot to each beacon in order "blue, orange/red, green".
 * It publishes to:
 * - /robot_pose ekf: it publishes the 2D pose (x,y,theta) of the turtlrbot and their corresponding covariance after modifying the predicted pose using the measurement.
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
#include <turtle_ekf/Pose2DWithCovariance.h>

//#include <geometry_msgs/PoseWithCovariance.h>
//#include <std_msgs/Float32.h>

// global variables...
turtle_ekf::Pose2DWithCovariance odom2D;
std_msgs::Float64MultiArray dist;
bool is_odom_available= false, is_dist_available=false;


// Callback functions...
void odom_Callback( turtle_ekf::Pose2DWithCovariance odom2D_msg) {
    is_odom_available= true;
    odom2D = odom2D_msg;
}

void msurmnt_Callback ( std_msgs::Float64MultiArray  dist_msg) {
    is_dist_available= true;
    dist = dist_msg;
}


int main (int argc, char** argv){

    //ROS Initialization
    ros::init(argc, argv, "ekf_node");

    // Define your node handles
    ros::NodeHandle nh_loc("~"), nh_glob;

    // Read the node parameters if any

    // Declare your node's subscriptions and service clients
    ros::Subscriber odom2D_sub = nh_glob.subscribe<turtle_ekf::Pose2DWithCovariance>("/odom2D",1, odom_Callback) ;
    ros::Subscriber msurmnt_sub = nh_glob.subscribe<std_msgs::Float64MultiArray>("/beacon_distances",1, msurmnt_Callback) ;

    // Declare you publishers and service servers
    ros::Publisher  pose_pub = nh_glob.advertise<turtle_ekf::Pose2DWithCovariance>("/robot_pose_ekf",1) ;

    // node initilization
    MatrixXd Q_alpha =  MatrixXd::Zero(3,3);
    MatrixXd Q_gamma =  MatrixXd::Zero(3,3);
    MatrixXd P =  MatrixXd::Zero(3,3);
    MatrixXd C =  MatrixXd::Zero(3,3);
    MatrixXd D =  MatrixXd::Zero(3,3);
    MatrixXd K =  MatrixXd::Zero(3,3);
    MatrixXd I =  MatrixXd::Zero(3,3);
    I(0,0)= 1;   I(1,1)= 1;     I(2,2)= 1;

    VectorXd X(3);
    VectorXd Y(3);
    VectorXd Y_hat(3);

    geometry_msgs::Point B1, B2, B3;
    B1.x=0;  B1.y=0; B3.x=4.40; B3.y=3.20; B2.x=0;  B2.y=5.20;

    // Init
    X(0)=1;   X(1)=1;   X(2)= 0; //M_PI/2;
// init matrices
//    for (int i=0; i++; i<3){
//        for(int j=0; j++; j<3){
//            P(i,j)= 0;
//            C(i,j)= 0;
//            D(i,j)= 0;
//            K(i,j)= 0;
//        }
//    }


    ros::Rate rate(10);

    while (ros::ok()){

        ros::spinOnce();
                if( ! is_dist_available || !is_odom_available ){
                    ROS_INFO("Waiting for odom2D and/or dist") ;
                    rate.sleep() ;
                    continue ;
                }

        // odometry part:
        // Prediction step: predicted_state Xk+1/k "extract pose from odom"
        X(0)= odom2D.pose.x;
        X(1)= odom2D.pose.y;
        X(2)= odom2D.pose.theta;

        // state_trans_uncertainty_noise: Q_alpha "extract covariance matrix from odom"
        Q_alpha(0,0)= odom2D.Covariance[0];
        Q_alpha(1,1)= odom2D.Covariance[4];
        Q_alpha(2,2)= odom2D.Covariance[8];

        // jacobian_matrix: A = I

        // pred_covariance_est: Pk+1/k = A. PK/k. A^T + Q_alpha  "P= A*P*A.transpose() + Q_alpha;" but A=I
        P=P+Q_alpha;

        // measurement part:

        // Actual measurement  Y
        Y(0)= dist.data[0]/100;
        Y(1)= dist.data[1]/100;
        Y(2)= dist.data[2]/100;

        // Expected measurement Y_hat
        Y_hat(0)= sqrt( (B1.x-X(0))*(B1.x-X(0)) + (B1.y-X(1))*(B1.y-X(1)) );
        Y_hat(1)= sqrt( (B2.x-X(0))*(B2.x-X(0)) + (B2.y-X(1))*(B2.y-X(1)) );
        Y_hat(2)= sqrt( (B3.x-X(0))*(B3.x-X(0)) + (B3.y-X(1))*(B3.y-X(1)) );


cout<< "Y:  "<< Y(0) << ", " << Y(1) << ", " <<  Y(2) << endl<< endl;
cout<< "Y_hat:  " << Y_hat(0) << ", " << Y_hat(1) << ", " <<  Y_hat(2) <<  endl ;

        // Measurement jacobian C_matrix
        C(0,0)= 2*(X(0) - B1.x) / Y_hat(0);
        C(0,1)= 2*(X(1) - B1.y) / Y_hat(0);
        C(1,0)= 2*(X(0) - B1.x) / Y_hat(1);
        C(1,1)= 2*(X(1) - B1.y) / Y_hat(1);
        C(2,0)= 2*(X(0) - B1.x) / Y_hat(2);
        C(2,1)= 2*(X(1) - B1.y) / Y_hat(2);

        // measurement covariance Q_gamma
        Q_gamma(0,0)= 10;
        Q_gamma(1,1)= 10;
        Q_gamma(2,2)= 10;

        if (Y(0)> 0.3 && Y(0)< 6)
        Q_gamma(0,0)= (.0000842*Y(0))+0.0158;
        if (Y(1)> 0.3 && Y(1)< 6)
        Q_gamma(1,1)= (.0000842*Y(1))+0.0158;
        if (Y(2)> 0.3 && Y(2)< 6)
        Q_gamma(2,2)= (.0000842*Y(2))+0.0158;


if (Y(0)> 0.3 && Y(0)< 6 && Y(1)> 0.3 && Y(1)< 6 && Y(2)> 0.3 && Y(2)< 6 ){

       // Residual covariance D is Q_(Y-Y_hat)
       D= ( C*P*C.transpose() + Q_gamma );

       // Correction propotional gain  K= P*C.transpose() * (C*P*C.transpose() + Q_gamma )^-1
        K= P*C.transpose() * D.reverse();


        // Estimation Step: estimated_state Xk+1/k+1 = Xk+1/k + Kk (Yk - Yk_hat)
        X = X + K*(Y-Y_hat);

        // Estimated_covariance_est: Pk+1/k+1 = ( I - Kk*Ck ) * Pk+1/k
        P= ( I - K*C ) * P;
}
        std::cout<<  "P" << P << endl << endl;
        std::cout<<  "C" << C << endl << endl;
        std::cout<<  "D" << D << endl << endl;
        std::cout<<  "K" << K << endl << endl;

        // publish the final pose ........................................................................................................
        turtle_ekf::Pose2DWithCovariance pose_ekf;
        pose_ekf.pose.x = X(0);
        pose_ekf.pose.y = X(1);
        pose_ekf.pose.theta =  X(2);

        pose_ekf.Covariance[0] =  P(0,0);
        pose_ekf.Covariance[1] =  0;
        pose_ekf.Covariance[2] =  0;
        pose_ekf.Covariance[3] =  0;
        pose_ekf.Covariance[4] =  P(1,1);
        pose_ekf.Covariance[5] =  0;
        pose_ekf.Covariance[6] =  0;
        pose_ekf.Covariance[7] =  0;
        pose_ekf.Covariance[8] =  P(2,2);

        pose_pub.publish(pose_ekf);




        rate.sleep();
    }


}
