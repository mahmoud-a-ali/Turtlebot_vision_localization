/**
 * \Description:
 * this node is used to estimate the absoulte location of the robot using the optimization library  "RobOptim".
 * It subscribe to:
 * - /beacon_distances
 * It publishes to:
 * - /robot_pose
 *
 * SERRANO&ALI_ECN_M1_2017
 */

// Cpp
#include <stdexcept>
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <math.h>

// Boost
#include <boost/shared_ptr.hpp>

// RobOptim
#include <roboptim/core/linear-function.hh>
#include <roboptim/core/differentiable-function.hh>
#include <roboptim/core/twice-differentiable-function.hh>
#include <roboptim/core/io.hh>
#include <roboptim/core/solver.hh>
#include <roboptim/core/solver-factory.hh>

//ROS
#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64MultiArray.h>

// blue beacon coordinates
#define X1 0
#define Y1 0
// red beacon coordinates
#define X2 0
#define Y2 190
// green beacon coordinates
#define X3 350
#define Y3 0
// upper and lower bounds of map
#define LB 0
#define UB 400

// global variables
float blue_distance, red_distance, green_distance;
bool are_dist_available = false;
ros::Publisher pose_pub;

using namespace roboptim;
using namespace std;

// function to minimize
struct F : public TwiceDifferentiableFunction
{
    F () : TwiceDifferentiableFunction (2, 1, "(x[0]-X1)^2 + (x[1]-Y1)^2 + (x[0]-X2)^2 + (x[1]-Y2)^2 + (x[0]-X3)^2 + (x[1]-Y3)^2")
    {
    }

    void impl_compute (result_ref result, const_argument_ref x) const
    {
        result[0] = pow(x[0]-X1, 2) + pow(x[1]-Y1, 2) + pow(x[0]-X2, 2) + pow(x[1]-Y2, 2) + pow(x[0]-X3, 2) + pow(x[1]-Y3, 2);
    }

    void impl_gradient (gradient_ref grad, const_argument_ref x, size_type) const
    {
        grad << 2*(x[0]-X1) + 2*(x[0]-X2) + 2*(x[0]-X3),
                2*(x[1]-Y1) + 2*(x[1]-Y2) + 2*(x[1]-Y3);
    }

    void impl_hessian (hessian_ref h, const_argument_ref x, size_type) const
    {
        h << 6.0, 0.0,
                0.0, 6.0;
    }
};

// constraint 1
struct G0 : public TwiceDifferentiableFunction
{
    G0 () : TwiceDifferentiableFunction (2, 1, "(x[0]-X1)^2 + (x[1]-Y1)^2")
    {
    }

    void impl_compute (result_ref result, const_argument_ref x) const
    {
        result[0] = pow(x[0]-X1, 2) + pow(x[1]-Y1, 2);
    }

    void impl_gradient (gradient_ref grad, const_argument_ref x, size_type) const
    {
        grad << 2*(x[0]-X1),
                2*(x[1]-Y1);
    }

    void impl_hessian (hessian_ref h, const_argument_ref x, size_type) const
    {
        h << 2.0, 0.0,
                0.0, 2.0;
    }
};

// constraint 2
struct G1 : public TwiceDifferentiableFunction
{
    G1 () : TwiceDifferentiableFunction (2, 1, "(x[0]-X2)^2 + (x[1]-Y2)^2")
    {
    }

    void impl_compute (result_ref result, const_argument_ref x) const
    {
        result[0] = pow(x[0]-X2, 2) + pow(x[1]-Y2, 2);
    }

    void impl_gradient (gradient_ref grad, const_argument_ref x, size_type) const
    {
        grad << 2*(x[0]-X2),
                2*(x[1]-Y2);
    }

    void impl_hessian (hessian_ref h, const_argument_ref x, size_type) const
    {
        h << 2.0, 0.0,
                0.0, 2.0;
    }
};

// constraint 3
struct G2 : public TwiceDifferentiableFunction
{
    G2 () : TwiceDifferentiableFunction (2, 1, "(x[0]-X3)^2 + (x[1]-Y3)^2")
    {
    }

    void impl_compute (result_ref result, const_argument_ref x) const
    {
        result[0] = pow(x[0]-X3, 2) + pow(x[1]-Y3, 2);
    }

    void impl_gradient (gradient_ref grad, const_argument_ref x, size_type) const
    {
        grad << 2*(x[0]-X3),
                2*(x[1]-Y3);
    }

    void impl_hessian (hessian_ref h, const_argument_ref x, size_type) const
    {
        h << 2.0, 0.0,
                0.0, 2.0;
    }
};

// Callback functions
void getDistanceCallback(std_msgs::Float64MultiArray distances)
{
    are_dist_available = true;
    blue_distance = distances.data[0];
    red_distance = distances.data[1];
    green_distance = distances.data[2];
}

int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "triangulation_optimization");

    // Define your node handles
    ros::NodeHandle nh, nh_loc("~");

    // Declare your subscribers
    ros::Subscriber dist_sub = nh.subscribe<std_msgs::Float64MultiArray>("beacon_distances", 1, getDistanceCallback);

    // Declare your publishers
    pose_pub = nh.advertise<geometry_msgs::Pose2D>("robot_pose", 1);

    // node initilization
    // ...

    // rate
    ros::Rate rate(30); // Hz
    while (ros::ok()){
        ros::spinOnce();

        // node code
        if (!are_dist_available) {
            ROS_INFO("Waiting for distances");
            continue;
        }

        // ------------ RobOptim ---------------
        typedef Solver<EigenMatrixDense> solver_t;

        // Create cost function.
        boost::shared_ptr<F> f (new F ());

        // Create problem.
        solver_t::problem_t pb (f);

        // Set bounds for all optimization parameters.
        // 1. < x_i < 5. (x_i in [1.;5.])
        for (Function::size_type i = 0; i < pb.function ().inputSize (); ++i)
            pb.argumentBounds ()[i] = Function::makeInterval (LB, UB);


        // Set the starting point.
        Function::vector_t start (pb.function ().inputSize ());
        start << 150.,100.;
        pb.startingPoint() = start;

        // Create constraints.
        boost::shared_ptr<G0> g0 (new G0 ());
        boost::shared_ptr<G1> g1 (new G1 ());
        boost::shared_ptr<G2> g2 (new G2 ());

        F::intervals_t bounds;
        solver_t::problem_t::scaling_t scaling;

        // Add constraints
        bounds.push_back(Function::makeLowerInterval( pow(blue_distance, 2) ));  // blue beacon distance
        scaling.push_back (1.);
        pb.addConstraint (g0, bounds, scaling);

        bounds.clear ();
        scaling.clear ();

        bounds.push_back(Function::makeLowerInterval( pow(red_distance, 2) ));   // red beacon distance
        scaling.push_back (1.);
        pb.addConstraint (g1, bounds, scaling);

        bounds.clear ();
        scaling.clear ();

        bounds.push_back(Function::makeLowerInterval( pow(green_distance, 2) )); // green beacon distance
        scaling.push_back (1.);
        pb.addConstraint (g2, bounds, scaling);

        bounds.clear ();
        scaling.clear ();

        // Initialize solver.

        // Here we are relying on the CFSQP solver.
        // You may change this string to load the solver you wish to use:
        //  - Ipopt: "ipopt", "ipopt-sparse", "ipopt-td"
        //  - Eigen: "eigen-levenberg-marquardt"
        //  etc.
        // The plugin is built for a given solver type, so choose it adequately.
        SolverFactory<solver_t> factory ("ipopt", pb);
        solver_t& solver = factory ();

        // Compute the minimum and retrieve the result.
        solver_t::result_t res = solver.minimum ();

        // Display solver information.
        // std::cout << solver << std::endl;

        // Check if the minimization has succeeded.

        // Process the result
        switch (res.which ())
        {
        case solver_t::SOLVER_VALUE:
        {
            // Get the result.
            Result& result = boost::get<Result> (res);

            // Display the result.
            std::cout << "A solution has been found: " << std::endl << result << std::endl;
            geometry_msgs::Pose2D pose_msg;
            pose_msg.x = result.x[0];
            pose_msg.y = result.x[1];
            pose_msg.theta = 0;
            pose_pub.publish(pose_msg);
            break;
        }

        case solver_t::SOLVER_VALUE_WARNINGS:
        {
            // Get the result.
            ResultWithWarnings& result = boost::get<ResultWithWarnings> (res);

            // Display the result.
            std::cout << "A solution w/warning has been found: " << std::endl
                      << result << std::endl;

            break;
        }

        case solver_t::SOLVER_NO_SOLUTION:
        case solver_t::SOLVER_ERROR:
        {
            std::cout << "A solution should have been found. Failing..."
                      << std::endl
                      << boost::get<SolverError> (res).what ()
                      << std::endl;

            break;
        }
        }

        are_dist_available = false;

        rate.sleep();
    }
}

