//
// Created by nima on 08/03/17.
//


#include <iostream>
#include "ActiveConstraintEnforcement.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "ACGeometryGeneration.h"



int main(int argc, char *argv[]) {


    ros::init(argc, argv, "dvrk_ac");
    ACGeometryGeneration r(ros::this_node::getName());

    ros::Rate loop_rate(r.ros_freq);

    ros::Rate one_second_sleep(1);
    one_second_sleep.sleep();

    geometry_msgs::PoseArray ac_path;
    r.GenerateXYCircle(KDL::Vector(0.0, 0.0, 0.0), 0.1, 50, ac_path);

    r.publisher_ac_path.publish(ac_path);

    bool first_run = true;

    while(ros::ok()){

//        if(first_run){
//            r.tool_pose_desired[0] =  r.tool_pose_current[0];
//            ROS_INFO_STREAM(std::string("p_desired[0] = ") << r.tool_pose_current[0].p);
//            first_run = false;
//        }

        r.ClosestPointToACPoints(r.tool_pose_current->p, ac_path, r.tool_pose_desired->p);

        r.PublishCurrentPose();

        r.PublishDesiredPose();

        r.PublishCurrentTwist();

        loop_rate.sleep();
        ros::spinOnce();

        //		}

    }

    ROS_INFO("Ending Session...\n");
    ros::shutdown();

    return 0;
}
