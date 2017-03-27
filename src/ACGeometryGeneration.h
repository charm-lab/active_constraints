//
// Created by nima on 08/03/17.
//

#ifndef ACTIVE_CONSTRAINTS_ACGEOMETRYGENERATION_H
#define ACTIVE_CONSTRAINTS_ACGEOMETRYGENERATION_H



#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/String.h>
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>
#include <geometry_msgs/PoseArray.h>

class ACGeometryGeneration {


public:
    ACGeometryGeneration(std::string node_name);

    // publishing the poses and twists in task space coordinate frame
    void PublishCurrentPose();

    void PublishDesiredPose();

    void PublishCurrentTwist();

public:

    double ros_freq;
    int n_arms;
    geometry_msgs::PoseArray ac_path;
    bool ac_path_received =0 ;
    ros::Time ac_path_time_stamp;

    KDL::Frame tool_pose_current[2];
    KDL::Frame tool_pose_desired[2];
    KDL::Twist tool_twist_current[2];

    //ros::Publisher publisher_ac_path;

    void ClosestPointToACPoints(const KDL::Vector tool_current_position,
                                const geometry_msgs::PoseArray & ac_path,
                                KDL::Vector & tool_desired_position);

    void GenerateXYCircle(const KDL::Vector center, const double radius, const int num_points,
                          geometry_msgs::PoseArray & ac_path);
private:

    void SetupROSCommunications();

    void Tool1PoseCurrentCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);
    void Tool2PoseCurrentCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);
    void Tool1TwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    void Tool2TwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    void ACPathCallback(const geometry_msgs::PoseArrayConstPtr & msg);


        // two function pointers for slave pose callbacks
    void (ACGeometryGeneration::*tool_pose_callbacks[2])
            (const geometry_msgs::PoseStamped::ConstPtr &msg);

    // two function pointers for slave twist callbacks
    void (ACGeometryGeneration::*tool_twist_callbacks[2])
            (const geometry_msgs::TwistStamped::ConstPtr &msg);



private:
    ros::NodeHandle n;
    ros::Subscriber *subscriber_tool_current_pose;
    ros::Subscriber *subscriber_tool_current_twist;

    ros::Publisher *publisher_tool_pose_current;
    ros::Publisher *publisher_tool_pose_desired;
    ros::Publisher *publisher_tool_twist_current;


    ros::Subscriber subscriber_foot_pedal_clutch;
    ros::Subscriber subscriber_ac_path;
    KDL::Frame RCM_to_task_space_tr[2];
    
};

namespace conversions{

    void VectorToKDLFrame(const std::vector<double> &in_vec, KDL::Frame &out_pose);
    void VectorToPoseMsg(const std::vector<double> in_vec,
                         geometry_msgs::Pose &out_pose);

};
// operator overload to print out vectors
std::ostream& operator<<(std::ostream& out, const std::vector<double>& vect);
std::ostream& operator<<(std::ostream& out, const KDL::Vector& vec);


#endif //ACTIVE_CONSTRAINTS_ACGEOMETRYGENERATION_H



