/*
 * ActiveConstraintsRos.hpp
 *
 *  Created on: Jan 23, 2017
 *      Author: nearlab
 */

#ifndef SRC_ACTIVECONSTRAINTSROS_HPP_
#define SRC_ACTIVECONSTRAINTSROS_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>

class ActiveConstraintsROS {

public:
	ActiveConstraintsROS(std::string node_name);
    void StartTeleop();
public:
	double ros_freq;
	double n_arms;
	geometry_msgs::PoseStamped slave_1_pose;
	geometry_msgs::TwistStamped slave_1_twist;
	geometry_msgs::PoseStamped slave_2_pose;
	geometry_msgs::TwistStamped slave_2_twist;
    std::string master_1_state;
    std::string master_2_state;
    bool coag_pressed;
    bool new_coag_event;

	ros::Publisher pub_master_1_wrench;
	ros::Publisher pub_master_2_wrench;


	ros::Publisher pub_wrench_body_orientation_absolute;
//    ros::Publisher pub_master_1_set_state;
//    ros::Publisher pub_master_2_set_state;

    ros::Publisher pub_dvrk_console_teleop_enable;
    ros::Publisher pub_dvrk_home;

private:

	void GetROSParameterValues();

	void Slave1PoseCurrentCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);
	void Slave1TwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

	void Slave2PoseCurrentCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);
	void Slave2TwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    void Master1StateCallback(const std_msgs::StringConstPtr &msg);
    void Master2StateCallback(const std_msgs::StringConstPtr &msg);

    void FootPedalCoagCallback(const sensor_msgs::Joy &msg);

private:
	ros::NodeHandle n;
	ros::Subscriber subscriber_slave_1_current_pose;
	ros::Subscriber subscriber_slave_1_twist;
    ros::Subscriber subscriber_master_1_state;

    ros::Subscriber subscriber_foot_pedal_clutch;

	ros::Subscriber subscriber_slave_2_current_pose;
	ros::Subscriber subscriber_slave_2_twist;
    ros::Subscriber subscriber_master_2_state;

};



#endif /* SRC_ACTIVECONSTRAINTSROS_HPP_ */
