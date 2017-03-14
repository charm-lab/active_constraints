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
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>
class ActiveConstraintsROS {

public:
	ActiveConstraintsROS(std::string node_name);
    void StartTeleop();
public:
	double ros_freq;
	int n_arms;
    KDL::Frame tool_pose_current[2];
    KDL::Frame tool_pose_desired[2];
    KDL::Twist slave_1_twist;
    KDL::Twist slave_2_twist;

    std::string master_1_state;
    std::string master_2_state;
    bool coag_pressed;
    bool new_coag_event;

//    ros::Publisher pub_master_1_set_state;
//    ros::Publisher pub_master_2_set_state;

    ros::Publisher pub_dvrk_console_teleop_enable;
    ros::Publisher pub_dvrk_home;
    ros::Publisher pub_dvrk_power_off;
	ros::Publisher *publisher_wrench;
    ros::Publisher *publisher_wrench_body_orientation_absolute;


private:

	void SetupROSCommunications();

	void Tool1PoseCurrentCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);

	void Tool2PoseCurrentCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);

    void Tool1PoseDesiredCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);

	void Tool2PoseDesiredCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);


    void Tool1TwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

	void Tool2TwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    void Master1StateCallback(const std_msgs::StringConstPtr &msg);

    void Master2StateCallback(const std_msgs::StringConstPtr &msg);

    void FootPedalCoagCallback(const sensor_msgs::Joy &msg);


    // two function pointers for slave pose current callbacks
	void (ActiveConstraintsROS::*tool_pose_current_callbacks[2])
			(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // two function pointers for slave pose desired callbacks
    void (ActiveConstraintsROS::*tool_pose_desired_callbacks[2])
            (const geometry_msgs::PoseStamped::ConstPtr &msg);

	// two function pointers for slave twist callbacks
	void (ActiveConstraintsROS::*tool_twist_callback[2])
			(const geometry_msgs::TwistStamped::ConstPtr &msg);

    // two function pointers for master state callbacks
    void (ActiveConstraintsROS::*master_state_callback[2])
            (const std_msgs::StringConstPtr &msg);

private:
	ros::NodeHandle n;
    ros::Subscriber *subscriber_tool_pose_current;
    ros::Subscriber *subscriber_tool_pose_desired;
	ros::Subscriber *subscriber_slaves_current_twist;

    ros::Subscriber *subscriber_master_state;

    ros::Subscriber subscriber_foot_pedal_clutch;


};

// operator overload to print out vectors
std::ostream& operator<<(std::ostream& out, const std::vector<double>& vect);
std::ostream& operator<<(std::ostream& out, const KDL::Vector& vec);



#endif /* SRC_ACTIVECONSTRAINTSROS_HPP_ */
