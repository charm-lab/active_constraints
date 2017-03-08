/*
 * ActiveConstraintsRos.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: nearlab
 */


#include "ActiveConstraintsROS.hpp"


ActiveConstraintsROS::ActiveConstraintsROS(std::string node_name)
        : n(node_name), coag_pressed(false)
{
    GetROSParameterValues();
}



//-----------------------------------------------------------------------------------
// GetROSParameterValues
//-----------------------------------------------------------------------------------

void ActiveConstraintsROS::GetROSParameterValues() {

    bool all_required_params_found = true;

    // all the parameters have default values.

    // Loop frequency
    n.param<double>("node_frequency", ros_freq, 25);
    ROS_INFO("Node frequency will be set as '%f'", ros_freq);

    n.param<double>("number_of_arms", n_arms, 1);
    ROS_INFO("Expecting '%d' arm(s)", n_arms);

    // tool1 name
    std::string slave_1_name;
    n.param<std::string>("slave_1_name",	slave_1_name,"PSM2");
    std::string master_1_name;
    n.param<std::string>("master_1_name",	master_1_name,"MTMR");


    // tool1 pose current subscriber
    std::string slave_1_pose_current_topic_name =
            std::string("/dvrk/") + slave_1_name + "/position_cartesian_current";

    //check if something is being published on it
    if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
            slave_1_pose_current_topic_name, ros::Duration(1)))
        ROS_WARN("Topic '%s' is not publishing.", slave_1_pose_current_topic_name.c_str());
    else
        ROS_INFO("Reading tool 1 current pose messages from topic '%s'",
                 slave_1_pose_current_topic_name.c_str());

    subscriber_slave_1_current_pose = n.subscribe(slave_1_pose_current_topic_name, 1,
                                                 &ActiveConstraintsROS::Slave1PoseCurrentCallback, this);


    // right tool twist subscriber
    std::string slave_1_twist_current_topic_name=
            std::string("/dvrk/") + slave_1_name + "/twist_body_current";

    //check if something is being published on it
    if (!ros::topic::waitForMessage<geometry_msgs::TwistStamped>(
            slave_1_twist_current_topic_name, ros::Duration(1)))
        ROS_WARN("Topic '%s' is not publishing.", slave_1_twist_current_topic_name.c_str());
    else
        ROS_INFO("Reading tool 1 twist messages from topic '%s'", slave_1_twist_current_topic_name.c_str());

    // register MTMR pose subscriber
    subscriber_slave_1_twist = n.subscribe(slave_1_twist_current_topic_name, 1,
                                          &ActiveConstraintsROS::Slave1TwistCallback, this);

    // right tool wrench publisher setup
    std::string master_1_wrench_command_topic_name=
            std::string("/dvrk/") + master_1_name + "/set_wrench_body";

    pub_master_1_wrench = n.advertise<geometry_msgs::Wrench>(master_1_wrench_command_topic_name, 1, 0);
    ROS_INFO("Publishing master 1 wrench command messages to topic '%s'", master_1_wrench_command_topic_name.c_str());


    // right tool twist subscriber
    std::string master_1_state_current_topic_name=
            std::string("/dvrk/") + master_1_name + "/robot_state";
    ROS_INFO("Reading master 1 state messages from topic '%s'", master_1_state_current_topic_name.c_str());

    // register MTMR pose subscriber
    subscriber_master_1_state = n.subscribe(master_1_state_current_topic_name, 1,
                                          &ActiveConstraintsROS::Master1StateCallback, this);




//    // robot state publisher setup
//    std::string master_1_state_topic_name=
//            std::string("/dvrk/") + master_1_name + "/set_robot_state";

//    pub_master_1_set_state = n.advertise<std_msgs::String>(master_1_state_topic_name, 1, 0);
//    ROS_INFO("Publishing master 1 state command messages to topic '%s'", master_1_state_topic_name.c_str());



    // if we have two tools
    if(n_arms ==2){

        // tool1 name
        std::string slave_2_name;
        n.param<std::string>("slave_2_name",	slave_2_name,"MTML");
        std::string master_2_name;
        n.param<std::string>("master_2_name",	master_2_name,"PSM1");

        // tool1 pose current subscriber
        std::string slave_2_pose_current_topic_name =
                std::string("/dvrk/") + slave_2_name + "/position_cartesian_current";

        //check if something is being published on it
        if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
                slave_2_pose_current_topic_name, ros::Duration(1))) {
            ROS_WARN("Topic '%s' is not publishing.", slave_2_pose_current_topic_name.c_str());
            all_required_params_found = false;
        }
        else
            ROS_INFO("Reading tool 2 current pose messages from topic '%s'",
                     slave_2_pose_current_topic_name.c_str());

        subscriber_slave_2_current_pose = n.subscribe(slave_2_pose_current_topic_name, 1,
                                                      &ActiveConstraintsROS::Slave2PoseCurrentCallback, this);


        // right tool twist subscriber
        std::string slave_2_twist_current_topic_name=
                std::string("/dvrk/") + slave_2_name + "/twist_body_current";

        //check if something is being published on it
        if (!ros::topic::waitForMessage<geometry_msgs::TwistStamped>(
                slave_2_twist_current_topic_name, ros::Duration(1))) {
            ROS_WARN("Topic '%s' is not publishing.", slave_2_twist_current_topic_name.c_str());
        }
        else
            ROS_INFO("Reading tool 2 twist messages from topic '%s'", slave_2_twist_current_topic_name.c_str());

        // register MTMR pose subscriber
        subscriber_slave_2_twist = n.subscribe(slave_2_twist_current_topic_name, 1,
                                               &ActiveConstraintsROS::Slave2TwistCallback, this);


        // right tool wrench publisher setup
        std::string slave_2_wrnech_command_topic_name=
                std::string("/dvrk/") + slave_2_name + "/set_wrench_body";

        pub_master_2_wrench = n.advertise<geometry_msgs::Wrench>(
                slave_2_wrnech_command_topic_name,1, 0);
        ROS_INFO("Publishing tool 2 wrench command messages to topic '%s'", slave_2_wrnech_command_topic_name.c_str());

        // robot state subscriber setup
        std::string slave_2_state_topic_name=
                std::string("/dvrk/") + slave_2_name + "/set_robot_state";
        // register MTMR pose subscriber
        subscriber_master_2_state = n.subscribe(slave_2_state_topic_name, 1,
                                              &ActiveConstraintsROS::Master2StateCallback, this);

        // robot state publisher setup
        std::string master_2_state_topic_name=
                std::string("/dvrk/") + master_2_name + "/set_robot_state";

//        pub_master_2_set_state = n.advertise<std_msgs::String>(master_2_state_topic_name, 1, 0);
//        ROS_INFO("Publishing master 2 state command messages to topic '%s'", master_2_state_topic_name.c_str());




    }

    subscriber_foot_pedal_clutch = n.subscribe("/dvrk/footpedals/coag", 1,
                                               &ActiveConstraintsROS::FootPedalCoagCallback, this);

        // wrench_body_orientation_absolute publisher setup
    std::string wrench_body_orientation_absolute_topic_name =
            std::string("/dvrk/") + slave_1_name + "/set_wrench_body_orientation_absolute";

    pub_wrench_body_orientation_absolute = n.advertise<std_msgs::Bool>(
            wrench_body_orientation_absolute_topic_name, 1, 0);

    pub_dvrk_console_teleop_enable = n.advertise<std_msgs::Bool>("/dvrk/console/teleop/enable", 1);
    pub_dvrk_home = n.advertise<std_msgs::Empty>("/dvrk/console/home", 1);


}





void ActiveConstraintsROS::Slave1PoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {

    //    ROS_INFO_STREAM("chatter1: [" << msg->position << "] [thread=" << boost::this_thread::get_id() << "]");
    slave_1_pose.pose.position = msg->pose.position;
    slave_1_pose.pose.orientation = msg->pose.orientation;

}


void ActiveConstraintsROS::Slave1TwistCallback(
        const geometry_msgs::TwistStamped::ConstPtr &msg) {

    //    ROS_INFO_STREAM("chatter1: [" << msg->position << "] [thread=" << boost::this_thread::get_id() << "]");
    slave_1_twist.twist.linear = msg->twist.linear;
    slave_1_twist.twist.angular= msg->twist.angular;

}



void ActiveConstraintsROS::Slave2PoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {

    //    ROS_INFO_STREAM("chatter1: [" << msg->position << "] [thread=" << boost::this_thread::get_id() << "]");
    slave_2_pose.pose.position = msg->pose.position;
    slave_2_pose.pose.orientation = msg->pose.orientation;

}


void ActiveConstraintsROS::Slave2TwistCallback(
        const geometry_msgs::TwistStamped::ConstPtr &msg) {

    //    ROS_INFO_STREAM("chatter1: [" << msg->position << "] [thread=" << boost::this_thread::get_id() << "]");
    slave_2_twist.twist.linear = msg->twist.linear;
    slave_2_twist.twist.angular= msg->twist.angular;

}

void ActiveConstraintsROS::Master1StateCallback(
        const std_msgs::StringConstPtr &msg) {
    master_1_state  = msg->data;
}

void ActiveConstraintsROS::Master2StateCallback(
        const std_msgs::StringConstPtr &msg) {
    master_2_state  = msg->data;
}

void ActiveConstraintsROS::FootPedalCoagCallback(const sensor_msgs::Joy & msg){

    coag_pressed = (bool)msg.buttons[0];
    new_coag_event = true;

}

void ActiveConstraintsROS::StartTeleop() {
    // first send the arms to home
    std_msgs::Empty empty;
    pub_dvrk_home.publish(empty);
    ROS_INFO("Setting dvrk console state to Home.");
    // wait till the arms are ready
    ros::Rate loop(5);
    while(master_1_state!= "DVRK_READY"){

        if(n_arms>1){
            while(master_2_state!= "DVRK_READY"){
                loop.sleep();
                ros::spinOnce();
            }
        }
        loop.sleep();
        ros::spinOnce();
    }
    // enable teleop

    ROS_INFO("Starting teleoperation.");
    std_msgs::Bool teleop_bool;
    teleop_bool.data = 1;
    pub_dvrk_console_teleop_enable.publish(teleop_bool);

}
