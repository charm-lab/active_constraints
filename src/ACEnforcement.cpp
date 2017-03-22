/*
 * ActiveConstraintsRos.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: nearlab
 */


#include "ACEnforcement.hpp"


ACEnforcement::ACEnforcement(std::string node_name)
        : n(node_name), coag_pressed(false)
{

    // assign the callback functions
    tool_pose_current_callbacks[0] = &ACEnforcement::Tool1PoseCurrentCallback;
    tool_pose_current_callbacks[1] = &ACEnforcement::Tool2PoseCurrentCallback;
    tool_pose_desired_callbacks[0] = &ACEnforcement::Tool1PoseDesiredCallback;
    tool_pose_desired_callbacks[1] = &ACEnforcement::Tool2PoseDesiredCallback;

    master_state_callback[0] = &ACEnforcement::Master1StateCallback;
    master_state_callback[1] = &ACEnforcement::Master2StateCallback;

    tool_twist_callback[0] = &ACEnforcement::Tool1TwistCallback;
    tool_twist_callback[1] = &ACEnforcement::Tool2TwistCallback;
    SetupROSCommunications();
}



//-----------------------------------------------------------------------------------
// SetupROSCommunications
//-----------------------------------------------------------------------------------

void ACEnforcement::SetupROSCommunications() {

    // all the parameters have default values.

    // Loop frequency
    n.param<double>("node_frequency", ros_freq, 50);
    ROS_INFO("%s: Node frequency will be set as '%f'", ros::this_node::getName().c_str(), ros_freq);

    n.param<int>("number_of_arms", n_arms, 1);
    ROS_INFO("%s: Expecting '%d' arm(s)", ros::this_node::getName().c_str(), n_arms);

    if(n_arms<1)
        ROS_ERROR("%s: Number of arms must be at least 1.", ros::this_node::getName().c_str());

    // Subscribers and publishers
    publisher_wrench = new ros::Publisher[n_arms];
    publisher_wrench_body_orientation_absolute = new ros::Publisher[n_arms];
    subscriber_tool_pose_current = new ros::Subscriber[n_arms];
    subscriber_tool_pose_desired = new ros::Subscriber[n_arms];
    subscriber_slaves_current_twist = new ros::Subscriber[n_arms];
    subscriber_master_state = new ros::Subscriber[n_arms];

    // tool1 name
    std::string slave_names[n_arms];
    std::string master_names[n_arms];

    for(int n_arm = 0; n_arm<n_arms; n_arm++){

        //getting the name of the arms
        std::stringstream param_name;
        param_name << std::string("slave_") << n_arm + 1 << "_name";
        n.getParam(param_name.str(), slave_names[n_arm]);

        param_name.str("");
        param_name << std::string("master_") << n_arm + 1 << "_name";
        n.getParam(param_name.str(), master_names[n_arm]);


        // publishers
        param_name.str("");
        param_name << std::string("/dvrk/")<< master_names[n_arm] << "/set_wrench_body";
        publisher_wrench[n_arm] = n.advertise<geometry_msgs::Wrench>(
                param_name.str().c_str(), 1 );
        ROS_INFO("%s: Will publish on %s",  ros::this_node::getName().c_str(),
                 param_name.str().c_str());


        param_name.str("");
        param_name << std::string("/dvrk/")<< master_names[n_arm] << "/set_wrench_body_orientation_absolute";
        publisher_wrench_body_orientation_absolute[n_arm] = n.advertise<std_msgs::Bool>(
                param_name.str().c_str(), 1 );
        ROS_INFO("%s: Will publish on %s",  ros::this_node::getName().c_str(),
                 param_name.str().c_str());


        // subscribers
        param_name.str("");
        param_name << std::string("/")<< slave_names[n_arm] << "/tool_pose_desired";
        subscriber_tool_pose_desired[n_arm] = n.subscribe(param_name.str(), 1,
                                                            tool_pose_desired_callbacks[n_arm], this);
        ROS_INFO("%s: Will subscribe to %s",  ros::this_node::getName().c_str(),
                 param_name.str().c_str());

        param_name.str("");
        param_name << std::string("/")<< slave_names[n_arm] << "/tool_pose_current";
        subscriber_tool_pose_current[n_arm] = n.subscribe(param_name.str(), 1,
                                                            tool_pose_current_callbacks[n_arm], this);
        ROS_INFO("%s: Will subscribe to %s",  ros::this_node::getName().c_str(),
                 param_name.str().c_str());


        param_name.str("");
        param_name << std::string("/") << slave_names[n_arm] << "/tool_twist_current";
        subscriber_slaves_current_twist[n_arm] = n.subscribe(param_name.str(), 1,
                                                             tool_twist_callback[n_arm], this);
        ROS_INFO("%s: Will subscribe to %s",  ros::this_node::getName().c_str(),
                 param_name.str().c_str());

        param_name.str("");
        param_name << std::string("/dvrk/") << master_names[n_arm] << "/robot_state";
        subscriber_master_state[n_arm] = n.subscribe(param_name.str(), 1,
                                                     master_state_callback[n_arm], this);
        ROS_INFO("%s: Will subscribe to %s",  ros::this_node::getName().c_str(),
                 param_name.str().c_str());
    }





//    // right tool twist subscriber
//    std::string master_1_state_current_topic_name=
//            std::string("/dvrk/") + master_1_name + "/robot_state";
//    ROS_INFO("Reading master 1 state messages from topic '%s'", master_1_state_current_topic_name.c_str());
//
//    // register MTMR pose subscriber
//    subscriber_master_1_state = n.subscribe(master_1_state_current_topic_name, 1,
//                                          &ACEnforcement::Master1StateCallback, this);



    // common subscriber and publishers
    subscriber_foot_pedal_clutch = n.subscribe("/dvrk/footpedals/coag", 1,
                                               &ACEnforcement::FootPedalCoagCallback, this);
    ROS_INFO("%s: Will subscribe to /dvrk/footpedals/coag",  ros::this_node::getName().c_str());

    pub_dvrk_console_teleop_enable = n.advertise<std_msgs::Bool>("/dvrk/console/teleop/enable", 1);

    pub_dvrk_home = n.advertise<std_msgs::Empty>("/dvrk/console/home", 1);

    pub_dvrk_power_off = n.advertise<std_msgs::Empty>("/dvrk/console/power_off", 1);


}


void ACEnforcement::Tool1PoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {

    tf::poseMsgToKDL(msg->pose, tool_pose_current[0]);
}

void ACEnforcement::Tool2PoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {

    tf::poseMsgToKDL(msg->pose, tool_pose_current[1]);
}


void ACEnforcement::Tool1PoseDesiredCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    tf::poseMsgToKDL(msg->pose, tool_pose_desired[0]);
}

void ACEnforcement::Tool2PoseDesiredCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    tf::poseMsgToKDL(msg->pose, tool_pose_desired[1]);

}

void ACEnforcement::Tool1TwistCallback(
        const geometry_msgs::TwistStamped::ConstPtr &msg) {

    tf::twistMsgToKDL(msg->twist, tool_twist[0]);

}

void ACEnforcement::Tool2TwistCallback(
        const geometry_msgs::TwistStamped::ConstPtr &msg) {

    tf::twistMsgToKDL(msg->twist, tool_twist[1]);
}

void ACEnforcement::Master1StateCallback(
        const std_msgs::StringConstPtr &msg) {
    master_1_state  = msg->data;
}

void ACEnforcement::Master2StateCallback(
        const std_msgs::StringConstPtr &msg) {
    master_2_state  = msg->data;
}

void ACEnforcement::FootPedalCoagCallback(const sensor_msgs::Joy & msg){

    coag_pressed = (bool)msg.buttons[0];
    new_coag_event = true;

}

void ACEnforcement::StartTeleop() {
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

    std_msgs::Bool teleop_bool;
    teleop_bool.data = 1;
    pub_dvrk_console_teleop_enable.publish(teleop_bool);
    ROS_INFO("Starting teleoperation.");

}



// operator overload to print out vectors
std::ostream& operator<<(std::ostream& out, const std::vector<double>& vect){
    for (unsigned int iter = 0; iter < vect.size(); ++iter) {
        out << "[" << iter <<"]: "<<vect.at(iter) << "\t";
    }

    return out;
}


// operator overload to print out vectors
std::ostream& operator<<(std::ostream& out, const KDL::Vector& vec){

    for (unsigned int iter = 0; iter < 3; ++iter) {
        out << "[" << iter <<"]: "<<vec[iter] << "\t";
    }

    return out;
}