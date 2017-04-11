/*
 * ActiveConstraintsRos.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: nearlab
 */


#include "ACEnforcement.hpp"
//  including the geometry generation for the conversion functions.
// will be fixed later.
#include "ACGeometryGeneration.h"


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
    new_desired_pose_msg[0] = false;
    new_desired_pose_msg[1] = false;

    SetupROSCommunications();
}



//-----------------------------------------------------------------------------------
// SetupROSCommunications
//-----------------------------------------------------------------------------------

void ACEnforcement::SetupROSCommunications() {

    // all the parameters have default values.

    // Loop frequency
    ROS_INFO("Node frequency depends on the desired pose topic.");

    n.param<int>("number_of_arms", n_arms, 1);
    ROS_INFO("Expecting '%d' arm(s)", n_arms);

    if(n_arms<1)
        ROS_ERROR("Number of arms must be at least 1.");

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

    for(int n_arm = 0; n_arm<n_arms; n_arm++) {

        //getting the name of the arms
        std::stringstream param_name;
        param_name << std::string("slave_") << n_arm + 1 << "_name";
        n.getParam(param_name.str(), slave_names[n_arm]);

        param_name.str("");
        param_name << std::string("master_") << n_arm + 1 << "_name";
        n.getParam(param_name.str(), master_names[n_arm]);


        // publishers
        param_name.str("");
        param_name << std::string("/dvrk/") << master_names[n_arm]
                   << "/set_wrench_body";
        publisher_wrench[n_arm] = n.advertise<geometry_msgs::Wrench>(
                param_name.str().c_str(), 1);
        ROS_INFO("Will publish on %s", param_name.str().c_str());


        param_name.str("");
        param_name << std::string("/dvrk/") << master_names[n_arm]
                   << "/set_wrench_body_orientation_absolute";
        publisher_wrench_body_orientation_absolute[n_arm] = n.advertise<std_msgs::Bool>(
                param_name.str().c_str(), 1);
        ROS_INFO("Will publish on %s", param_name.str().c_str());


        // subscribers
        param_name.str("");
        param_name << std::string("/") << slave_names[n_arm]
                   << "/tool_pose_desired";
        subscriber_tool_pose_desired[n_arm] = n.subscribe(param_name.str(), 1,
                                                          tool_pose_desired_callbacks[n_arm],
                                                          this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s", param_name.str().c_str());


        // the current pose of the tools (slaves)
        param_name.str("");
        param_name << std::string("/dvrk/") <<slave_names[n_arm] << "/position_cartesian_current";
        subscriber_tool_pose_current[n_arm] = n.subscribe(param_name.str(), 1,
                                                          tool_pose_current_callbacks[n_arm], this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s", param_name.str().c_str());
        // we will later check to see if something is publishing on the current slave pose


        param_name.str("");
        param_name << std::string("/dvrk/") <<slave_names[n_arm] << "/twist_body_current";
        subscriber_slaves_current_twist[n_arm] = n.subscribe(param_name.str(),
                                                             1, tool_twist_callback[n_arm], this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s", param_name.str().c_str());

        param_name.str("");
        param_name << std::string("/dvrk/") << master_names[n_arm]
                   << "/robot_state";
        subscriber_master_state[n_arm] = n.subscribe(param_name.str(), 1,
                                                     master_state_callback[n_arm],
                                                     this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s", param_name.str().c_str());


        // the transformation from the coordinate frame of the slave (RCM) to the task coordinate
        // frame.
        param_name.str("");
        param_name << (std::string) "/calibrations/task_frame_to_"
                   << slave_names[n_arm] << "_frame";
        std::vector<double> vect_temp = std::vector<double>(7, 0.0);
        if (n.getParam(param_name.str(), vect_temp)) {
            conversions::VectorToKDLFrame(vect_temp,
                                          slave_frame_to_task_frame[n_arm]);
            // param is from task to slave, we want the inverse
            slave_frame_to_task_frame[n_arm] = slave_frame_to_task_frame[n_arm].Inverse();
        } else
            ROS_ERROR("Parameter %s is needed.", param_name.str().c_str());


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
    subscriber_foot_pedal_coag = n.subscribe("/dvrk/footpedals/coag", 1,
                                               &ACEnforcement::FootPedalCoagCallback, this);
    ROS_INFO("[SUBSCRIBERS] Will subscribe to /dvrk/footpedals/coag");

    subscriber_foot_pedal_clutch = n.subscribe("/dvrk/footpedals/clutch", 1,
                                               &ACEnforcement::FootPedalClutchCallback, this);
    ROS_INFO("[SUBSCRIBERS] Will subscribe to /dvrk/footpedals/clutch");

    pub_dvrk_console_teleop_enable = n.advertise<std_msgs::Bool>("/dvrk/console/teleop/enable", 1);

    pub_dvrk_home = n.advertise<std_msgs::Empty>("/dvrk/console/home", 1);

    pub_dvrk_power_off = n.advertise<std_msgs::Empty>("/dvrk/console/power_off", 1);


}



void ACEnforcement::Tool1PoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // take the pose from the arm frame to the task frame
    KDL::Frame frame;
    tf::poseMsgToKDL(msg->pose, frame);
    tool_pose_current[0] =  slave_frame_to_task_frame[0] * frame;

}

void ACEnforcement::Tool2PoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // take the pose from the arm frame to the task frame
    KDL::Frame frame;
    tf::poseMsgToKDL(msg->pose, frame);
    tool_pose_current[1] =  slave_frame_to_task_frame[1] * frame;
}

void ACEnforcement::Tool1PoseDesiredCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    tf::poseMsgToKDL(msg->pose, tool_pose_desired[0]);
    new_desired_pose_msg[0] = true;
}

void ACEnforcement::Tool2PoseDesiredCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    tf::poseMsgToKDL(msg->pose, tool_pose_desired[1]);
    new_desired_pose_msg[1] = true;

}

void ACEnforcement::Tool1TwistCallback(
        const geometry_msgs::TwistStamped::ConstPtr &msg) {

    tf::twistMsgToKDL(msg->twist, tool_twist[0]);
    tool_twist[0] =  slave_frame_to_task_frame[0] * tool_twist[0];
}

void ACEnforcement::Tool2TwistCallback(
        const geometry_msgs::TwistStamped::ConstPtr &msg) {

    tf::twistMsgToKDL(msg->twist, tool_twist[1]);
    tool_twist[1] =  slave_frame_to_task_frame[1] * tool_twist[1];

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

void ACEnforcement::FootPedalClutchCallback(const sensor_msgs::Joy & msg){
    clutch_pressed = (bool)msg.buttons[0];
}

void ACEnforcement::PublishWrenchInSlaveFrame(const int num_arm, const KDL::Vector f,
                                              const KDL::Vector taw) {

    geometry_msgs::Wrench wrench_out;
    KDL::Vector f_out, taw_out;

    f_out =  slave_frame_to_task_frame[num_arm].M.Inverse() * f;
    taw_out =  slave_frame_to_task_frame[num_arm].M.Inverse() * taw;

    wrench_out.force.x = f_out[0];
    wrench_out.force.y = f_out[1];
    wrench_out.force.z = f_out[2];

    wrench_out.torque.x = taw_out[0];
    wrench_out.torque.y = taw_out[1];
    wrench_out.torque.z = taw_out[2];
    publisher_wrench[num_arm].publish(wrench_out);

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




void conversions::VectorToPoseMsg(const std::vector<double> in_vec,
                                  geometry_msgs::Pose &out_pose) {

    out_pose.position.x = in_vec.at(0);
    out_pose.position.y = in_vec.at(1);
    out_pose.position.z = in_vec.at(2);
    out_pose.orientation.x = in_vec.at(3);
    out_pose.orientation.y = in_vec.at(4);
    out_pose.orientation.z = in_vec.at(5);
    out_pose.orientation.w = in_vec.at(6);

}

void conversions::VectorToKDLFrame(const std::vector<double> &in_vec, KDL::Frame &out_pose) {
    geometry_msgs::Pose pose_msg;
    conversions::VectorToPoseMsg(in_vec, pose_msg);
    tf::poseMsgToKDL(pose_msg, out_pose);
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