//
// Created by nima on 08/03/17.
//

#include "ACGeometryGeneration.h"



ACGeometryGeneration::ACGeometryGeneration(std::string node_name)
        : n(node_name)
{


    // assign the callback functions
    tool_pose_callbacks[0] = &ACGeometryGeneration::Tool1PoseCurrentCallback;
    tool_pose_callbacks[1] = &ACGeometryGeneration::Tool2PoseCurrentCallback;
    tool_twist_callbacks[0] = &ACGeometryGeneration::Tool1TwistCallback;
    tool_twist_callbacks[1] = &ACGeometryGeneration::Tool2TwistCallback;

    SetupROSCommunications();

}



//-----------------------------------------------------------------------------------
// SetupROSCommunications
//-----------------------------------------------------------------------------------

void ACGeometryGeneration::SetupROSCommunications() {


    // all the parameters have default values.

    // Loop frequency
    n.param<double>("node_frequency", ros_freq, 50);
    ROS_INFO("Node frequency will be set as '%f'", ros_freq);

    n.param<int>("number_of_arms", n_arms, 1);
    ROS_INFO("Expecting '%d' arm(s)", n_arms);

    // Subscribers and publishers
    publisher_tool_pose_current = new ros::Publisher[(uint)n_arms];
    publisher_tool_twist_current = new ros::Publisher[(uint)n_arms];
    publisher_tool_pose_desired = new ros::Publisher[(uint)n_arms];
    subscriber_tool_current_pose = new ros::Subscriber[(uint)n_arms];
    subscriber_tool_current_twist = new ros::Subscriber[(uint)n_arms];

    // tool1 name
    std::string slave_names[n_arms];
    std::string master_names[n_arms];
    std::string check_topic_name;
    for(int n_arm = 0; n_arm<n_arms; n_arm++){

        //getting the name of the arms
        std::stringstream param_name;
        param_name << std::string("slave_") << n_arm + 1 << "_name";
        n.getParam(param_name.str(), slave_names[n_arm]);


        // publishers
        param_name.str("");
        param_name << std::string("/") << slave_names[n_arm] << "/tool_pose_current";
        publisher_tool_pose_current[n_arm] = n.advertise<geometry_msgs::PoseStamped>(
                param_name.str().c_str(), 1 );
        ROS_INFO("Will publish on %s",
                 param_name.str().c_str());


        param_name.str("");
        param_name << std::string("/")<< slave_names[n_arm] << "/tool_pose_desired";
        publisher_tool_pose_desired[n_arm] = n.advertise<geometry_msgs::PoseStamped>(
                param_name.str().c_str(), 1 );
        ROS_INFO("Will publish on %s", param_name.str().c_str());


        param_name.str("");
        param_name << std::string("/") << slave_names[n_arm] << "/tool_twist_current";
        publisher_tool_twist_current[n_arm] = n.advertise<geometry_msgs::TwistStamped>(
                param_name.str().c_str(), 1 );
        ROS_INFO("Will publish on %s", param_name.str().c_str());

        // subscribers
        param_name.str("");
        param_name << std::string("/dvrk/") <<slave_names[n_arm] << "/position_cartesian_current";
        subscriber_tool_current_pose[n_arm] = n.subscribe(param_name.str(), 1,
                                                            tool_pose_callbacks[n_arm], this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s",
                 param_name.str().c_str());
        // we will later check to see if something is publishing on the current slave pose
        check_topic_name = param_name.str();

        param_name.str("");
        param_name << std::string("/dvrk/") <<slave_names[n_arm] << "/twist_body_current";
        subscriber_tool_current_twist[n_arm] = n.subscribe(param_name.str(), 1,
                                                            tool_twist_callbacks[n_arm], this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s", param_name.str().c_str());


        // other parameters

        // the transformation from the coordinate frame of the slave (RCM) to the task coordinate
        // frame.
        param_name.str("");
        param_name << (std::string)"/calibrations/task_frame_to_" << slave_names[n_arm] << "_frame";
        std::vector<double> vect_temp = std::vector<double>(7, 0.0);
        if(n.getParam(param_name.str(), vect_temp)){
            conversions::VectorToKDLFrame(vect_temp, slave_frame_to_task_frame[n_arm]);
            // param is from task to RCM, we want the inverse
            slave_frame_to_task_frame[n_arm] = slave_frame_to_task_frame[n_arm].Inverse();
        }
        else
            ROS_ERROR("Parameter %s is needed.", param_name.str().c_str());


    }

    //ac geom

//    std::string topic_name = "/ac_path";
//    publisher_ac_path = n.advertise<geometry_msgs::PoseArray>(
//            topic_name, 1 );
//    ROS_INFO("Will publish on %s",
//             topic_name.c_str());

    std::string topic_name = "/ac_path";
    subscriber_ac_path = n.subscribe("/ac_path", 1, &ACGeometryGeneration::ACPathCallback, this);
    ROS_INFO("[SUBSCRIBERS] Will Subscribe to %s",
             topic_name.c_str());


    // wait here till data is available.
    // the dvrk published zero values during the homing, so just checking the availability of the
    // messages is not enough.
    ros::Rate half_second_sleep(1);
    while(tool_pose_current[0].p.Norm() == 0.0 ){
        ROS_INFO("Got zero data on %s.",

                 check_topic_name.c_str());
        half_second_sleep.sleep();
        ros::spinOnce();
    }

}





void ACGeometryGeneration::Tool1PoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // take the pose from the arm frame to the task frame
    KDL::Frame frame;
    tf::poseMsgToKDL(msg->pose, frame);
    tool_pose_current[0] =  slave_frame_to_task_frame[0] * frame;

}

void ACGeometryGeneration::Tool2PoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // take the pose from the arm frame to the task frame
    KDL::Frame frame;
    tf::poseMsgToKDL(msg->pose, frame);
    tool_pose_current[1] =  slave_frame_to_task_frame[1] * frame;
}

void ACGeometryGeneration::Tool1TwistCallback(
        const geometry_msgs::TwistStamped::ConstPtr &msg) {
        // take the twist from the arm frame to the task frame
    tf::twistMsgToKDL(msg->twist, tool_twist_current[0]);
    tool_twist_current[0] = slave_frame_to_task_frame[0] * tool_twist_current[0];
}

void ACGeometryGeneration::Tool2TwistCallback(
        const geometry_msgs::TwistStamped::ConstPtr &msg) {
    // take the twist from the arm frame to the task frame
    tf::twistMsgToKDL(msg->twist, tool_twist_current[1]);
    tool_twist_current[1] = slave_frame_to_task_frame[1] * tool_twist_current[1];

}

void ACGeometryGeneration::PublishCurrentPose() {

    for (int n_arm = 0; n_arm < n_arms; ++n_arm) {

        geometry_msgs::PoseStamped pose_msg;
        tf::poseKDLToMsg(tool_pose_current[n_arm], pose_msg.pose);
        publisher_tool_pose_current[n_arm].publish(pose_msg);
    }
}

void ACGeometryGeneration::PublishDesiredPose() {

    for (int n_arm = 0; n_arm < n_arms; ++n_arm) {

        geometry_msgs::PoseStamped pose_msg;
        tf::poseKDLToMsg(tool_pose_desired[n_arm], pose_msg.pose);
        publisher_tool_pose_desired[n_arm].publish(pose_msg);
    }
}

void ACGeometryGeneration::ACPathCallback(const geometry_msgs::PoseArrayConstPtr & msg){

    ac_path.poses = msg->poses;
    ac_path_time_stamp = msg->header.stamp;

    ac_path_received = true;
}

void ACGeometryGeneration::GenerateXYCircle(const KDL::Vector center, const double radius, const int num_points,
                     geometry_msgs::PoseArray & ac_path){
    geometry_msgs::Pose point;
    for (int n_point = 0; n_point <num_points ; ++n_point) {

        double angle = 2* M_PI*(double)n_point / (double)(num_points);
        point.position.z = center[2];
        point.position.x = center[0] + radius * cos(angle);
        point.position.y = center[1] + radius * sin(angle);

        ac_path.poses.push_back(point);
    }


}

void ACGeometryGeneration::ClosestPointToACPoints(const KDL::Vector tool_current_position,
                                                  const geometry_msgs::PoseArray & ac_path,
                                                  KDL::Vector & tool_desired_position){

    double min_d = 100000; // something large
    size_t i_min = 0;
    //	cout << ac_points.size() << endl;
    ;
    for(size_t i=0; i<ac_path.poses.size(); i++){
        double dx = tool_current_position[0] - ac_path.poses[i].position.x;
        double dy = tool_current_position[1] - ac_path.poses[i].position.y;
        double dz = tool_current_position[2] - ac_path.poses[i].position.z;
        double norm2 = dx*dx + dy*dy + dz*dz;
        if(norm2 < min_d){
            min_d = norm2;
            i_min = i;
        }

    }
    tool_desired_position[0] = ac_path.poses[i_min].position.x;
    tool_desired_position[1] = ac_path.poses[i_min].position.y;
    tool_desired_position[2] = ac_path.poses[i_min].position.z;


}


void ACGeometryGeneration::PublishCurrentTwist() {

    for (int n_arm = 0; n_arm < n_arms; ++n_arm) {

        geometry_msgs::TwistStamped twist_msg;
        tf::twistKDLToMsg(tool_twist_current[n_arm], twist_msg.twist);
        twist_msg.header.stamp = ros::Time::now();
        twist_msg.header.frame_id = "/task_space";
        publisher_tool_twist_current[n_arm].publish(twist_msg);
    }
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