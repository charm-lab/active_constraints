/*
 * ActiveConstraintsRos.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: nima
 */


#include "ACEnforcement.hpp"
//  including the geometry generation for the conversion functions.
// will be fixed later.
#include "ACGeometryGeneration.h"

ACEnforcement::ACEnforcement(std::string node_name)
        : n(node_name), coag_pressed(false)
{

    // assign the callback functions
    slave_pose_current_callbacks[0] = &ACEnforcement::Slave0PoseCurrentCallback;
    slave_pose_current_callbacks[1] = &ACEnforcement::Slave1PoseCurrentCallback;
    master_pose_current_callbacks[0] = &ACEnforcement::Master0PoseCurrentCallback;
    master_pose_current_callbacks[1] = &ACEnforcement::Master1PoseCurrentCallback;
    tool_pose_desired_callbacks[0] = &ACEnforcement::Tool0PoseDesiredCallback;
    tool_pose_desired_callbacks[1] = &ACEnforcement::Tool1PoseDesiredCallback;

    master_state_callback[0] = &ACEnforcement::Master0StateCallback;
    master_state_callback[1] = &ACEnforcement::Master1StateCallback;

    ac_params_callback[0] = &ACEnforcement::ACParams0Callback;
    ac_params_callback[1] = &ACEnforcement::ACParams1Callback;

    master_twist_callback[0] = &ACEnforcement::Master0TwistCallback;
    master_twist_callback[1] = &ACEnforcement::Master1TwistCallback;

    new_desired_pose_msg[0] = false;
    new_desired_pose_msg[1] = false;
    SetupROSCommunications();
    posbuf = new double *[3];
    for(int i = 0; i < 3; i++)
        posbuf[i] = new double[foaw_n];

    memset(posbuf[0], 0, sizeof(float)*foaw_n);
    memset(posbuf[1], 0, sizeof(float)*foaw_n);
    memset(posbuf[2], 0, sizeof(float)*foaw_n);

}



//-----------------------------------------------------------------------------------
// SetupROSCommunications
//-----------------------------------------------------------------------------------

void ACEnforcement::SetupROSCommunications() {

    // all the parameters have default values.

    // Loop frequency
    ROS_INFO("Node frequency depends on the desired pose topic.");
    n.param<double>("sampling_frequency", filtering_frequency, 100);
    ROS_INFO("For Filtering purposes, sampling rate is set as: %f.", filtering_frequency);

    n.param<int>("number_of_arms", n_arms, 1);
    ROS_INFO("Expecting '%d' arm(s)", n_arms);

    if(n_arms<1)
        ROS_ERROR("Number of arms must be at least 1.");

    // Subscribers and publishers
    publisher_wrench = new ros::Publisher[n_arms];
    publisher_wrench_body_orientation_absolute = new ros::Publisher[n_arms];
    subscriber_slave_pose_current = new ros::Subscriber[n_arms];
    subscriber_master_pose_current = new ros::Subscriber[n_arms];
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


        // the current pose of the slaves
        param_name.str("");
        param_name << std::string("/dvrk/") <<slave_names[n_arm] << "/position_cartesian_current";
        subscriber_slave_pose_current[n_arm] = n.subscribe(param_name.str(), 1,
                                                          slave_pose_current_callbacks[n_arm], this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s", param_name.str().c_str());
        // we will later check to see if something is publishing on the current slave pose



        // the current pose of the masters
        param_name.str("");
        param_name << std::string("/dvrk/") <<master_names[n_arm] << "/position_cartesian_current";
        subscriber_master_pose_current[n_arm] = n.subscribe(param_name.str(), 1,
                                                          master_pose_current_callbacks[n_arm], this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s", param_name.str().c_str());


        // NOTE we are using the master's velocity
        param_name.str("");
        param_name << std::string("/dvrk/") <<master_names[n_arm] << "/twist_body_current";
        subscriber_slaves_current_twist[n_arm] = n.subscribe(param_name.str(),
                                                             1, master_twist_callback[n_arm], this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s", param_name.str().c_str());

        param_name.str("");
        param_name << std::string("/dvrk/") << master_names[n_arm]
                   << "/robot_state";
        subscriber_master_state[n_arm] = n.subscribe(param_name.str(), 1,
                                                     master_state_callback[n_arm],
                                                     this);
        ROS_INFO("[SUBSCRIBERS] Will subscribe to %s", param_name.str().c_str());


        param_name.str("");
        param_name << std::string("/") << master_names[n_arm]
                   << "/active_constraint_param";
        subscriber_master_state[n_arm] = n.subscribe(param_name.str(), 1,
                                                     ac_params_callback[n_arm],
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

    ////////////////////////////
    double k_coeff, f_max, b_coeff, taw_max, kappa_coeff,c_coeff;

    n.param<double>("max_force", f_max, 4.0);
    ROS_INFO("max_force:  '%f'", f_max);

    n.param<double>("max_torque", taw_max, 0.02);
    ROS_INFO("taw_max:  '%f'", taw_max);

    n.param<double>("linear_elastic_coeff", k_coeff, 400);
    ROS_INFO("linear_elastic_coeff:  '%f'", k_coeff);

    n.param<double>("linear_damping_coeff", b_coeff, 2);
    ROS_INFO("linear_damping_coeff:  '%f'", b_coeff);

    n.param<double>("angular_elastic_coeff", kappa_coeff, 0.01);
    ROS_INFO("angular_elastic_coeff:  '%f'", kappa_coeff);

    n.param<double>("angular_damping_coeff", c_coeff, 0.001);
    ROS_INFO("angular_damping_coeff:  '%f'", c_coeff);


    // params will be overwritten if new message arrives
    for (int j = 0; j < 2; ++j) {
        ac_elastic[j] = new acElastic( f_max,  taw_max, k_coeff,  b_coeff, kappa_coeff, c_coeff);
        // will add params for the other constraints soon ...
        ac_plast_redirect[j] = new acPlastRedirect(f_max, 0.005, 0.002);
        ac_visc_redirect[j] = new acViscousRedirect(f_max, 70, 0.002);
    }

//    // right tool twist subscriber
//    std::string master_1_state_current_topic_name=
//            std::string("/dvrk/") + master_1_name + "/robot_state";
//    ROS_INFO("Reading master 1 state messages from topic '%s'", master_1_state_current_topic_name.c_str());
//
//    // register MTMR pose subscriber
//    subscriber_master_1_state = n.subscribe(master_1_state_current_topic_name, 1,
//                                          &ACEnforcement::Master0StateCallback, this);



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


    pub_twist = n.advertise<geometry_msgs::Twist>("/twist", 1);

}





void ACEnforcement::Slave0PoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // take the pose from the arm frame to the task frame
    KDL::Frame frame;
    tf::poseMsgToKDL(msg->pose, frame);
    slave_pose_current[0] =  slave_frame_to_task_frame[0] * frame;

}

void ACEnforcement::Slave1PoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // take the pose from the arm frame to the task frame
    KDL::Frame frame;
    tf::poseMsgToKDL(msg->pose, frame);
    slave_pose_current[1] =  slave_frame_to_task_frame[1] * frame;
}


void ACEnforcement::Master0PoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // take the pose from the arm frame to the task frame
    KDL::Frame frame;
    tf::poseMsgToKDL(msg->pose, frame);
    master_pose_current[0] =  slave_frame_to_task_frame[0] * frame;

    double r,p,y;
    master_pose_current[0].M.GetRPY(r,p,y);

    master_twist_filt[0].vel[0] = do_foaw_sample(posbuf[0], foaw_n, &foaw_i[0],
                                                 master_pose_current[0].p[0], 1,
                                                 0.001);
    master_twist_filt[0].vel[1] = do_foaw_sample(posbuf[1], foaw_n, &foaw_i[1],
                                                 master_pose_current[0].p[1], 1,
                                                 0.001);
    master_twist_filt[0].vel[2] = do_foaw_sample(posbuf[2], foaw_n, &foaw_i[2],
                                                 master_pose_current[0].p[2], 1,
                                                 0.001);

    // just averaging the angular velocity
    master_twist_filt[0].rot -= master_twist_filt[0].rot/10;
    master_twist_filt[0].rot += master_twist_dvrk[0].rot/10;



    geometry_msgs::Twist twist_foaw_msg;
    twist_foaw_msg.linear.x = master_twist_filt[0].vel[0];
    twist_foaw_msg.linear.y = master_twist_filt[0].vel[1];
    twist_foaw_msg.linear.z = master_twist_filt[0].vel[2];
    twist_foaw_msg.angular.x = master_twist_filt[0].rot[0];
    twist_foaw_msg.angular.y = master_twist_filt[0].rot[1];
    twist_foaw_msg.angular.z = master_twist_filt[0].rot[2];
    pub_twist.publish(twist_foaw_msg);

}

void ACEnforcement::Master1PoseCurrentCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // take the pose from the arm frame to the task frame
    KDL::Frame frame;
    tf::poseMsgToKDL(msg->pose, frame);
    master_pose_current[1] =  slave_frame_to_task_frame[1] * frame;

}

void ACEnforcement::Tool0PoseDesiredCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    tf::poseMsgToKDL(msg->pose, tool_pose_desired[0]);
    new_desired_pose_msg[0] = true;
}

void ACEnforcement::Tool1PoseDesiredCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    tf::poseMsgToKDL(msg->pose, tool_pose_desired[1]);
    new_desired_pose_msg[1] = true;

}

void ACEnforcement::Master0TwistCallback(
        const geometry_msgs::TwistStamped::ConstPtr &msg) {

    tf::twistMsgToKDL(msg->twist, master_twist_dvrk[0]);
    master_twist_dvrk[0] =  slave_frame_to_task_frame[0] * master_twist_dvrk[0];
}

void ACEnforcement::Master1TwistCallback(
        const geometry_msgs::TwistStamped::ConstPtr &msg) {

    tf::twistMsgToKDL(msg->twist, master_twist_dvrk[1]);
    master_twist_dvrk[1] =  slave_frame_to_task_frame[1] * master_twist_dvrk[1];

}

void ACEnforcement::Master0StateCallback(
        const std_msgs::StringConstPtr &msg) {
    master_state[0]  = msg->data;
}

void ACEnforcement::Master1StateCallback(
        const std_msgs::StringConstPtr &msg) {
    master_state[1]  = msg->data;
}

void ACEnforcement::ACParams0Callback(
        const active_constraints::ActiveConstraintParametersConstPtr &msg){

    // isn't there a better way to do this?!
    ac_params[0].active = msg->active;
    ac_params[0].method = msg->method;
    ac_params[0].angular_damping_coeff = msg->angular_damping_coeff;
    ac_params[0].angular_elastic_coeff = msg->angular_elastic_coeff;
    ac_params[0].linear_damping_coeff = msg->linear_damping_coeff;
    ac_params[0].linear_elastic_coeff = msg->linear_elastic_coeff;
    ac_params[0].max_force = msg->max_force;
    ac_params[0].max_torque = msg->max_torque;

    // will add params for other methods too
    if(ac_params[0].method == 0){
        ac_elastic[0]->setParameters(ac_params[0].linear_elastic_coeff,
                                     ac_params[0].linear_damping_coeff,
                                     ac_params[0].angular_elastic_coeff,
                                     ac_params[0].angular_damping_coeff);
    }


}


void ACEnforcement::ACParams1Callback(
        const active_constraints::ActiveConstraintParametersConstPtr &msg){

    // isn't there a better way to do this?!
    ac_params[1].active = msg->active;
    ac_params[1].method = msg->method;
    ac_params[1].angular_damping_coeff = msg->angular_damping_coeff;
    ac_params[1].angular_elastic_coeff = msg->angular_elastic_coeff;
    ac_params[1].linear_damping_coeff = msg->linear_damping_coeff;
    ac_params[1].linear_elastic_coeff = msg->linear_elastic_coeff;
    ac_params[1].max_force = msg->max_force;
    ac_params[1].max_torque = msg->max_torque;

    // will add params for other methods too
    if(ac_params[1].method == 0){
        ac_elastic[1]->setParameters(ac_params[1].linear_elastic_coeff,
                                     ac_params[1].linear_damping_coeff,
                                     ac_params[1].angular_elastic_coeff,
                                     ac_params[1].angular_damping_coeff);
    }

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
    while(master_state[0]!= "DVRK_READY"){

        if(n_arms>1){
            while(master_state[1]!= "DVRK_READY"){
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




double ACEnforcement::do_foaw_sample(double *posbuf, int size, int *k, double current_pos, int best,
                                     const double noise_max)
{
    int i, j, l, bad;
    double b, ykj;
    double velocity = 0;
    double T = 1/filtering_frequency;


    /* circular buffer */
    *k = (*k+1)%size;
    posbuf[*k] = current_pos;

    for (i=1; i<size; i++)
    {
        if (best)
        {
            // best-fit-FOAW
            b = 0;
            for (l=0; l<(i+1); l++)
                b +=  i*posbuf[(*k-l+size)%size]
                      - 2*posbuf[(*k-l+size)%size]*l;
            b = b / (T*i*(i+1)*(i+2)/6);
        }
        else
            // end-fit-FOAW
            b = (posbuf[*k]-posbuf[(*k-i+size)%size]) / (i*T);
        bad = 0;
        for (j=1; j<i; j++)
        {
            ykj = posbuf[*k]-(b*j*T);
            if (   (ykj < (posbuf[(*k-j+size)%size]-noise_max))
                   || (ykj > (posbuf[(*k-j+size)%size]+noise_max)))
            {
                bad = 1;
                break;
            }
        }
        if (bad) break;
        velocity = b;
    }

    return velocity;
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


