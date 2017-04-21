
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "ACEnforcement.hpp"
#include <ros/xmlrpc_manager.h>
#include <signal.h>



// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
    g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    int num_params = 0;
    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
        num_params = params.size();
    if (num_params > 1)
    {
        std::string reason = params[1];
        ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
        g_request_shutdown = 1; // Set flag
    }

    result = ros::xmlrpc::responseInt(1, "", 0);
}



int main(int argc, char *argv[]) {


	/////////////////////////
    // Override SIGINT handler

    ros::init(argc, argv, "dvrk_ac_enforce", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);

    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

	ACEnforcement r(ros::this_node::getName());

	std_msgs::String robot1_state_command;
	std_msgs::Bool wrench_body_orientation_absolute;


    KDL::Vector f_out[2];
    KDL::Vector taw_out[2];

    // the forces are published only when new desired poses are arrived.
    // so the frequency of the published forces is equal to the frequency
    // of the desired pose topic
    // loop_rate is the frequency of spinning and checking for new messages
    ros::Rate loop_rate(200);

	ros::Rate half_second_sleep(1);
    half_second_sleep.sleep();

//    robot1_state_command.data = "Home";
//    ROS_INFO("Setting robot state to %s", robot1_state_command.data.c_str());
//    r.pub_tool_1_set_state.publish(robot1_state_command);
//
//
//    while(r.master_1_state !=  "DVRK_READY" && !g_request_shutdown){
//        half_second_sleep.sleep();
//        ros::spinOnce();
//    }
//
//
//
//	r.pub_tool_1_set_state.publish(robot1_state_command);

    r.StartTeleop();
    wrench_body_orientation_absolute.data = 1;
	r.publisher_wrench_body_orientation_absolute->publish(wrench_body_orientation_absolute);

	half_second_sleep.sleep();
	ros::spinOnce();


	bool first_run = true;
	while(!g_request_shutdown){

        for (int k = 0; k < r.n_arms; ++k) {

            if(r.new_desired_pose_msg[k] && r.ac_params[k].active) {

                r.new_desired_pose_msg[k] = false;

                if (r.coag_pressed && !r.clutch_pressed) {
                    r.ac_elastic[k]->getForce(f_out[k], r.slave_pose_current[k].p,
                                              r.tool_pose_desired[k].p, r.master_twist_filt[k].vel);
                    r.ac_elastic[k]->getTorque(taw_out[k], r.slave_pose_current[k].M,
                                               r.tool_pose_desired[k].M, r.master_twist_filt[k].rot);
                } else {
                    KDL::SetToZero(f_out[k]);
                    KDL::SetToZero(taw_out[k]);
                }

                if (r.master_state[k] == "DVRK_EFFORT_CARTESIAN") {
                    r.PublishWrenchInSlaveFrame(k, f_out[k], taw_out[k]);
                }

            }
        }

		//r.pub_wrench_body_orientation_absolute.publish(wrench_body_orientation_absolute);


		loop_rate.sleep();
		ros::spinOnce();

	}

	loop_rate.sleep();

    r.PublishWrenchInSlaveFrame(0, KDL::Vector(0.0, 0.0, 0.0), KDL::Vector());

    ROS_INFO("Setting zero forces...");

//	robot1_state_command.data = "DVRK_READY";
//	ROS_INFO("Setting robot state to DVRK_GRAVITY_COMPENSATION");
//	r.pub_tool_1_set_state.publish(robot1_state_command);
	loop_rate.sleep();
    std_msgs::Empty empty;
    r.pub_dvrk_power_off.publish(empty);

	ROS_INFO("Ending Session...\n");
	ros::shutdown();

    return 0;
}
