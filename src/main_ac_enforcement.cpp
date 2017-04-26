
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

    KDL::Wrench wrench_out[2];

    // the forces are published only when new desired poses are arrived.
    // so the frequency of the published forces is equal to the frequency
    // of the desired pose topic
    // loop_rate is the frequency of spinning and checking for new messages
    ros::Rate loop_rate(200);

    r.StartTeleop();

	ros::spinOnce();

    while(!g_request_shutdown){

        for (int k = 0; k < r.n_arms; ++k) {

            if(r.IsDesiredPoseNew(k)) {

                // Generate non-zero wrenches when:
                // 1- coag foot switch is pressed (master/slave are coupled)
                // 2- and clutch is not pressed (to move master while slave is fixed)
                // 3- and if we are told that active constraint should be active
                if (r.coag_pressed && !r.clutch_pressed  && r.ac_params[k].active)
                    wrench_out[k] = r.GetWrench(k);
                else
                    KDL::SetToZero(wrench_out[k]);

                if (r.IsPublishingAllowed() && r.master_state[k] == "DVRK_EFFORT_CARTESIAN")
                    r.PublishWrenchInSlaveFrame(k, wrench_out[k]);

            } // if(new_desired_pose_msg[k])
        } // for

		loop_rate.sleep();
		ros::spinOnce();

	}

	loop_rate.sleep();

    ROS_INFO("Setting wrenches to zero...");
    r.PublishWrenchInSlaveFrame(0, KDL::Wrench());

	loop_rate.sleep();

    //    ROS_INFO("Turning dvrk off...");
    //    std_msgs::Empty empty;
    //    r.pub_dvrk_power_off.publish(empty);

	ROS_INFO("Ending Session...\n");
	ros::shutdown();

    return 0;
}
