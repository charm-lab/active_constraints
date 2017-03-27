
#include <iostream>
#include "ActiveConstraintEnforcementMethods.hpp"

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

    ros::init(argc, argv, "dvrk_ac", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);

    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

	ACEnforcement r(ros::this_node::getName());

	std_msgs::String robot1_state_command;
	std_msgs::Bool wrench_body_orientation_absolute;

	////////////////////////////

	double f_max = 4.00;
	double k_coeff = 200;
	double b_coeff = 10;

	acElastic ac_elastic( f_max,  k_coeff,  b_coeff,  0.01);
    acPlastRedirect ac_plast_redirect(f_max, 0.005, 0.002);
    acViscousRedirect ac_visc_redirect(f_max, 70, 0.002);

	KDL::Vector f_out;

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


    KDL::Rotation slave_to_master_tr;
    slave_to_master_tr.data[4]  = -1;
    slave_to_master_tr.data[8]  = -1;
    KDL::Rotation slave_to_master_tr_2;
    slave_to_master_tr_2.data[0]  = -1;
    slave_to_master_tr_2.data[4]  = -0.866025404;
    slave_to_master_tr_2.data[5]  = -0.5;
    slave_to_master_tr_2.data[7]  = -0.5;
    slave_to_master_tr_2.data[8]  =  0.866025404;

	bool first_run = true;
	while(!g_request_shutdown){


        if(r.new_desired_pose_msg[0]) {
            r.new_desired_pose_msg[0] = false;
            if (r.coag_pressed) {
                ac_elastic.getForce(f_out, r.tool_pose_current[0].p, r.tool_pose_desired[0].p, r.tool_twist[0].vel);
            } else {
                KDL::SetToZero(f_out);
            }

            if (r.master_1_state == "DVRK_EFFORT_CARTESIAN") {
                r.PublishWrenchInSlaveFrame(0, f_out);
            }

        }


		//r.pub_wrench_body_orientation_absolute.publish(wrench_body_orientation_absolute);


		loop_rate.sleep();
		ros::spinOnce();

		//		}

	}

	loop_rate.sleep();

    r.PublishWrenchInSlaveFrame(0, KDL::Vector(0.0, 0.0, 0.0));

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
