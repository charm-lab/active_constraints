
#include <iostream>
#include "ActiveConstraintEnforcement.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "ActiveConstraintsROS.hpp"
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

	ActiveConstraintsROS r(ros::this_node::getName());

	geometry_msgs::Wrench wrench_out;
	std_msgs::String robot1_state_command;
	std_msgs::Bool wrench_body_orientation_absolute;

	////////////////////////////

	double f_max = 4.0;
	double k_coeff = 500;
	double b_coeff = 20;

	acElastic ac_elastic( f_max,  k_coeff,  b_coeff,  0.01);
    acPlastRedirect ac_plast_redirect(f_max, 0.005, 0.002);
    acViscousRedirect ac_visc_redirect(f_max, 70, 0.002);

	KDL::Vector f_out;
	KDL::Vector p_tool, p_desired, v_msrd;


	ros::Rate loop_rate(r.ros_freq);

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
	r.pub_wrench_body_orientation_absolute.publish(wrench_body_orientation_absolute);

	half_second_sleep.sleep();
	ros::spinOnce();

	bool first_run = true;
	while(!g_request_shutdown){

		if(first_run){
//			p_desired[0] = r.slave_1_pose.pose.position.x;
//			p_desired[1] = r.slave_1_pose.pose.position.y;
//			p_desired[2] = r.slave_1_pose.pose.position.z;
            p_desired[0] =  0.08;
			p_desired[1] = -0.083;
			p_desired[2] = -0.106;

			ROS_INFO_STREAM(
					"p_desired[0] = " << p_desired[0] <<
					"p_desired[1] = " << p_desired[1] <<
					"p_desired[2] = " << p_desired[2]);

			first_run = false;

		}


		v_msrd[0] = r.slave_1_twist.twist.linear.x;
		v_msrd[1] = r.slave_1_twist.twist.linear.y;
		v_msrd[2] = r.slave_1_twist.twist.linear.z;
//		v_msrd[0] = 0.0;
//		v_msrd[1] = 0.0;
//		v_msrd[2] = 0.0;

		p_tool[0] = r.slave_1_pose.pose.position.x;
		p_tool[1] = r.slave_1_pose.pose.position.y;
		p_tool[2] = r.slave_1_pose.pose.position.z;

        if(r.coag_pressed){
//            if(r.new_coag_event){
//                robot1_state_command.data = "DVRK_EFFORT_CARTESIAN";
//                ROS_INFO("Setting robot state to %s", robot1_state_command.data.c_str());
//                r.new_coag_event = false;
//            }


            ac_elastic.getForce(f_out, p_tool, p_desired, v_msrd);

            wrench_out.force.x = f_out[0];
            wrench_out.force.y = f_out[1];
            wrench_out.force.z = f_out[2];

            if(r.master_1_state == "DVRK_EFFORT_CARTESIAN"){
                r.pub_master_1_wrench.publish(wrench_out);
            }
        }
        else{
//            if(r.new_coag_event){
//                robot1_state_command.data = "DVRK_EFFORT_CARTESIAN";
//                ROS_INFO("Setting robot state to %s", robot1_state_command.data.c_str());
//                r.new_coag_event = false;
//            }
            KDL::SetToZero(f_out);
        }




		//r.pub_wrench_body_orientation_absolute.publish(wrench_body_orientation_absolute);


		loop_rate.sleep();
		ros::spinOnce();

		//		}

	}

	loop_rate.sleep();

	wrench_out.force.x = 0.0;
	wrench_out.force.y = 0.0;
	wrench_out.force.z = 0.0;
    ROS_INFO("Setting zero forces...");
    r.pub_master_1_wrench.publish(wrench_out);

//	robot1_state_command.data = "DVRK_READY";
//	ROS_INFO("Setting robot state to DVRK_GRAVITY_COMPENSATION");
//	r.pub_tool_1_set_state.publish(robot1_state_command);
	loop_rate.sleep();

	ROS_INFO("Ending Session...\n");
	ros::shutdown();

    return 0;
}
