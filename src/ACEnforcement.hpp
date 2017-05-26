//==============================================================================
/*
    Software License provided in the home directory, Nearlab

    \author    <http://nearlab.polimi.it/>
    \author    Nima Enayati
    \version   -
*/
//==============================================================================
/*
 * ActiveConstraintsRos.hpp
 *
 *  Created on: Jan 23, 2017
 *      Author: nima
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
#include "ActiveConstraintEnforcementMethods.hpp"
#include "custom_msgs/ActiveConstraintParameters.h"

/**
 * \class ACEnforcement
 * \brief This is a .
 *  NOTE we are using the master's velocity
 *  I noticed there was about 100 ms delay in the dvrk teleop loop, which made
 *  the force feedback unstable specially the viscous part. Using the master's
 *  twist instead of the slave made things much better. If there was no clutching
 *  I would have used also the position of the master
 *
 *  Methods: 0: visco/elastic
 *           1: ? (TODO)
 *           2: ?
 *           3: ?
 */

enum ac_method :uint {ViscoElastic, Plastic, PlasticRedirect, ViscousRedirect};

class ACEnforcement {

public:

    ACEnforcement(std::string node_name);

    /**
    * @brief  Sends the dvrk arms to home and asks them to start the teleop
     * mode
    **/
    void StartTeleop();

public:

    /**
    * @brief  Takes the wrench to slave/master coordinate frame and publish
     *
     * @param arm_number. 1 or 2.
     *
    **/
    void PublishWrenchInSlaveFrame(const int num_arm, const KDL::Wrench wrench);

    /**
     * @brief   Assuming once this method is called we can mark the data as used.
     *
     * @param arm_number. 1 or 2.
    */
    bool IsDesiredPoseNew(const int arm_number);

    /**
    * @brief   We publish in two cases:
     * 1- when the clutch is not pressed
     * 2- when clutch is pressed. In this case we publish just for one time
     * to prevent the data being cluttered by zeros in case we are going to
     * analyze the wrench data later.
     */
    bool IsPublishingAllowed();

    /**
    * @brief   generates the active constraint wrench based on the method assigned in
     * .
     *
     * @param arm_number. 1 or 2.
   */
    KDL::Wrench GetWrench(const int arm_number);

public:

    int n_arms;

    std::string master_state[2];
    bool coag_pressed;
    bool clutch_pressed;
    bool new_clutch_event;

    custom_msgs::ActiveConstraintParameters ac_params[2];

private:

    void SetupROSCommunications();

    void Slave0PoseCurrentCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);

    void Slave1PoseCurrentCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);

    void Master0PoseCurrentCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);

    void Master1PoseCurrentCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);

    void Tool0PoseDesiredCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);

    void Tool1PoseDesiredCallback(
            const geometry_msgs::PoseStamped::ConstPtr &msg);


    void Master0TwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    void Master1TwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    void Master0StateCallback(const std_msgs::StringConstPtr &msg);

    void Master1StateCallback(const std_msgs::StringConstPtr &msg);

    void ACParams0Callback(
            const custom_msgs::ActiveConstraintParametersConstPtr &msg);

    void ACParams1Callback(
            const custom_msgs::ActiveConstraintParametersConstPtr &msg);

    void FootPedalCoagCallback(const sensor_msgs::Joy &msg);

    void FootPedalClutchCallback(const sensor_msgs::Joy &msg);


    // two function pointers for slave pose current callbacks
    void (ACEnforcement::*slave_pose_current_callbacks[2])
            (const geometry_msgs::PoseStamped::ConstPtr &msg);

    // two function pointers for slave pose current callbacks
    void (ACEnforcement::*master_pose_current_callbacks[2])
            (const geometry_msgs::PoseStamped::ConstPtr &msg);

    // two function pointers for slave pose desired callbacks
    void (ACEnforcement::*tool_pose_desired_callbacks[2])
            (const geometry_msgs::PoseStamped::ConstPtr &msg);

    // two function pointers for slave twist callbacks
    void (ACEnforcement::*master_twist_callback[2])
            (const geometry_msgs::TwistStamped::ConstPtr &msg);

    // two function pointers for master state callbacks
    void (ACEnforcement::*master_state_callback[2])
            (const std_msgs::StringConstPtr &msg);

    // two function pointers for master state callbacks
    void (ACEnforcement::*ac_params_callback[2])
            (const custom_msgs::ActiveConstraintParametersConstPtr &msg);

    double do_foaw_sample(double *posbuf, int size, int *k,
                          double current_pos, int best, const double noise);
private:

    acElastic * ac_elastic[2];
    acPlast *  ac_plast[2];
    acPlastRedirect *  ac_plast_redirect[2];
    acViscousRedirect * ac_visc_redirect[2];

    KDL::Frame slave_pose_current[2];
    KDL::Frame master_pose_current[2];
    KDL::Frame tool_pose_desired[2];
    KDL::Twist master_twist_dvrk[2];
    KDL::Twist master_twist_filt[2];

    // velocity filtering
    double filtering_frequency;
    const int foaw_n = 15;
    int ** foaw_i;
//    int foaw_i_0[3] = {0, 0 ,0};
//    int foaw_i_1[3] = {0, 0 ,0};
    double *** foaw_position_buffer;

    // Tool poses are all in task coordinate frame
    bool new_desired_pose_msg[2];

    ros::NodeHandle n;
    ros::Subscriber *subscriber_slave_pose_current;
    ros::Subscriber *subscriber_master_pose_current;
    ros::Subscriber *subscriber_tool_pose_desired;
    ros::Subscriber *subscriber_slaves_current_twist;

    ros::Subscriber *subscriber_master_state;
    ros::Subscriber *subscriber_ac_params;
    ros::Subscriber subscriber_foot_pedal_coag;
    ros::Subscriber subscriber_foot_pedal_clutch;

    ros::Publisher pub_dvrk_console_teleop_enable;
    ros::Publisher pub_dvrk_home;
    ros::Publisher pub_dvrk_power_off;
    ros::Publisher pub_twist;
    ros::Publisher *publisher_wrench;
    ros::Publisher *publisher_wrench_body_orientation_absolute;

    // slave_frame_to_task_frame used to take the generated forces to slave ref rame
    KDL::Frame slave_frame_to_task_frame[2];

};

namespace conversions{

    void VectorToKDLFrame(const std::vector<double> &in_vec, KDL::Frame &out_pose);
    void VectorToPoseMsg(const std::vector<double> in_vec,
                         geometry_msgs::Pose &out_pose);

};
// operator overload to print out vectors
std::ostream& operator<<(std::ostream& out, const std::vector<double>& vect);
std::ostream& operator<<(std::ostream& out, const KDL::Vector& vec);


#endif /* SRC_ACTIVECONSTRAINTSROS_HPP_ */
