/*
 * Copyright (C) 2020 Interactive Robotics Group, MIT, USA
 * Author:  Nadia Figueroa
 * email:   nadiafig@mit.edu
 * website: https://nbfigueroa.github.io/
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __CDDYNAMICS_FILTER_H__
#define __CDDYNAMICS_FILTER_H__

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include "MathLib.h"
#include "CDDynamics.h"
#include <mutex>

/* ***************************************************
	CDDynamics is a Critical Damped 2nd order Dynamics
	filter used to smooth our states or velocities
	with velocity and acceleration limits.  

*******************************************************  */
class CDDynamicsFilter {


private:

	// Filter variables
	std::unique_ptr<CDDynamics> CCDyn_filter_;
	double                      dt_;
	double                      Wn_;
	MathLib::Vector             accLimits_;
	MathLib::Vector             velLimits_;
	int                         M_;
	double                      vlim_;
	double                      alim_;
	
	// ROS variables
	ros::NodeHandle             nh_;
	ros::Rate                   loop_rate_;

	ros::Subscriber             sub_real_pose_;
	ros::Subscriber             sub_desired_twist_;
	ros::Publisher              pub_desired_twist_filtered_;

	std::string                 input_state_topic_name_;
	std::string                 input_velocity_topic_name_;
	std::string                 output_velocity_topic_name_;

	geometry_msgs::Pose         msg_real_pose_;
	geometry_msgs::Twist        msg_desired_twist_;
	geometry_msgs::Twist        msg_desired_twist_filtered_;

	// Internal Class variables
	std::mutex                  mutex_;
	MathLib::Vector             current_state_;
	MathLib::Vector             desired_velocity_lin_;
	MathLib::Vector             desired_velocity_ang_;
	MathLib::Vector             desired_velocity_filtered_lin_;


public:
	CDDynamicsFilter(ros::NodeHandle &n,
	                      	    double frequency,
						        std::string input_state_topic_name,
						        std::string input_velocity_topic_name,
						        std::string output_filtered_velocity_topic_name);

	bool Init();
	void Run();

private:

	bool InitializeROS();
	void UpdateCurrentPose(const geometry_msgs::Pose::ConstPtr& msg);
	void UpdateDesiredTwist(const geometry_msgs::Twist::ConstPtr& msg);
	void FilterDesiredVelocity();
	void PublishDesiredVelocity();

};


#endif
