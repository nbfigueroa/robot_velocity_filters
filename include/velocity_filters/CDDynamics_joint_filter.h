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

#ifndef __CDDYNAMICS_JOINT_FILTER_H__
#define __CDDYNAMICS_JOINT_FILTER_H__

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <vector>
#include "MathLib.h"
#include "CDDynamics.h"
#include <mutex>

/* ***************************************************
	CDDynamics is a Critical Damped 2nd order Dynamics
	filter used to smooth our states or velocities
	with velocity and acceleration limits.  

*******************************************************  */
class CDDynamicsJointFilter {


private:

	// Filter variables
	std::unique_ptr<CDDynamics>   CCDyn_filter_joint_;
	double                        dt_;
	double                        Wn_;
	MathLib::Vector               accLimits_;
	MathLib::Vector               velLimits_;
	int                           M_;
	double                        filt_vlim_, filt_alim_;
	double                        joint_velocity_limit_;

	// ROS variables
	ros::NodeHandle               nh_;
	ros::Rate                     loop_rate_;	
	ros::Subscriber               sub_desired_joint_velocity_;
	ros::Publisher                pub_desired_joint_velocity_filtered_;

	std::string                   input_velocity_topic_name_;
	std::string                   output_velocity_topic_name_;

	std_msgs::Float64MultiArray   msg_desired_joint_velocity_;
	std_msgs::Float64MultiArray   msg_desired_joint_velocity_filtered_;

	// Internal Class variables
	std::mutex                    mutex_;
	MathLib::Vector               initial_;
	MathLib::Vector               desired_velocity_;
	MathLib::Vector               desired_velocity_filtered_;


public:
	CDDynamicsJointFilter(ros::NodeHandle &n,
	                      	    double frequency,
						        std::string input_velocity_topic_name,
						        std::string output_filtered_velocity_topic_name,
						        double lin_velocity_limit);

	bool Init();
	void Run();

private:

	bool InitializeROS();
	void UpdateDesiredVelocities(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void FilterDesiredVelocities();
	void PublishDesiredVelocities();

};


#endif
