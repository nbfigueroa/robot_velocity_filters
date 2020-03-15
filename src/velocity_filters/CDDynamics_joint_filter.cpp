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

#include "CDDynamics_joint_filter.h"

CDDynamicsJointFilter::CDDynamicsJointFilter(ros::NodeHandle &n,
        double frequency,
        std::string input_velocity_topic_name,
        std::string output_velocity_topic_name, 
        double joint_velocity_limit)
	: nh_(n),
	  loop_rate_(frequency),
	  input_velocity_topic_name_(input_velocity_topic_name),
	  output_velocity_topic_name_(output_velocity_topic_name),
	  dt_(1 / frequency), Wn_(2.5), M_(6),
	  filt_vlim_(0.5), filt_alim_(0.0),
	  joint_velocity_limit_(joint_velocity_limit){
		
		ROS_INFO_STREAM("CDDynamics joint velocity filter node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}


bool CDDynamicsJointFilter::Init() {

	desired_velocity_.Resize(M_);
	desired_velocity_filtered_.Resize(M_);
	velLimits_.Resize(M_);
	accLimits_.Resize(M_);

	/* Define vel and acc limits for filter */
	for (int i=0; i<M_; i++){
		velLimits_(i) = filt_vlim_;
		accLimits_(i) = filt_alim_;
	}	

	initial_.Resize(M_);
	initial_.Zero();

	/* Initialize the filter for desired joint velocities*/
	CCDyn_filter_joint_.reset (new CDDynamics(M_, dt_, Wn_));
	CCDyn_filter_joint_->SetVelocityLimits(velLimits_);
	CCDyn_filter_joint_->SetAccelLimits(accLimits_);
	CCDyn_filter_joint_->SetState(initial_);
	CCDyn_filter_joint_->SetTarget(initial_);

	/* Initialize ROS stuff */
	if (!InitializeROS()) {
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}

	return true;
}



bool CDDynamicsJointFilter::InitializeROS() {

	/* Subscriber */
	sub_desired_joint_velocity_ = nh_.subscribe( input_velocity_topic_name_ , 1000,
	                                &CDDynamicsJointFilter::UpdateDesiredVelocities, this, ros::TransportHints().reliable().tcpNoDelay());
	/* Publisher */
	pub_desired_joint_velocity_filtered_ = nh_.advertise<std_msgs::Float64MultiArray>(output_velocity_topic_name_, 1);


	/* Clear data from message types */
	msg_desired_joint_velocity_.data.clear();
	msg_desired_joint_velocity_filtered_.data.clear();

	if (nh_.ok()) { 
		ros::spinOnce();
		ROS_INFO("The filter is ready.");
		return true;
	}
	else {
		ROS_ERROR("The ros node has a problem.");
		return false;
	}

}


void CDDynamicsJointFilter::Run() {

	while (nh_.ok()) {
		ros::spinOnce();
		loop_rate_.sleep();
	}
}


void CDDynamicsJointFilter::UpdateDesiredVelocities(const std_msgs::Float64MultiArray::ConstPtr& msg) {

	msg_desired_joint_velocity_ = *msg;

	/* Extract joint velocity values from array message */
	int i = 0;
	for(std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
		desired_velocity_(i) = *it;
		i++;
	}

	FilterDesiredVelocities();
	PublishDesiredVelocities();

}

void CDDynamicsJointFilter::FilterDesiredVelocities() {

	mutex_.lock();

	/******************* Filter the desired linear velocity *******************/
	/* Checks to send feasible velocities to low-level controller */
	if (std::isnan(desired_velocity_.Norm2())) {
		ROS_WARN_THROTTLE(1, "Desired joint velocities have NaN values. Setting the output to zero.");
		desired_velocity_filtered_.Zero();
	}

	/* Filter the desired linear velocity */
	if (desired_velocity_.Norm2() == 0) {
	    desired_velocity_filtered_.Zero();
		CCDyn_filter_joint_->SetState(initial_);
        CCDyn_filter_joint_->SetTarget(initial_);
	}
	else{
		CCDyn_filter_joint_->SetTarget(desired_velocity_);
		CCDyn_filter_joint_->Update();
		CCDyn_filter_joint_->GetState(desired_velocity_filtered_);
		
		/* Cap velocities with limits */
		for (int j=0; j<M_;j++){			
			/* Cap velocity with desired velocity from motion generator */
			if (abs(desired_velocity_filtered_(j)) > abs(desired_velocity_(j))) 
				desired_velocity_filtered_(j) = desired_velocity_(j);	

			/* Cap velocity with maximum limits */
			if (desired_velocity_filtered_(j) > joint_velocity_limit_) 
				desired_velocity_filtered_(j) = joint_velocity_limit_;
		}
		

	}		

	ROS_INFO_STREAM( "--------------------------------------------------------");
	ROS_INFO_STREAM( "Desired vel: "  << desired_velocity_(0) << " " << desired_velocity_(1) << " " << desired_velocity_(2) 
									  << desired_velocity_(3) << " " << desired_velocity_(4) << " " << desired_velocity_(5));
	ROS_INFO_STREAM( "Filtered vel: " << desired_velocity_filtered_(0) << " " << desired_velocity_filtered_(1) << " " << desired_velocity_filtered_(2)
									  << desired_velocity_filtered_(3) << " " << desired_velocity_filtered_(4) << " " << desired_velocity_filtered_(5));
	ROS_INFO_STREAM( "--------------------------------------------------------");

	mutex_.unlock();

}


void CDDynamicsJointFilter::PublishDesiredVelocities() {

	
	msg_desired_joint_velocity_filtered_.data.clear();
	for (int j = 0; j < M_; j++)
			msg_desired_joint_velocity_filtered_.data.push_back(desired_velocity_filtered_(j));

	pub_desired_joint_velocity_filtered_.publish(msg_desired_joint_velocity_filtered_);

}




