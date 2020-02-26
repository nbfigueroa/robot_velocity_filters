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

#include "CDDynamics_filter.h"

CDDynamicsFilter::CDDynamicsFilter(ros::NodeHandle &n,
        double frequency,
        std::string input_state_topic_name,
        std::string input_velocity_topic_name,
        std::string output_velocity_topic_name)
	: nh_(n),
	  loop_rate_(frequency),
	  input_state_topic_name_(input_state_topic_name),
	  input_velocity_topic_name_(input_velocity_topic_name),
	  output_velocity_topic_name_(output_velocity_topic_name),
	  dt_(1 / frequency),
	  Wn_(15), 
	  M_(3),
	  vlim_(1), 
	  alim_(3){

	ROS_INFO_STREAM("CDDynamics twist filter node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}


bool CDDynamicsFilter::Init() {

	current_state_.Resize(M_);
	desired_velocity_lin_.Resize(M_);
	desired_velocity_ang_.Resize(M_);
	desired_velocity_filtered_lin_.Resize(M_);
	velLimits_.Resize(M_);
	accLimits_.Resize(M_);

	/* Define vel and acc limits for filter */
	for (int i=0; i<M_; i++){
		velLimits_(i) = vlim_;
		accLimits_(i) = alim_;
	}	

	/* Initialize the filter */
	CCDyn_filter_.reset (new CDDynamics(M_, dt_, Wn_));
	CCDyn_filter_->SetVelocityLimits(velLimits_);
	CCDyn_filter_->SetAccelLimits(accLimits_);
	MathLib::Vector initial(M_);
	initial.Zero();
	CCDyn_filter_->SetState(initial);
	CCDyn_filter_->SetTarget(initial);


	/* Initialize ROS stuff */
	if (!InitializeROS()) {
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}

	return true;
}



bool CDDynamicsFilter::InitializeROS() {

	/* Subscribers */
	sub_real_pose_     = nh_.subscribe( input_state_topic_name_ , 1000,
	                                &CDDynamicsFilter::UpdateCurrentPose, this, ros::TransportHints().reliable().tcpNoDelay());

	sub_desired_twist_ = nh_.subscribe( input_velocity_topic_name_ , 1000,
	                                &CDDynamicsFilter::UpdateDesiredTwist, this, ros::TransportHints().reliable().tcpNoDelay());

	/* Publisher */
	pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::Twist>(output_velocity_topic_name_, 1);


	if (nh_.ok()) { // Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The filter is ready.");
		return true;
	}
	else {
		ROS_ERROR("The ros node has a problem.");
		return false;
	}

}


void CDDynamicsFilter::Run() {

	while (nh_.ok()) {

		// FilterDesiredVelocity();
		// PublishDesiredVelocity();
		ros::spinOnce();
		loop_rate_.sleep();
	}
}

void CDDynamicsFilter::UpdateCurrentPose(const geometry_msgs::Pose::ConstPtr& msg) {

	msg_real_pose_ = *msg;

	current_state_(0) = msg_real_pose_.position.x;
	current_state_(1) = msg_real_pose_.position.y;
	current_state_(2) = msg_real_pose_.position.z;

}


void CDDynamicsFilter::UpdateDesiredTwist(const geometry_msgs::Twist::ConstPtr& msg) {

	msg_desired_twist_ = *msg;

	desired_velocity_lin_(0) = msg_desired_twist_.linear.x;
	desired_velocity_lin_(1) = msg_desired_twist_.linear.y;
	desired_velocity_lin_(2) = msg_desired_twist_.linear.z;

	desired_velocity_ang_(0) = msg_desired_twist_.angular.x;
	desired_velocity_ang_(1) = msg_desired_twist_.angular.y;
	desired_velocity_ang_(2) = msg_desired_twist_.angular.z;


	FilterDesiredVelocity();
	PublishDesiredVelocity();

}

void CDDynamicsFilter::FilterDesiredVelocity() {

	mutex_.lock();

	if (std::isnan(desired_velocity_lin_.Norm2())) {
		ROS_WARN_THROTTLE(1, "Desired linear velocity is NaN. Setting the output to zero.");
		desired_velocity_filtered_lin_.Zero();
	}

	if (std::isnan(desired_velocity_ang_.Norm2())) {
		ROS_WARN_THROTTLE(1, "Desired angular velocity is NaN. Setting the output to zero.");
		desired_velocity_ang_.Zero();
	}


	if (desired_velocity_lin_.Norm2() == 0) 
		desired_velocity_filtered_lin_.Zero();
	else
	{
		/* Filter the desired velocity */
		CCDyn_filter_->SetTarget(desired_velocity_lin_);
		CCDyn_filter_->Update();
		CCDyn_filter_->GetState(desired_velocity_filtered_lin_);
	}

	ROS_INFO_STREAM( "Desired linear vel: "  << desired_velocity_lin_(0) << " " << desired_velocity_lin_(1) << " " << desired_velocity_lin_(2));
	ROS_INFO_STREAM( "Filtered linear vel: " << desired_velocity_filtered_lin_(0) << " " << desired_velocity_filtered_lin_(1) << " " << desired_velocity_filtered_lin_(2));

	mutex_.unlock();

}


void CDDynamicsFilter::PublishDesiredVelocity() {

	// msg_desired_twist_filtered_.header.stamp    = ros::Time::now();
	msg_desired_twist_filtered_.linear.x  = desired_velocity_filtered_lin_(0);
	msg_desired_twist_filtered_.linear.y  = desired_velocity_filtered_lin_(1);
	msg_desired_twist_filtered_.linear.z  = desired_velocity_filtered_lin_(2);
	msg_desired_twist_filtered_.angular.x = desired_velocity_ang_(0);
	msg_desired_twist_filtered_.angular.y = desired_velocity_ang_(1);
	msg_desired_twist_filtered_.angular.z = desired_velocity_ang_(2);

	pub_desired_twist_filtered_.publish(msg_desired_twist_filtered_);

}




