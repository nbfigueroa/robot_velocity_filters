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
        std::string output_velocity_topic_name, 
        double lin_velocity_limit,
        double ang_velocity_limit)
	: nh_(n),
	  loop_rate_(frequency),
	  input_state_topic_name_(input_state_topic_name),
	  input_velocity_topic_name_(input_velocity_topic_name),
	  output_velocity_topic_name_(output_velocity_topic_name),
	  dt_(1 / frequency), Wn_(2.5), M_(3),
	  filt_vlim_(0.0), filt_alim_(0.0),
	  lin_velocity_limit_(lin_velocity_limit), ang_velocity_limit_(ang_velocity_limit){
		
		ROS_INFO_STREAM("CDDynamics twist filter node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}


bool CDDynamicsFilter::Init() {

	desired_velocity_lin_.Resize(M_);
	desired_velocity_ang_.Resize(M_);
	desired_velocity_filtered_lin_.Resize(M_);
	desired_velocity_filtered_ang_.Resize(M_);	
	velLimits_.Resize(M_);
	accLimits_.Resize(M_);

	/* Define vel and acc limits for filter */
	for (int i=0; i<M_; i++){
		velLimits_(i) = filt_vlim_;
		accLimits_(i) = filt_alim_;
	}	

	initial_.Resize(M_);
	initial_.Zero();

	/* Initialize the filter for linear velocity component */
	CCDyn_filter_lin_.reset (new CDDynamics(M_, dt_, Wn_));
	CCDyn_filter_lin_->SetVelocityLimits(velLimits_);
	CCDyn_filter_lin_->SetAccelLimits(accLimits_);
	CCDyn_filter_lin_->SetState(initial_);
	CCDyn_filter_lin_->SetTarget(initial_);

	/* Initialize the filter for angular velocity component */
	CCDyn_filter_ang_.reset (new CDDynamics(M_, dt_, Wn_));
	CCDyn_filter_ang_->SetVelocityLimits(velLimits_);
	CCDyn_filter_ang_->SetAccelLimits(accLimits_);
	CCDyn_filter_ang_->SetState(initial_);
	CCDyn_filter_ang_->SetTarget(initial_);


	/* Initialize ROS stuff */
	if (!InitializeROS()) {
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}

	return true;
}



bool CDDynamicsFilter::InitializeROS() {

	/* Subscriber */
	sub_desired_twist_ = nh_.subscribe( input_velocity_topic_name_ , 1000,
	                                &CDDynamicsFilter::UpdateDesiredTwist, this, ros::TransportHints().reliable().tcpNoDelay());
	/* Publisher */
	pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::Twist>(output_velocity_topic_name_, 1);


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


void CDDynamicsFilter::Run() {

	while (nh_.ok()) {
		ros::spinOnce();
		loop_rate_.sleep();
	}
}


void CDDynamicsFilter::UpdateDesiredTwist(const geometry_msgs::Twist::ConstPtr& msg) {

	msg_desired_twist_ = *msg;

	desired_velocity_lin_(0) = msg_desired_twist_.linear.x;
	desired_velocity_lin_(1) = msg_desired_twist_.linear.y;
	desired_velocity_lin_(2) = msg_desired_twist_.linear.z;

	desired_velocity_ang_(0) = msg_desired_twist_.angular.x;
	desired_velocity_ang_(1) = msg_desired_twist_.angular.y;
	desired_velocity_ang_(2) = msg_desired_twist_.angular.z;

	FilterDesiredVelocities();
	PublishDesiredVelocities();

}

void CDDynamicsFilter::FilterDesiredVelocities() {

	mutex_.lock();

	/******************* Filter the desired linear velocity *******************/
	/* Checks to send feasible velocities to low-level controller */
	if (std::isnan(desired_velocity_lin_.Norm2())) {
		ROS_WARN_THROTTLE(1, "Desired linear velocity is NaN. Setting the output to zero.");
		desired_velocity_filtered_lin_.Zero();
	}

	/* Filter the desired linear velocity */
	if (desired_velocity_lin_.Norm2() == 0) {
	    desired_velocity_filtered_lin_.Zero();
		CCDyn_filter_lin_->SetState(initial_);
        CCDyn_filter_lin_->SetTarget(initial_);
	}
	else{
		CCDyn_filter_lin_->SetTarget(desired_velocity_lin_);
		CCDyn_filter_lin_->Update();
		CCDyn_filter_lin_->GetState(desired_velocity_filtered_lin_);
		
		/* Cap velocity with maximum limits */
		if (desired_velocity_filtered_lin_.Norm() > lin_velocity_limit_) 
			desired_velocity_filtered_lin_ = desired_velocity_filtered_lin_ / desired_velocity_filtered_lin_.Norm() * lin_velocity_limit_;

		/* Cap velocity with desired velocity from motion generator */
		if (desired_velocity_filtered_lin_.Norm() > desired_velocity_lin_.Norm()) 
			desired_velocity_filtered_lin_ = desired_velocity_filtered_lin_ / desired_velocity_filtered_lin_.Norm() * desired_velocity_lin_.Norm();

	}		

	ROS_INFO_STREAM( "--------------------------------------------------------");
	// ROS_INFO_STREAM( "Desired linear vel: "  << desired_velocity_lin_(0) << " " << desired_velocity_lin_(1) << " " << desired_velocity_lin_(2));
	// ROS_INFO_STREAM( "Filtered linear vel: " << desired_velocity_filtered_lin_(0) << " " << desired_velocity_filtered_lin_(1) << " " << desired_velocity_filtered_lin_(2));
	ROS_INFO_STREAM( "||x_dot|| desired: "  << desired_velocity_lin_.Norm() << " filtered: " << desired_velocity_filtered_lin_.Norm() );

	/******************* Filter the desired angular velocity *******************/	
	/* Checks to send feasible velocities to low-level controller */
	if (std::isnan(desired_velocity_ang_.Norm2())) {
		ROS_WARN_THROTTLE(1, "Desired angular velocity is NaN. Setting the output to zero.");
		desired_velocity_filtered_ang_.Zero();
	}

	/* Filter the desired angular velocity */
	if (desired_velocity_ang_.Norm2() == 0) {
	    desired_velocity_filtered_ang_.Zero();
		CCDyn_filter_ang_->SetState(initial_);
        CCDyn_filter_ang_->SetTarget(initial_);
	}
	else{
		CCDyn_filter_ang_->SetTarget(desired_velocity_ang_);
		CCDyn_filter_ang_->Update();
		CCDyn_filter_ang_->GetState(desired_velocity_filtered_ang_);
		
		/* Cap velocity with maximum limits */
		if (desired_velocity_filtered_ang_.Norm() > ang_velocity_limit_) 
			desired_velocity_filtered_ang_ = desired_velocity_filtered_ang_ / desired_velocity_filtered_ang_.Norm() * ang_velocity_limit_;

		/* Cap velocity with desired velocity from motion generator */
		if (desired_velocity_filtered_ang_.Norm() > desired_velocity_ang_.Norm()) 
			desired_velocity_filtered_ang_ = desired_velocity_filtered_ang_ / desired_velocity_filtered_ang_.Norm() * desired_velocity_ang_.Norm();
	}		
	
    ROS_INFO_STREAM( "||omega|| desired: "  << desired_velocity_ang_.Norm() << " filtered: " << desired_velocity_filtered_ang_.Norm() );

	// ROS_INFO_STREAM( "Desired angular vel: "  << desired_velocity_ang_(0) << " " << desired_velocity_ang_(1) << " " << desired_velocity_ang_(2));
	// ROS_INFO_STREAM( "Filtered angular vel: " << desired_velocity_filtered_ang_(0) << " " << desired_velocity_filtered_ang_(1) << " " << desired_velocity_filtered_ang_(2));
	ROS_INFO_STREAM( "--------------------------------------------------------");

	mutex_.unlock();

}


void CDDynamicsFilter::PublishDesiredVelocities() {

	// msg_desired_twist_filtered_.header.stamp    = ros::Time::now();
	msg_desired_twist_filtered_.linear.x  = desired_velocity_filtered_lin_(0);
	msg_desired_twist_filtered_.linear.y  = desired_velocity_filtered_lin_(1);
	msg_desired_twist_filtered_.linear.z  = desired_velocity_filtered_lin_(2);
	msg_desired_twist_filtered_.angular.x = desired_velocity_filtered_ang_(0);
	msg_desired_twist_filtered_.angular.y = desired_velocity_filtered_ang_(1);
	msg_desired_twist_filtered_.angular.z = desired_velocity_filtered_ang_(2);

	pub_desired_twist_filtered_.publish(msg_desired_twist_filtered_);

}




