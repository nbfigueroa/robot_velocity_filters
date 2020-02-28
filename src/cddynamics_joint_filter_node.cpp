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

#include "ros/ros.h"
#include "CDDynamics_joint_filter.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cddynamics_joint_filter_node");

  ros::NodeHandle nh;
  double frequency = 150.0;

  // Parameters
  std::string input_velocity_topic_name;
  std::string output_velocity_topic_name;
  double joint_velocity_limit (1.5); // rad/s

  // Getting parameters from launch file or parameter server (uploaded through yaml file)
  if (!nh.getParam("input_velocity_topic_name", input_velocity_topic_name))   {
    ROS_ERROR("Couldn't retrieve the input velocity topic name. ");
    return -1;
  }

  if (!nh.getParam("output_velocity_topic_name", output_velocity_topic_name))   {
    ROS_ERROR("Couldn't retrieve the output velocity topic name. ");
    return -1;
  }

  if (!nh.getParam("joint_velocity_limit", joint_velocity_limit))   {
    ROS_ERROR("Couldn't retrieve the output velocity topic name. ");
    return -1;
  }

  ROS_INFO("Starting the CDDynamics joint velocity filter...");


  CDDynamicsJointFilter cddynamics_joint_filter(nh,
      frequency,
      input_velocity_topic_name,
      output_velocity_topic_name,
      joint_velocity_limit);

  if (!cddynamics_joint_filter.Init()) {
    return -1;
  }
  else {
    cddynamics_joint_filter.Run();
  }

  return 0;
}