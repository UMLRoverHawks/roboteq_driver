/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include "sample_acquisition/ArmMovement.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class TeleopJoy
{
public:
  TeleopJoy();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, pan_, tilt_, gripper_, arm_disengage_, deadman_axis_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Publisher arm_pub;
  ros::Publisher arm_engage_pub;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  ros::Timer timer_;

};

TeleopJoy::TeleopJoy():
  ph_("~"),
  linear_(1),
  angular_(0),
  pan_(2),
  tilt_(3),
  gripper_(5),
  arm_disengage_(10),
  deadman_axis_(4),
  l_scale_(0.3),
  a_scale_(0.9),
  deadman_pressed_(false)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_pan", pan_, pan_);
  ph_.param("axis_tilt", tilt_, tilt_);
  ph_.param("axis_gripper", gripper_, gripper_);
  ph_.param("button_disengage_arm", arm_disengage_, arm_disengage_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  arm_pub = nh_.advertise<sample_acquisition::ArmMovement>("/arm/movement", 1);
  arm_engage_pub = nh_.advertise<std_msgs::Bool>("/arm/on", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&TeleopJoy::publish, this));
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_msgs::Bool arm_engage_msg;
  sample_acquisition::ArmMovement arm_msg;
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
  if  (joy->axes[pan_] != 0 || joy->axes[tilt_] != 0 || joy->buttons[gripper_] !=0)
  {
	arm_engage_msg.data = true;
	arm_engage_pub.publish(arm_engage_msg);
	arm_msg.pan_motor_velocity =  joy->axes[pan_];
	arm_msg.tilt_motor_velocity = joy->axes[tilt_];
	arm_msg.gripper_open = (joy->axes[gripper_] <= 0);
  }
  last_published_ = vel;
  if (deadman_axis_ != -1)
    deadman_pressed_ = joy->buttons[deadman_axis_];
  else
    deadman_pressed_ = true;
   if(joy->buttons[arm_disengage_] == 1)
   {
	   arm_engage_msg.data = false;
	   arm_engage_pub.publish(arm_engage_msg);
	   sleep(1);
   }
   arm_pub.publish(arm_msg);
}

void TeleopJoy::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_);
  }

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  TeleopJoy t;

  ros::spin();
}
