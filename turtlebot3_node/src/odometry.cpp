// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#include "turtlebot3_node/odometry.hpp"

#include <memory>
#include <string>
#include <utility>

using robotis::turtlebot3::Odometry;
using namespace std::chrono_literals;

Odometry::Odometry(
    std::shared_ptr<rclcpp::Node> &nh,
    const double wheels_separation_x,
    const double wheels_separation_y,
    const double wheels_radius)
    : nh_(nh),
      wheels_separation_x_(wheels_separation_x),
      wheels_separation_y_(wheels_separation_y),
      wheels_radius_(wheels_radius),
      use_imu_(false),
      publish_tf_(false),
      last_theta_initialized_(false),
      last_theta_(0.0f),
      imu_angle_(0.0f),
      robot_pose_({0.0, 0.0, 0.0}),
      robot_vel_({0.0, 0.0, 0.0})
{
  RCLCPP_INFO(nh_->get_logger(), "Init Odometry");

  nh_->declare_parameter<std::string>("odometry.frame_id");
  nh_->declare_parameter<std::string>("odometry.child_frame_id");
  nh_->declare_parameter<std::string>("namespace");

  nh_->declare_parameter<bool>("odometry.use_imu");
  nh_->declare_parameter<bool>("odometry.publish_tf");

  nh_->get_parameter_or<bool>(
      "odometry.use_imu",
      use_imu_,
      false);

  nh_->get_parameter_or<bool>(
      "odometry.publish_tf",
      publish_tf_,
      false);

  nh_->get_parameter_or<std::string>(
      "odometry.frame_id",
      frame_id_of_odometry_,
      std::string("odom"));

  nh_->get_parameter_or<std::string>(
      "odometry.child_frame_id",
      child_frame_id_of_odometry_,
      std::string("base_footprint"));

  nh_->get_parameter_or<std::string>(
      "namespace",
      name_space_,
      std::string(""));

  if (name_space_ != "")
  {
    frame_id_of_odometry_ = name_space_ + "/" + frame_id_of_odometry_;
    child_frame_id_of_odometry_ = name_space_ + "/" + child_frame_id_of_odometry_;
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);

  if (use_imu_)
  {
    uint32_t queue_size = 10;
    joint_state_imu_sync_ = std::make_shared<SynchronizerJointStateImu>(queue_size);

    msg_ftr_joint_state_sub_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::msg::JointState>>(
            nh_,
            "joint_states");

    msg_ftr_imu_sub_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(
            nh_,
            "imu");

    // connect message filters to synchronizer
    joint_state_imu_sync_->connectInput(*msg_ftr_joint_state_sub_, *msg_ftr_imu_sub_);

    joint_state_imu_sync_->setInterMessageLowerBound(
        0,
        rclcpp::Duration(75ms));

    joint_state_imu_sync_->setInterMessageLowerBound(
        1,
        rclcpp::Duration(15ms));

    joint_state_imu_sync_->registerCallback(
        std::bind(
            &Odometry::joint_state_and_imu_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
  }
  else
  {
    joint_state_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        qos,
        std::bind(&Odometry::joint_state_callback, this, std::placeholders::_1));
  }
}

void Odometry::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
{
  const rclcpp::Time current_time = joint_state_msg->header.stamp;
  static rclcpp::Time last_time = current_time;
  const rclcpp::Duration duration = current_time - last_time;

  update_joint_state(joint_state_msg);
  calculate_odometry(duration);
  publish(current_time);

  last_time = current_time;
}

void Odometry::joint_state_and_imu_callback(
    const std::shared_ptr<sensor_msgs::msg::JointState const> &joint_state_msg,
    const std::shared_ptr<sensor_msgs::msg::Imu const> &imu_msg)
{
  RCLCPP_DEBUG(
      nh_->get_logger(),
      "[joint_state_msg_] nanosec : %d [imu_msg] nanosec : %d",
      joint_state_msg->header.stamp.nanosec,
      imu_msg->header.stamp.nanosec);

  const rclcpp::Time current_time = joint_state_msg->header.stamp;
  static rclcpp::Time last_time = current_time;
  const rclcpp::Duration duration = current_time - last_time;

  update_joint_state(joint_state_msg);
  update_imu(imu_msg);
  calculate_odometry(duration);
  publish(current_time);

  last_time = current_time;
}

void Odometry::publish(const rclcpp::Time &now)
{
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  odom_msg->header.frame_id = frame_id_of_odometry_;
  odom_msg->child_frame_id = child_frame_id_of_odometry_;
  odom_msg->header.stamp = now;

  odom_msg->pose.pose.position.x = robot_pose_[0];
  odom_msg->pose.pose.position.y = robot_pose_[1];
  odom_msg->pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, robot_pose_[2]);

  odom_msg->pose.pose.orientation.x = q.x();
  odom_msg->pose.pose.orientation.y = q.y();
  odom_msg->pose.pose.orientation.z = q.z();
  odom_msg->pose.pose.orientation.w = q.w();

  odom_msg->twist.twist.linear.x = robot_vel_[0];
  odom_msg->twist.twist.linear.y = robot_vel_[1];
  odom_msg->twist.twist.angular.z = robot_vel_[2];

  // TODO(Will Son): Find more accurate covariance.
  // odom_msg->pose.covariance[0] = 0.05;
  // odom_msg->pose.covariance[7] = 0.05;
  // odom_msg->pose.covariance[14] = 1.0e-9;
  // odom_msg->pose.covariance[21] = 1.0e-9;
  // odom_msg->pose.covariance[28] = 1.0e-9;
  // odom_msg->pose.covariance[35] = 0.0872665;

  // odom_msg->twist.covariance[0] = 0.001;
  // odom_msg->twist.covariance[7] = 1.0e-9;
  // odom_msg->twist.covariance[14] = 1.0e-9;
  // odom_msg->twist.covariance[21] = 1.0e-9;
  // odom_msg->twist.covariance[28] = 1.0e-9;
  // odom_msg->twist.covariance[35] = 0.001;

  geometry_msgs::msg::TransformStamped odom_tf;

  odom_tf.transform.translation.x = odom_msg->pose.pose.position.x;
  odom_tf.transform.translation.y = odom_msg->pose.pose.position.y;
  odom_tf.transform.translation.z = odom_msg->pose.pose.position.z;
  odom_tf.transform.rotation = odom_msg->pose.pose.orientation;

  odom_tf.header.frame_id = frame_id_of_odometry_;
  odom_tf.child_frame_id = child_frame_id_of_odometry_;
  odom_tf.header.stamp = now;

  odom_pub_->publish(std::move(odom_msg));

  if (publish_tf_)
  {
    tf_broadcaster_->sendTransform(odom_tf);
  }
}

void Odometry::update_joint_state(
    const std::shared_ptr<sensor_msgs::msg::JointState const> &joint_state)
{
  int idx_fl = std::find(joint_state->name.begin(), joint_state->name.end(), "wheel_frontleft_joint") - joint_state->name.begin();
  int idx_fr = std::find(joint_state->name.begin(), joint_state->name.end(), "wheel_frontright_joint") - joint_state->name.begin();
  int idx_rl = std::find(joint_state->name.begin(), joint_state->name.end(), "wheel_backleft_joint") - joint_state->name.begin();
  int idx_rr = std::find(joint_state->name.begin(), joint_state->name.end(), "wheel_backright_joint") - joint_state->name.begin();
  RCLCPP_INFO(nh_->get_logger(), "Wheel indices - FL: %d, FR: %d, RL: %d, RR: %d", idx_fl, idx_fr, idx_rl, idx_rr);
  static std::array<double, 2> last_joint_positions = {0.0f, 0.0f};

  diff_joint_positions_[0] = joint_state->position[0] - last_joint_positions[0]; //FL;3
  diff_joint_positions_[1] = joint_state->position[1] - last_joint_positions[1]; //FR;4
  diff_joint_positions_[2] = joint_state->position[2] - last_joint_positions[2]; //RL;1
  diff_joint_positions_[3] = joint_state->position[3] - last_joint_positions[3]; //RR;2

  last_joint_positions[0] = joint_state->position[0]; //FL
  last_joint_positions[1] = joint_state->position[1]; //FR
  last_joint_positions[2] = joint_state->position[2]; //RL
  last_joint_positions[3] = joint_state->position[3]; //RR
}

void Odometry::update_imu(const std::shared_ptr<sensor_msgs::msg::Imu const> &imu)
{
  imu_angle_ = atan2f(
      imu->orientation.x * imu->orientation.y + imu->orientation.w * imu->orientation.z,
      0.5f - imu->orientation.y * imu->orientation.y - imu->orientation.z * imu->orientation.z);
}

bool Odometry::calculate_odometry(const rclcpp::Duration &duration)
{

  // rotation value of wheel [rad]
  double wheel_fl = diff_joint_positions_[3]; //FL;3
  double wheel_fr = diff_joint_positions_[4]; //FR;4
  double wheel_rl = diff_joint_positions_[1]; //RL;1 EDITED
  double wheel_rr = diff_joint_positions_[2]; //RR;2 EDITED

  double delta_x = 0.0;
  double delta_y = 0.0;
  double delta_theta = 0.0;

  double theta = 0.0;

  // v = translational velocity [m/s]
  // w = rotational velocity [rad/s]
  double v_x = 0.0;
  double v_y = 0.0;
  double w = 0.0;

  double step_time = duration.seconds();

 if (step_time < 1e-5) {
  RCLCPP_WARN(nh_->get_logger(), "Step time too small: %f seconds. Skipping odometry update.", step_time);
  return false;
}

  if (std::isnan(wheel_fl))
  {
    wheel_fl = 0.0;
  }

  if (std::isnan(wheel_fr))
  {
    wheel_fr = 0.0;
  }

  if (std::isnan(wheel_rl))
  {
    wheel_rl = 0.0;
  }

  if (std::isnan(wheel_rr))
  {
    wheel_rr = 0.0;
  }

  delta_x = wheels_radius_ * (wheel_fl + wheel_fr + wheel_rl + wheel_rr) / 4.0;
  delta_y = wheels_radius_ * (-wheel_fl + wheel_fr - wheel_rl + wheel_rr) / 4.0;

  if (use_imu_)
  {
    if (last_theta_initialized_)
    {
    theta = imu_angle_;
    delta_theta = theta - last_theta_;     
   RCLCPP_INFO(nh_->get_logger(),
      "Odometry, IMU based, last_theta:%f, theta:%f, delta_theta:%f",
      last_theta_,theta,delta_theta);
  }
    else
    {
      theta = imu_angle_;
      last_theta_ = imu_angle_;
      delta_theta = theta - last_theta_;
      last_theta_initialized_ = true;
      RCLCPP_INFO(nh_->get_logger(), "Odometry, IMU based, delta_theta:%f, last_theta:%f",delta_theta,last_theta_);
    }
  }
  else
  {
  theta = wheels_radius_ * (-wheel_fl + wheel_fr + wheel_rl - wheel_rr) / (2 * (wheels_separation_y_ + wheels_separation_x_));
  delta_theta = theta;
 // RCLCPP_INFO(nh_->get_logger(), "Odometry, calculated delta_theta : %f", delta_theta);
  }

  // compute odometric pose
  robot_pose_[0] += delta_x * cos(robot_pose_[2] + (delta_theta / 2.0)) - delta_y * sin(robot_pose_[2] + (delta_theta / 2.0));
  robot_pose_[1] += delta_x * sin(robot_pose_[2] + (delta_theta / 2.0)) + delta_y * cos(robot_pose_[2] + (delta_theta / 2.0));
  robot_pose_[2] += delta_theta;

  RCLCPP_DEBUG(nh_->get_logger(), "x : %f, y : %f", robot_pose_[0], robot_pose_[1]);

  // compute odometric instantaneouse velocity
  v_x = delta_x / step_time;
  v_y = delta_y / step_time;
  w = delta_theta / step_time;

  robot_vel_[0] = v_x;
  robot_vel_[1] = v_y;
  robot_vel_[2] = w;

  if (!std::isfinite(robot_pose_[0]) ||
    !std::isfinite(robot_pose_[1]) ||
    !std::isfinite(robot_pose_[2]) ||
    !std::isfinite(robot_vel_[0]) ||
    !std::isfinite(robot_vel_[1]) ||
    !std::isfinite(robot_vel_[2])) {
  RCLCPP_WARN(nh_->get_logger(), "Skipping odometry publish due to NaN or Inf values.");
  return;
  }
  last_theta_ = theta;
  return true;
}
