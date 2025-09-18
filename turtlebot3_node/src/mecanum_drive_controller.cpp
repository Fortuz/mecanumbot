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

#include "turtlebot3_node/mecanum_drive_controller.hpp"

#include <memory>

using robotis::turtlebot3::MecanumDriveController;

MecanumDriveController::MecanumDriveController(const float wheel_separation_x,const float wheel_seperation_y, const float wheel_radius)
: Node("mecanum_drive_controller", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  odometry_ = std::make_unique<Odometry>(
    nh_,
    wheel_separation_x,
    wheel_seperation_y,
    wheel_radius);

  RCLCPP_INFO(this->get_logger(), "Run!");
}
