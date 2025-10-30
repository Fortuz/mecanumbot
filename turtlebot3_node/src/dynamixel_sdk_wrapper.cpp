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
#include "rclcpp/rclcpp.hpp"
#include "turtlebot3_node/dynamixel_sdk_wrapper.hpp"

#include <algorithm>
#include <string>

using robotis::turtlebot3::DynamixelSDKWrapper;

DynamixelSDKWrapper::DynamixelSDKWrapper(const Device & device)
: device_(device)
{
  if (init_dynamixel_sdk_handlers() == false) {
    LOG_ERROR("DynamixelSDKWrapper", "Failed to initialize SDK handlers");
    return;
  } else {
    LOG_DEBUG("DynamixelSDKWrapper", "Success to initilize SDK handlers");
  }
}

DynamixelSDKWrapper::~DynamixelSDKWrapper()
{
  portHandler_->closePort();
}

bool DynamixelSDKWrapper::is_connected_to_device()
{
  // uint8_t data[2];
  // return this->read_register(device_.id, 0, 2, &data[0]);
  return this->ping(device_.id);
}

bool DynamixelSDKWrapper::ping(uint8_t id)
{
  uint16_t model_number = 0;
  uint8_t error = 0;

  int result = packetHandler_->ping(portHandler_, id, &model_number, &error);

  if (result == COMM_SUCCESS) {
    // RCLCPP_INFO(rclcpp::get_logger("DynamixelSDKWrapper"), "Ping succeeded: ID %d, Model: %d", id, model_number);
    return true;
  } else {
    // RCLCPP_ERROR(rclcpp::get_logger("DynamixelSDKWrapper"), "Ping failed: %s", packetHandler_->getTxRxResult(result));
    return false;
  }
}

void DynamixelSDKWrapper::init_read_memory(const uint16_t & start_addr, const uint16_t & length)
{
  read_memory_.start_addr = start_addr;
  read_memory_.length = length;
  read_memory_.data = &read_data_[0];
}

void DynamixelSDKWrapper::read_data_set() // TODO: The code robably fail here
{
  const char * log = NULL;
  bool ret = this->read_register(
    device_.id,
    read_memory_.start_addr,
    read_memory_.length,
    &read_data_buffer_[0],
    &log);
  //  LOG_INFO("DynamixelSDKWrapper", "Read data set called, ret=%d", ret);
  /*
  [turtlebot3_ros-3] [ERROR] [1756826600.476925401] [Device ID]: [200]
  [turtlebot3_ros-3] [ERROR] [1756826600.477010293] [Start address]: [10]
  [turtlebot3_ros-3] [ERROR] [1756826600.477039183] [Memory length]: [192]
  
  Thought: the OpenCRs memory is sorter than this propably? Too long to read back?
  */

  if (ret == false) {
    LOG_ERROR("Device ID", "[%d]", device_.id);
    LOG_ERROR("Start address", "[%d]", read_memory_.start_addr);
    LOG_ERROR("Memory length", "[%d]", read_memory_.length);
    LOG_ERROR("DynamixelSDKWrapper", "Failed to read[%s]", log);
  } 
  else {
    std::lock_guard<std::mutex> lock(read_data_mutex_);
    std::copy(
              read_data_buffer_,
              read_data_buffer_ + std::min(static_cast<size_t>(read_memory_.length), static_cast<size_t>(READ_DATA_SIZE)),
              read_data_);
    // LOG_INFO("DynamixelSDKWrapper", "Succeeded to read");
    LOG_DEBUG("DynamixelSDKWrapper", "Succeeded to read");
  }
}

bool DynamixelSDKWrapper::set_data_to_device(
  const uint16_t & addr,
  const uint16_t & length,
  uint8_t * get_data,
  std::string * msg)
{
  const char * log = nullptr;
  bool ret = false;

  std::lock_guard<std::mutex> lock(write_data_mutex_);
  ret = write_register(device_.id, addr, length, get_data, &log);

  if (ret == true) {
    *msg = "Succeeded to write data";
    return true;
  } else {
    *msg = "Failed to write data" + std::string(log);
    return false;
  }

  return ret;
}

bool DynamixelSDKWrapper::init_dynamixel_sdk_handlers()
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_.usb_port.c_str());
  packetHandler_ =
    dynamixel::PacketHandler::getPacketHandler(static_cast<int>(device_.protocol_version));

  if (portHandler_->openPort()) {
    LOG_INFO("DynamixelSDKWrapper", "Succeeded to open the port(%s)!", device_.usb_port.c_str());
  } else {
    LOG_ERROR("DynamixelSDKWrapper", "Failed to open the port(%s)!", device_.usb_port.c_str());
    return false;
  }

  if (portHandler_->setBaudRate(static_cast<int>(device_.baud_rate))) {
    LOG_INFO("DynamixelSDKWrapper", "Succeeded to change the baudrate!");
  } else {
    LOG_ERROR("DynamixelSDKWrapper", "Failed to change the baudrate(%d)!", device_.baud_rate);
    return false;
  }

  return true;
}

bool DynamixelSDKWrapper::read_register(
  uint8_t id,
  uint16_t address,
  uint16_t length,
  uint8_t * data_basket,
  const char ** log)
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);

  int32_t dxl_comm_result = COMM_RX_FAIL;
  uint8_t dxl_error = 0;

  dxl_comm_result = packetHandler_->readTxRx(
    portHandler_,
    id,
    address,
    length,
    data_basket,
    &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) 
  {
    RCLCPP_ERROR(rclcpp::get_logger("DynamixelSDKWrapper"), "Comm error, read register failed: %d", dxl_error);
    RCLCPP_ERROR(rclcpp::get_logger("DynamixelSDKWrapper"), "Error no.: %d", dxl_comm_result);


    if (log != NULL) 
    {
      *log = packetHandler_->getTxRxResult(dxl_comm_result);
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelSDKWrapper"), "Logs: %s", *log);
      
    }

    return false;
  }

  else if (dxl_error != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("DynamixelSDKWrapper"), "Comm success, but read register failed: %d", dxl_error);
    if (log != NULL) 
      {
      *log = packetHandler_->getRxPacketError(dxl_error);
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelSDKWrapper"), "Logs: %s", *log);
      }
      return false;
      
  }

  else {
    return true;
  }
  RCLCPP_ERROR(rclcpp::get_logger("DynamixelSDKWrapper"), "Weird subcase, how does this even happen? %d", dxl_error);
  return false;
}

bool DynamixelSDKWrapper::write_register(
  uint8_t id,
  uint16_t address,
  uint16_t length,
  uint8_t * data,
  const char ** log)
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);

  int32_t dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  dxl_comm_result = packetHandler_->writeTxRx(
    portHandler_,
    id,
    address,
    length,
    data,
    &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    if (log != NULL) {*log = packetHandler_->getTxRxResult(dxl_comm_result);}
    return false;
  } else if (dxl_error != 0) {
    if (log != NULL) {*log = packetHandler_->getRxPacketError(dxl_error);}
    return false;
  } else {
    return true;
  }

  return false;
}