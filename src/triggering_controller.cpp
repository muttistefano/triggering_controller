// Copyright 2021 ros2_control development team
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

#include "triggering_controller/triggering_controller.hpp"

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace triggering_controller
{

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

TriggeringController::TriggeringController() {}

controller_interface::CallbackReturn TriggeringController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TriggeringController::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration TriggeringController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;


  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : params_.joints)
  {
    for (const auto & interface : params_.interfaces)
    {
      state_interfaces_config.names.push_back(joint + "/" + interface);
    }
  }
  

  return state_interfaces_config;
}

controller_interface::CallbackReturn TriggeringController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  std::string _robot_description_msg = "";
  #ifdef TEST_GAZEBO
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "TESTING WITH GAZEBO BUILD " );
    #pragma message("TESTING WITH GAZEBO BUILD ")
    #include<triggering_controller/test/ur_urdf.hpp>
    _robot_description_msg = robot_urdf;
  #else
    auto robot_sub = get_node()->create_subscription<std_msgs::msg::String>(
      params_.robot_description_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [&_robot_description_msg,this](std_msgs::msg::String::SharedPtr msg) { _robot_description_msg = msg->data; });

    while(_robot_description_msg == "")
    {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "_robot_description_msg " << _robot_description_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));  
    }
  #endif

  kdl_parser::treeFromString(_robot_description_msg,_kdl_tree);
  _kdl_tree.getChain(params_.robot_chain_root,params_.robot_chain_tip,_kdl_chain_robot);
  _jnt_to_pose_solver_robot.reset(new KDL::ChainFkSolverPos_recursive(_kdl_chain_robot));
  _q_robot.resize(_kdl_chain_robot.getNrOfJoints());


  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TriggeringController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!init_joint_data())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "None of requested interfaces exist. Controller will not run.");
    return CallbackReturn::ERROR;
  }

  for (const auto & state_interface : state_interfaces_)
  {
    std::string interface_name = state_interface.get_interface_name();
    name_if_value_mapping_[state_interface.get_prefix_name()][interface_name] =
      state_interface.get_value();
    RCLCPP_DEBUG(
      get_node()->get_logger(), "%s: %f\n", state_interface.get_name().c_str(),
      state_interface.get_value());
  }

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    _q_robot.data(i) = get_value(name_if_value_mapping_, joint_names_[i], HW_IF_POSITION);

  }

  _jnt_to_pose_solver_robot->JntToCart(_q_robot,_fk_robot_stored);
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "initial pose :  " << _fk_robot_stored.p(0) << " " <<  _fk_robot_stored.p(1) << " " << _fk_robot_stored.p(2) );

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TriggeringController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_.clear();

  return CallbackReturn::SUCCESS;
}

template <typename T>
bool has_any_key(
  const std::unordered_map<std::string, T> & map, const std::vector<std::string> & keys)
{
  bool found_key = false;
  for (const auto & key_item : map)
  {
    const auto & key = key_item.first;
    if (std::find(keys.cbegin(), keys.cend(), key) != keys.cend())
    {
      found_key = true;
      break;
    }
  }
  return found_key;
}

bool TriggeringController::init_joint_data()
{
  joint_names_.clear();
  if (state_interfaces_.empty())
  {
    return false;
  }

  // loop in reverse order, this maintains the order of values at retrieval time
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++)
  {
    // initialize map if name is new
    if (name_if_value_mapping_.count(si->get_prefix_name()) == 0)
    {
      name_if_value_mapping_[si->get_prefix_name()] = {};
    }
    // add interface name
    std::string interface_name = si->get_interface_name();
    name_if_value_mapping_[si->get_prefix_name()][interface_name] = kUninitializedValue;
  }

  // filter state interfaces that have at least one of the joint_states fields,
  // the rest will be ignored for this message
  for (const auto & name_ifv : name_if_value_mapping_)
  {
    const auto & interfaces_and_values = name_ifv.second;
    if (has_any_key(interfaces_and_values, {HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT}))
    {
      joint_names_.push_back(name_ifv.first);
    }
  }



  return true;
}




controller_interface::return_type TriggeringController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  for (const auto & state_interface : state_interfaces_)
  {
    std::string interface_name = state_interface.get_interface_name();
    name_if_value_mapping_[state_interface.get_prefix_name()][interface_name] =
      state_interface.get_value();
    RCLCPP_DEBUG(
      get_node()->get_logger(), "%s: %f\n", state_interface.get_name().c_str(),
      state_interface.get_value());
  }

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    _q_robot.data(i) = get_value(name_if_value_mapping_, joint_names_[i], HW_IF_POSITION);

  }

  _jnt_to_pose_solver_robot->JntToCart(_q_robot,_fk_robot_actual);

  // RCLCPP_INFO_STREAM(get_node()->get_logger(),fk_dist(_fk_robot_stored,_fk_robot_actual));
  if (fk_dist(_fk_robot_stored,_fk_robot_actual) > params_.trigger_distance)
  {
    RCLCPP_INFO_STREAM(get_node()->get_logger(),fk_dist(_fk_robot_stored,_fk_robot_actual));
    _fk_robot_stored = _fk_robot_actual;
    
  }


  return controller_interface::return_type::OK;
}

}  // namespace triggering_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  triggering_controller::TriggeringController, controller_interface::ControllerInterface)
