// Copyright 2025 Enactic, Inc.
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

#include "openarm_hardware/fake_openarm_hw.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

FakeOpenArmV10HW::FakeOpenArmV10HW() = default;

bool FakeOpenArmV10HW::parse_config(
    const hardware_interface::HardwareInfo& info) {
  // Parse arm prefix (default: empty for single arm)
  auto it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse prefix (alias used in single-arm xacro)
  if (arm_prefix_.empty()) {
    it = info.hardware_parameters.find("prefix");
    if (it != info.hardware_parameters.end()) {
      arm_prefix_ = it->second;
    }
  }

  // Parse hand/gripper enable
  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;
  } else {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    hand_ = (value == "true");
  }

  // Parse control gains (same params as real hardware)
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    it = info.hardware_parameters.find("kp" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kp_[i - 1] = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kd_[i - 1] = std::stod(it->second);
    }
  }
  if (hand_) {
    it = info.hardware_parameters.find("kp_hand");
    if (it != info.hardware_parameters.end()) {
      gripper_kp_ = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd_hand");
    if (it != info.hardware_parameters.end()) {
      gripper_kd_ = std::stod(it->second);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("FakeOpenArmV10HW"),
              "Config: arm_prefix=%s, hand=%s", arm_prefix_.c_str(),
              hand_ ? "enabled" : "disabled");
  return true;
}

void FakeOpenArmV10HW::generate_joint_names() {
  joint_names_.clear();
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    joint_names_.push_back("openarm_" + arm_prefix_ + "joint" +
                           std::to_string(i));
  }
  if (hand_) {
    joint_names_.push_back("openarm_" + arm_prefix_ + "finger_joint1");
  }

  // Add prismatic rail joints if URDF contains them (underwater mode)
  bool has_rail_joints = false;
  for (const auto& j : info_.joints) {
    if (j.name == "Pjoint_X") { has_rail_joints = true; break; }
  }
  if (has_rail_joints) {
    joint_names_.push_back("Pjoint_X");
    joint_names_.push_back("Pjoint_Y");
    joint_names_.push_back("Pjoint_Z");
  }

  RCLCPP_INFO(rclcpp::get_logger("FakeOpenArmV10HW"),
              "Generated %zu joint names", joint_names_.size());
}

hardware_interface::CallbackReturn FakeOpenArmV10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }
  generate_joint_names();

  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("FakeOpenArmV10HW"),
              "Fake OpenArm V10 initialized successfully");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeOpenArmV10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("FakeOpenArmV10HW"),
              "Fake OpenArm V10 configured");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
FakeOpenArmV10HW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FakeOpenArmV10HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT,
        &tau_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn FakeOpenArmV10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("FakeOpenArmV10HW"),
              "Fake OpenArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeOpenArmV10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("FakeOpenArmV10HW"),
              "Fake OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FakeOpenArmV10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  // State already updated in write() via PD simulation
  (void)period;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FakeOpenArmV10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  // Compute dt in seconds
  const double dt = period.seconds();

  // Friction parameters (Coulomb + viscous, smoothed with tanh)
  // Joints 1-2 (J8009, big joints): higher friction
  // Joints 3-4 (J4340, mid size)
  // Joints 5-7 (J4310, small wrist joints): lower friction
  static const double coulomb_friction[ARM_DOF] = {0.3, 0.3, 0.15, 0.15, 0.05, 0.05, 0.05};
  static const double viscous_friction[ARM_DOF]  = {0.5, 0.5, 0.3, 0.3, 0.1, 0.1, 0.1};
  const double friction_smooth = 50.0;  // higher = sharper Coulomb transition

  // PD simulation for arm joints
  for (size_t i = 0; i < ARM_DOF; ++i) {
    // PD control: tau = Kp * (cmd_pos - cur_pos) + Kd * (cmd_vel - cur_vel)
    const double pos_error = pos_commands_[i] - pos_states_[i];
    const double vel_error = vel_commands_[i] - vel_states_[i];
    double effort = kp_[i] * pos_error + kd_[i] * vel_error;

    // smooth Coulomb + viscous friction (tanh to avoid chattering)
    const double vel = vel_states_[i];
    effort -= viscous_friction[i] * vel;
    effort -= coulomb_friction[i] * std::tanh(vel * friction_smooth);

    // Integrate: acc = effort (unit inertia), then semi-implicit Euler
    vel_states_[i] += effort * dt;
    pos_states_[i] += vel_states_[i] * dt;
    tau_states_[i] = effort;
  }

  // PD simulation for gripper joint
  if (hand_ && joint_names_.size() > ARM_DOF) {
    const size_t g = ARM_DOF;
    const double pos_error = pos_commands_[g] - pos_states_[g];
    const double vel_error = vel_commands_[g] - vel_states_[g];
    double effort = gripper_kp_ * pos_error + gripper_kd_ * vel_error;
    const double vel = vel_states_[g];
    effort -= 0.05 * vel;  // small viscous
    effort -= 0.02 * std::tanh(vel * friction_smooth);  // small smooth Coulomb
    vel_states_[g] += effort * dt;
    pos_states_[g] += vel_states_[g] * dt;
    tau_states_[g] = effort;
  }

  // Prismatic rail joints (Pjoint_X/Y/Z): direct pass-through
  const size_t base_count = ARM_DOF + (hand_ ? 1 : 0);
  for (size_t i = base_count; i < joint_names_.size(); ++i) {
    pos_states_[i] = pos_commands_[i];
    vel_states_[i] = vel_commands_[i];
    tau_states_[i] = 0.0;
  }

  // Clamp position to reasonable range to prevent runaway
  for (size_t i = 0; i < pos_states_.size(); ++i) {
    if (std::abs(pos_states_[i]) > 100.0) {
      pos_states_[i] = std::copysign(100.0, pos_states_[i]);
    }
    if (std::abs(vel_states_[i]) > 50.0) {
      vel_states_[i] = std::copysign(50.0, vel_states_[i]);
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::FakeOpenArmV10HW,
                       hardware_interface::SystemInterface)
