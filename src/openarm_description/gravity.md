背景：为什么要给 OpenArm 添加重力补偿

在使用 OpenArm 真机进行 MoveIt/RViz 控制时，机械臂可以正常规划和执行轨迹。但在特定姿态下（如抬臂、前伸、悬停），会出现负载感明显的问题：

    末端有下坠趋势。
    电机维持姿态时声音变重。
    关节感觉像是在依靠位置环硬撑。

问题根源： 控制系统 主要依赖位置闭环和阻尼项维持姿态，缺乏显式的重力前馈力矩。7自由度机械臂在不同姿态下，各关节需抵消的重力力矩G(q) 会变化。若硬件接口仅下发位置、速度和控制器输出力矩，而未计算当前关节角
𝑞
对应的
𝐺(𝑞)
，机械臂在高负载姿态下会更依赖位置环增益 kp 和阻尼项 kd 来维持，导致上述现象。

目标修改： 在原有 ros2_control 硬件接口中添加重力补偿前馈项： \tau_{\text{final}} = \tau_{\text{command}} + \alpha \cdot G(q)即：ff_tau = tau_commands_[i] + gravity_scale_ * gravity_ff_[i];

    保持原有 MoveIt、JointTrajectoryController、MIT 控制流程不变。
    仅在最终下发电机力矩前叠加补偿项。

问题原因：原始硬件接口没有计算重力项

OpenArm 真机控制链路：

    MoveIt / RViz
    ↓
    JointTrajectoryController
    ↓
    ros2_control
    ↓
    OpenArm_v10HW::write()
    ↓
    openarm_->get_arm().mit_control_all(...)
    ↓
    Damiao motor

    所有控制指令最终都经过 openarm_hardware 中的 OpenArm_v10HW::write()。
    核心问题： 原始 write() 函数仅将 tau_commands_[i] 原样下发，未计算当前姿态下的重力项 G(q) 。

修改位置合理性： 在 write() 函数中判断重力补偿是否启用，若启用则调用动力学计算接口获取 G(q) ，并叠加到最终下发的力矩上。此位置是硬件接口的最后下发环节，修改后可覆盖 MoveIt 和 ros2_control 的正常流程。 物理学
解决思路：利用 URDF + KDL 计算 G(q)

关键： 计算当前关节角 q 下每个关节需抵消的重力矩
𝐺(𝑞)
。

方案： 使用 KDL (Kinematica and Dynamics Library) 实现。

    KDL 可用树结构表示 机器人机构的运动学/动力学参数。
    kdl_parser 可从 URDF 模型构造 KDL Tree。

流程：

    展开后的 URDF
    ↓
    kdl_parser::treeFromUrdfModel(...)
    ↓
    KDL::Tree
    ↓
    getChain(gravity_base_link, gravity_tip_link)
    ↓
    KDL::Chain
    ↓
    KDL::ChainDynParam
    ↓
    JntToGravity(q, gravity_forces)

OpenArm 双臂 KDL 链选取：

    左臂： openarm_body_link0 -> openarm_left_link7
    右臂： openarm_body_link0 -> openarm_right_link7

实现步骤总览

    新增 Dynamics 动力学求解类： 负责封装利用 KDL 计算 G(q) 的功能。
    修改 v10_simple_hardware.hpp： 增加重力补偿相关的成员变量（如 gravity_compensation_、gravity_scale_、gravity_ff_ 数组，指向 Dynamics 类的指针）。
    修改 v10_simple_hardware.cpp：
        读取参数： 在 on_init 或 configure 中解析重力补偿开关状态、缩放因子 \alpha、基础连杆、末端连杆等参数。
        初始化 Dynamics： 在 on_init 或 configure 中，根据传入的 URDF 模型（或路径）和指定的基础/末端连杆，初始化 Dynamics 类实例。
        在 write() 中叠加重力前馈：
            读取当前关节位置 pos_states_。
            若重力补偿启用，调用 dynamics_->GetGravity(pos_states_.data(), gravity_ff_.data()) 计算 G(q)。
            计算最终力矩 ff_tau = tau_commands_[i] + gravity_scale_ * gravity_ff_[i]。
            使用 ff_tau 代替原来的 tau_commands_[i] 进行下发（如调用 mit_control_all）。
    修改 CMakeLists.txt 和 package.xml： 添加对 kdl_parser、urdf、Eigen (KDL 依赖) 库的依赖。
    修改 openarm.bimanual.ros2_control.xacro： 给左右臂的硬件接口参数配置添加重力补偿相关参数（如 use_gravity_compensation、gravity_scale、gravity_base_link、gravity_tip_link）。
    启动前处理： 在启动机器人前，确保已将包含重力补偿参数的 xacro 文件展开为最终的 URDF 文件（例如 /tmp/openarm_bimanual.urdf），供 kdl_parser 使用。

修改一：新增 Dynamics 动力学求解 类
1.1 dynamics.hpp

新增文件：

~/ros2_ws/src/openarm_ros2/openarm_hardware/include/openarm_hardware/dynamics.hpp

完整代码如下：

    #pragma once
     
    #include <fstream>
    #include <memory>
    #include <sstream>
    #include <string>
     
    #include <Eigen/Dense>
    #include <kdl/chain.hpp>
    #include <kdl/chaindynparam.hpp>
    #include <kdl/chainfksolverpos_recursive.hpp>
    #include <kdl/chainjnttojacsolver.hpp>
    #include <kdl/jacobian.hpp>
    #include <kdl/jntarray.hpp>
    #include <kdl/tree.hpp>
    #include <kdl_parser/kdl_parser.hpp>
    #include <urdf/model.h>
     
    class Dynamics {
     public:
      Dynamics(std::string urdf_path, std::string start_link, std::string end_link);
      ~Dynamics();
     
      bool Init();
     
      void GetGravity(const double* joint_position, double* gravity);
     
      void GetCoriolis(
          const double* joint_position,
          const double* joint_velocity,
          double* coriolis);
     
      void GetMassMatrixDiagonal(
          const double* joint_position,
          double* inertia_diag);
     
      void GetJacobian(
          const double* joint_position,
          Eigen::MatrixXd& jacobian);
     
     private:
      urdf::Model urdf_model_;
      std::string urdf_path_;
      std::string start_link_;
      std::string end_link_;
     
      KDL::JntSpaceInertiaMatrix inertia_matrix_;
      KDL::JntArray coriolis_forces_;
      KDL::JntArray gravity_forces_;
      KDL::Tree kdl_tree_;
      KDL::Chain kdl_chain_;
      std::unique_ptr<KDL::ChainDynParam> solver_;
    };

bash

在调试过程中，一个关键点：最初尝试使用 urdf::parseURDF(buffer.str()) 时，在当前 ROS Humble 环境下编译失败，报错信息明确显示 parseURDF is not a member of urdf 以及 invalid use of incomplete type 'class Dynamics'。这两类错误成为了定位问题的关键线索。最终发现，改用以下方式即可正常编译： 编程

    urdf::Model urdf_model_;
    urdf_model_.initString(buffer.str());

cpp
运行

1.2 dynamics.cpp

新增文件：

~/ros2_ws/src/openarm_ros2/openarm_hardware/src/dynamics.cpp
bash

完整代码如下：

    #include "openarm_hardware/dynamics.hpp"
     
    #include <iostream>
    #include <memory>
    #include <utility>
     
    Dynamics::Dynamics(
        std::string urdf_path,
        std::string start_link,
        std::string end_link)
        : urdf_path_(std::move(urdf_path)),
          start_link_(std::move(start_link)),
          end_link_(std::move(end_link)) {}
     
    Dynamics::~Dynamics() = default;
     
    bool Dynamics::Init() {
      std::ifstream file(urdf_path_);
      if (!file.is_open()) {
        std::cerr << "Failed to open URDF file: " << urdf_path_ << std::endl;
        return false;
      }
      std::stringstream buffer;
      buffer << file.rdbuf();
      file.close();
      if (!urdf_model_.initString(buffer.str())) {
        std::cerr << "Failed to parse URDF: " << urdf_path_ << std::endl;
        return false;
      }
      if (!kdl_parser::treeFromUrdfModel(urdf_model_, kdl_tree_)) {
        std::cerr << "Failed to extract KDL tree" << std::endl;
        return false;
      }
      if (!kdl_tree_.getChain(start_link_, end_link_, kdl_chain_)) {
        std::cerr << "Failed to get KDL chain: "
                  << start_link_ << " -> " << end_link_ << std::endl;
        return false;
      }
      const auto n = kdl_chain_.getNrOfJoints();
      coriolis_forces_.resize(n);
      gravity_forces_.resize(n);
      inertia_matrix_.resize(n);
      coriolis_forces_.data.setZero();
      gravity_forces_.data.setZero();
      inertia_matrix_.data.setZero();
      solver_ = std::make_unique<KDL::ChainDynParam>(
          kdl_chain_,
          KDL::Vector(0.0, 0.0, -9.81));
      return true;
    }
    void Dynamics::GetGravity(const double* joint_position, double* gravity) {
      KDL::JntArray q(kdl_chain_.getNrOfJoints());
      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
        q(i) = joint_position[i];
      }
      solver_->JntToGravity(q, gravity_forces_);
      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
        gravity[i] = gravity_forces_(i);
      }
    }
    void Dynamics::GetCoriolis(
        const double* joint_position,
        const double* joint_velocity,
        double* coriolis) {
      KDL::JntArray q(kdl_chain_.getNrOfJoints());
      KDL::JntArray q_dot(kdl_chain_.getNrOfJoints());
      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
        q(i) = joint_position[i];
        q_dot(i) = joint_velocity[i];
      }
      solver_->JntToCoriolis(q, q_dot, coriolis_forces_);
      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
        coriolis[i] = coriolis_forces_(i);
      }
    }
    void Dynamics::GetMassMatrixDiagonal(
        const double* joint_position,
        double* inertia_diag) {
      KDL::JntArray q(kdl_chain_.getNrOfJoints());
      KDL::JntSpaceInertiaMatrix mass(kdl_chain_.getNrOfJoints());
      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
        q(i) = joint_position[i];
      }
      solver_->JntToMass(q, mass);
      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
        inertia_diag[i] = mass(i, i);
      }
    }
    void Dynamics::GetJacobian(
        const double* joint_position,
        Eigen::MatrixXd& jacobian) {
      KDL::JntArray q(kdl_chain_.getNrOfJoints());
      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
        q(i) = joint_position[i];
      }
      KDL::Jacobian kdl_jac(kdl_chain_.getNrOfJoints());
      KDL::ChainJntToJacSolver jac_solver(kdl_chain_);
      jac_solver.JntToJac(q, kdl_jac);
      jacobian = Eigen::MatrixXd(6, kdl_chain_.getNrOfJoints());
      for (int i = 0; i < 6; ++i) {
        for (unsigned int j = 0; j < kdl_chain_.getNrOfJoints(); ++j) {
          jacobian(i, j) = kdl_jac(i, j);
        }
      }
    }

bash

该类的核心功能只有一个：GetGravity()。其他方法如GetCoriolis()、GetMassMatrixDiagonal()、GetJacobian()是为后续扩展预留的接口，例如未来可能添加惯量补偿、雅可比速度估计或末端力控制等功能。

该类的核心功能只有一个：GetGravity()。其他方法如GetCoriolis()、GetMassMatrixDiagonal()、GetJacobian()是为后续扩展预留的接口，例如未来可能添加惯量补偿、雅可比速度估计或末端力控制等功能。
修改二：v10_simple_hardware.hpp

在：

~/ros2_ws/src/openarm_ros2/openarm_hardware/include/openarm_hardware/v10_simple_hardware.hpp
bash

中加入：

#include "openarm_hardware/dynamics.hpp"
bash

并新增这些成员变量：

    bool gravity_compensation_{false};
    double gravity_scale_{1.0};
    std::string gravity_urdf_path_;
    std::string gravity_base_link_;
    std::string gravity_tip_link_;
     
    std::unique_ptr<Dynamics> dynamics_;
    std::vector<double> gravity_ff_;

bash

 

这里有一个关键点需要注意：仅使用 class Dynamics; 进行前向声明是不够的。因为在 v10_simple_hardware.cpp 文件中，实际使用了 std::make_unique<Dynamics>()、dynamics_->Init() 和 dynamics_->GetGravity() 等操作，这些都需要 Dynamics 类型的完整定义。此前编译日志中出现的 invalid use of incomplete type 'class Dynamics' 错误，正是由于最初仅做了前向声明导致的。 计算机硬件
修改三：v10_simple_hardware.cpp

文件路径 ：

~/ros2_ws/src/openarm_ros2/openarm_hardware/src/v10_simple_hardware.cpp
bash

具体代码：

    #include "openarm_hardware/v10_simple_hardware.hpp"
     
    #include <algorithm>
    #include <cctype>
    #include <chrono>
    #include <thread>
    #include <vector>
     
    #include "hardware_interface/types/hardware_interface_type_values.hpp"
    #include "rclcpp/logging.hpp"
    #include "rclcpp/rclcpp.hpp"
     
    namespace openarm_hardware {
     
    OpenArm_v10HW::OpenArm_v10HW() = default;
     
    bool OpenArm_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
      auto it = info.hardware_parameters.find("can_interface");
      can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";
     
      it = info.hardware_parameters.find("arm_prefix");
      arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";
     
      it = info.hardware_parameters.find("hand");
      if (it == info.hardware_parameters.end()) {
        hand_ = true;
      } else {
        std::string value = it->second;
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);
        hand_ = (value == "true");
      }
     
      it = info.hardware_parameters.find("can_fd");
      if (it == info.hardware_parameters.end()) {
        can_fd_ = true;
      } else {
        std::string value = it->second;
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);
        can_fd_ = (value == "true");
      }
     
      it = info.hardware_parameters.find("gravity_compensation");
      if (it != info.hardware_parameters.end()) {
        std::string value = it->second;
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);
        gravity_compensation_ = (value == "true");
      }
     
      it = info.hardware_parameters.find("gravity_scale");
      if (it != info.hardware_parameters.end()) {
        gravity_scale_ = std::stod(it->second);
      }
     
      it = info.hardware_parameters.find("gravity_urdf_path");
      gravity_urdf_path_ = (it != info.hardware_parameters.end()) ? it->second : "";
     
      it = info.hardware_parameters.find("gravity_base_link");
      gravity_base_link_ = (it != info.hardware_parameters.end()) ? it->second : "";
     
      it = info.hardware_parameters.find("gravity_tip_link");
      gravity_tip_link_ = (it != info.hardware_parameters.end()) ? it->second : "";
     
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
     
      RCLCPP_INFO(
          rclcpp::get_logger("OpenArm_v10HW"),
          "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s, gravity_comp=%s, gravity_scale=%.3f",
          can_interface_.c_str(),
          arm_prefix_.c_str(),
          hand_ ? "enabled" : "disabled",
          can_fd_ ? "enabled" : "disabled",
          gravity_compensation_ ? "enabled" : "disabled",
          gravity_scale_);
     
      return true;
    }
     
    void OpenArm_v10HW::generate_joint_names() {
      joint_names_.clear();
     
      for (size_t i = 1; i <= ARM_DOF; ++i) {
        joint_names_.push_back("openarm_" + arm_prefix_ + "joint" + std::to_string(i));
      }
     
      if (hand_) {
        joint_names_.push_back("openarm_" + arm_prefix_ + "finger_joint1");
      }
    }
     
    hardware_interface::CallbackReturn OpenArm_v10HW::on_init(
        const hardware_interface::HardwareInfo& info) {
      if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
      }
     
      if (!parse_config(info)) {
        return CallbackReturn::ERROR;
      }
     
      generate_joint_names();
     
      const size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
      if (joint_names_.size() != expected_joints) {
        RCLCPP_ERROR(
            rclcpp::get_logger("OpenArm_v10HW"),
            "Generated %zu joint names, expected %zu",
            joint_names_.size(),
            expected_joints);
        return CallbackReturn::ERROR;
      }
     
      openarm_ = std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);
      openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS, DEFAULT_RECV_CAN_IDS);
     
      if (hand_) {
        openarm_->init_gripper_motor(
            DEFAULT_GRIPPER_MOTOR_TYPE,
            DEFAULT_GRIPPER_SEND_CAN_ID,
            DEFAULT_GRIPPER_RECV_CAN_ID);
      }
     
      pos_commands_.assign(expected_joints, 0.0);
      vel_commands_.assign(expected_joints, 0.0);
      tau_commands_.assign(expected_joints, 0.0);
      pos_states_.assign(expected_joints, 0.0);
      vel_states_.assign(expected_joints, 0.0);
      tau_states_.assign(expected_joints, 0.0);
      gravity_ff_.assign(ARM_DOF, 0.0);
     
      if (gravity_compensation_) {
        if (gravity_urdf_path_.empty() || gravity_base_link_.empty() || gravity_tip_link_.empty()) {
          RCLCPP_ERROR(
              rclcpp::get_logger("OpenArm_v10HW"),
              "gravity_compensation=true but gravity_urdf_path / gravity_base_link / gravity_tip_link not fully set");
          return CallbackReturn::ERROR;
        }
     
        dynamics_ = std::make_unique<Dynamics>(
            gravity_urdf_path_,
            gravity_base_link_,
            gravity_tip_link_);
     
        if (!dynamics_->Init()) {
          RCLCPP_ERROR(
              rclcpp::get_logger("OpenArm_v10HW"),
              "Failed to initialize gravity dynamics model");
          return CallbackReturn::ERROR;
        }
      }
     
      RCLCPP_INFO(
          rclcpp::get_logger("OpenArm_v10HW"),
          "OpenArm V10 Simple HW initialized successfully");
     
      return CallbackReturn::SUCCESS;
    }
     
    hardware_interface::CallbackReturn OpenArm_v10HW::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/) {
      openarm_->refresh_all();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      openarm_->recv_all();
      return CallbackReturn::SUCCESS;
    }
     
    std::vector<hardware_interface::StateInterface>
    OpenArm_v10HW::export_state_interfaces() {
      std::vector<hardware_interface::StateInterface> state_interfaces;
     
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        state_interfaces.emplace_back(
            joint_names_[i],
            hardware_interface::HW_IF_POSITION,
            &pos_states_[i]);
     
        state_interfaces.emplace_back(
            joint_names_[i],
            hardware_interface::HW_IF_VELOCITY,
            &vel_states_[i]);
     
        state_interfaces.emplace_back(
            joint_names_[i],
            hardware_interface::HW_IF_EFFORT,
            &tau_states_[i]);
      }
     
      return state_interfaces;
    }
     
    std::vector<hardware_interface::CommandInterface>
    OpenArm_v10HW::export_command_interfaces() {
      std::vector<hardware_interface::CommandInterface> command_interfaces;
     
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces.emplace_back(
            joint_names_[i],
            hardware_interface::HW_IF_POSITION,
            &pos_commands_[i]);
     
        command_interfaces.emplace_back(
            joint_names_[i],
            hardware_interface::HW_IF_VELOCITY,
            &vel_commands_[i]);
     
        command_interfaces.emplace_back(
            joint_names_[i],
            hardware_interface::HW_IF_EFFORT,
            &tau_commands_[i]);
      }
     
      return command_interfaces;
    }
     
    hardware_interface::CallbackReturn OpenArm_v10HW::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Activating OpenArm V10...");
     
      openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
      openarm_->enable_all();
     
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      openarm_->recv_all();
     
      return_to_zero();
     
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 activated");
      return CallbackReturn::SUCCESS;
    }
     
    hardware_interface::CallbackReturn OpenArm_v10HW::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Deactivating OpenArm V10...");
     
      openarm_->disable_all();
     
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      openarm_->recv_all();
     
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 deactivated");
      return CallbackReturn::SUCCESS;
    }
     
    hardware_interface::return_type OpenArm_v10HW::read(
        const rclcpp::Time& /*time*/,
        const rclcpp::Duration& /*period*/) {
      openarm_->refresh_all();
      openarm_->recv_all();
     
      const auto& arm_motors = openarm_->get_arm().get_motors();
     
      for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
        pos_states_[i] = arm_motors[i].get_position();
        vel_states_[i] = arm_motors[i].get_velocity();
        tau_states_[i] = arm_motors[i].get_torque();
      }
     
      if (hand_ && joint_names_.size() > ARM_DOF) {
        const auto& gripper_motors = openarm_->get_gripper().get_motors();
     
        if (!gripper_motors.empty()) {
          double motor_pos = gripper_motors[0].get_position();
          pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);
          vel_states_[ARM_DOF] = gripper_motors[0].get_velocity();
          tau_states_[ARM_DOF] = gripper_motors[0].get_torque();
        }
      }
     
      return hardware_interface::return_type::OK;
    }
     
    hardware_interface::return_type OpenArm_v10HW::write(
        const rclcpp::Time& /*time*/,
        const rclcpp::Duration& /*period*/) {
      if (gravity_compensation_ && dynamics_) {
        dynamics_->GetGravity(pos_states_.data(), gravity_ff_.data());
      } else {
        std::fill(gravity_ff_.begin(), gravity_ff_.end(), 0.0);
      }
     
      std::vector<openarm::damiao_motor::MITParam> arm_params;
      arm_params.reserve(ARM_DOF);
     
      for (size_t i = 0; i < ARM_DOF; ++i) {
        const double ff_tau = tau_commands_[i] + gravity_scale_ * gravity_ff_[i];
     
        arm_params.push_back(
            {kp_[i], kd_[i], pos_commands_[i], vel_commands_[i], ff_tau});
      }
     
      openarm_->get_arm().mit_control_all(arm_params);
     
      if (hand_ && joint_names_.size() > ARM_DOF) {
        double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
     
        openarm_->get_gripper().mit_control_all(
            {{GRIPPER_KP, GRIPPER_KD, motor_command, 0.0, 0.0}});
      }
     
      openarm_->recv_all(1000);
      return hardware_interface::return_type::OK;
    }
     
    void OpenArm_v10HW::return_to_zero() {
      RCLCPP_INFO(
          rclcpp::get_logger("OpenArm_v10HW"),
          "Returning to zero position...");
     
      std::vector<openarm::damiao_motor::MITParam> arm_params;
      arm_params.reserve(ARM_DOF);
     
      for (size_t i = 0; i < ARM_DOF; ++i) {
        arm_params.push_back({kp_[i], kd_[i], 0.0, 0.0, 0.0});
      }
     
      openarm_->get_arm().mit_control_all(arm_params);
     
      if (hand_) {
        const double motor_zero = joint_to_motor_radians(GRIPPER_JOINT_0_POSITION);
     
        openarm_->get_gripper().mit_control_all(
            {{GRIPPER_KP, GRIPPER_KD, motor_zero, 0.0, 0.0}});
      }
     
      std::this_thread::sleep_for(std::chrono::microseconds(1000));
      openarm_->recv_all();
    }
     
    double OpenArm_v10HW::joint_to_motor_radians(double joint_value) {
      if (arm_prefix_ == "left_") {
        joint_value -= 0.02;
      }
     
      return (joint_value / GRIPPER_JOINT_0_POSITION) *
             GRIPPER_MOTOR_1_RADIANS;
    }
     
    double OpenArm_v10HW::motor_radians_to_joint(double motor_radians) {
      double joint_value = GRIPPER_JOINT_0_POSITION *
                           (motor_radians / GRIPPER_MOTOR_1_RADIANS);
     
      if (arm_prefix_ == "left_") {
        joint_value += 0.02;
      }
     
      return joint_value;
    }
     
    }  // namespace openarm_hardware
     
    #include "pluginlib/class_list_macros.hpp"
     
    PLUGINLIB_EXPORT_CLASS(
        openarm_hardware::OpenArm_v10HW,
        hardware_interface::SystemInterface)

bash

这份代码中，parse_config() 新增了 gravity_compensation、gravity_scale、gravity_urdf_path、gravity_base_link、gravity_tip_link 参数读取，并在日志中输出 gravity_comp 和 gravity_scale，便于启动时判断参数是否真的进入硬件接口。在 on_init() 中，代码 初始化 gravity_ff_，并在开启重力补偿时检查 URDF 路径、base link 和 tip link 是否完整；如果不完整或者 KDL 初始化失败，会直接返回错误。这个设计避免了参数缺失时机械臂带着错误的动力学模型运行。最核心的是 write()：这里先根据当前关节状态调用 GetGravity()，然后将 gravity_scale_ * gravity_ff_[i] 加到 tau_commands_[i] 上作为 ff_tau 下发给 MIT 控制。
修改四：CMakeLists.txt

原始 openarm_hardware 只需要编译 v10_simple_hardware.cpp，现在增加了 KDL 动力学和 URDF 解析，因此必须修改 CMake 。

文件：

~/ros2_ws/src/openarm_ros2/openarm_hardware/CMakeLists.txt
bash

核心修改如下：

    cmake_minimum_required(VERSION 3.22)
    project(openarm_hardware)
     
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wall -Wextra -Wpedantic)
    endif()
     
    find_package(ament_cmake REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(hardware_interface REQUIRED)
    find_package(kdl_parser REQUIRED)
    find_package(orocos_kdl REQUIRED)
    find_package(pluginlib REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_lifecycle REQUIRED)
    find_package(urdf REQUIRED)
    find_package(OpenArmCAN REQUIRED)
     
    add_library(${PROJECT_NAME} SHARED
      src/v10_simple_hardware.cpp
      src/dynamics.cpp
    )
     
    target_include_directories(${PROJECT_NAME}
      PRIVATE
        include
        ${EIGEN3_INCLUDE_DIRS}
    )
     
    target_link_libraries(${PROJECT_NAME}
      OpenArmCAN::openarm_can
      orocos-kdl
    )
     
    ament_target_dependencies(${PROJECT_NAME}
      hardware_interface
      kdl_parser
      orocos_kdl
      pluginlib
      rclcpp
      rclcpp_lifecycle
      urdf
    )
     
    pluginlib_export_plugin_description_file(
      hardware_interface
      openarm_hardware.xml
    )
     
    install(TARGETS ${PROJECT_NAME}
      DESTINATION lib
    )
     
    install(DIRECTORY include/
      DESTINATION include
    )
     
    ament_export_include_directories(include)
    ament_export_libraries(${PROJECT_NAME})
    ament_export_dependencies(
      hardware_interface
      kdl_parser
      orocos_kdl
      pluginlib
      rclcpp
      rclcpp_lifecycle
      urdf
    )
     
    ament_package()

bash

如果忘记加入 src/dynamics.cpp，会出现链接或类型未定义问题。如果忘记加入 KDL、URDF 相关依赖，则会出现 KDL、urdf、kdl_parser 找不到的问题。 编程
修改五：package.xml

文件：

~/ros2_ws/src/openarm_ros2/openarm_hardware/package.xml
bash

核心依赖如下：

    <?xml version="1.0"?>
    <package format="3">
      <name>openarm_hardware</name>
      <version>0.3.0</version>
      <description>Hardware interface for OpenArm with gravity compensation</description>
     
      <maintainer email="support@enactic.com">Enactic, Inc.</maintainer>
      <license>Apache-2.0</license>
     
      <buildtool_depend>ament_cmake</buildtool_depend>
     
      <depend>rclcpp</depend>
      <depend>hardware_interface</depend>
      <depend>pluginlib</depend>
      <depend>kdl_parser</depend>
      <depend>orocos_kdl</depend>
      <depend>urdf</depend>
     
      <test_depend>ament_lint_auto</test_depend>
      <test_depend>ament_lint_common</test_depend>
      <test_depend>ros2_control_test_assets</test_depend>
     
      <export>
        <build_type>ament_cmake</build_type>
      </export>
    </package>

bash

修改六：双臂 ros2_control xacro

文件：

~/ros2_ws/src/openarm_description/urdf/ros2_control/openarm.bimanual.ros2_control.xacro
bash

在左臂硬件段加入：

    <!-- gravity compensation -->
    <param name="gravity_compensation">true</param>
    <param name="gravity_scale">0.20</param>
    <param name="hold_position_on_activate">false</param>
    <param name="gravity_urdf_path">/tmp/openarm_bimanual.urdf</param>
    <param name="gravity_base_link">openarm_body_link0</param>
    <param name="gravity_tip_link">openarm_left_link7</param>

bash

在右臂硬件段加入： 计算机硬件

    <!-- gravity compensation -->
    <param name="gravity_compensation">true</param>
    <param name="gravity_scale">0.20</param>
    <param name="hold_position_on_activate">false</param>
    <param name="gravity_urdf_path">/tmp/openarm_bimanual.urdf</param>
    <param name="gravity_base_link">openarm_body_link0</param>
    <param name="gravity_tip_link">openarm_right_link7</param>

bash

为什么必须生成 /tmp/openarm_bimanual.urdf

这次实现中，Dynamics 读取的是磁盘上的 URDF 文件路径：

<param name="gravity_urdf_path">/tmp/openarm_bimanual.urdf</param>
bash

它不是直接读取 xacro，也不是直接读取 /robot_description。因此启动前必须把 xacro 展开成 URDF。

生成命令：

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
     
    ros2 run xacro xacro ~/ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro \
      bimanual:=true ros2_control:=true hand:=true > /tmp/openarm_bimanual.urdf

bash

检查命令：

    ls -l /tmp/openarm_bimanual.urdf
     
    grep -n "gravity_" /tmp/openarm_bimanual.urdf
     
    grep -n "openarm_body_link0\|openarm_left_link7\|openarm_right_link7" \
      /tmp/openarm_bimanual.urdf

bash

之前实际检查结果显示，展开后的 URDF 中已经包含左右臂的 gravity_compensation、gravity_scale、gravity_urdf_path、gravity_base_link 和 gravity_tip_link，同时也能找到 openarm_body_link0、openarm_left_link7 和 openarm_right_link7。这说明 xacro 展开和 KDL 链配置是正确的。 编程
启动流程

每次启动真机前，建议先生成一次 URDF：

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
     
    ros2 run xacro xacro ~/ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro \
      bimanual:=true ros2_control:=true hand:=true > /tmp/openarm_bimanual.urdf

bash

然后启动真机：

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
     
    ros2 launch openarm_bringup openarm.bimanual.launch.py

bash

验证方法

    主观感受：
        在高负载姿态（如右臂抬高前伸悬停）下，末端下坠趋势应显著减弱或消失。
        电机维持姿态时的异常声音（嗡嗡声、沉重感）应减轻。
        悬停状态应感觉更“轻松”。
    客观观察：
        使用 rqt_plot 或类似工具，观察启用重力补偿前后，维持相同姿态时各关节实际下发的总力矩值变化。应能看到补偿项叠加的效果。
        对比执行相同轨迹时，启用与不启用重力补偿下的轨迹跟踪精度（尤其是在需要抵抗重力的姿态段）。
    参数调整： 通过调整 gravity_scale_ 参数 \alpha(通常在 0.8~1.2 范围)，找到最佳补偿效果。\alpha过大可能导致过补偿（关节反向漂移），过小则补偿不足。
————————————————
版权声明：本文为CSDN博主「嗷嗷冲」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/qq_53520547/article/details/160255892