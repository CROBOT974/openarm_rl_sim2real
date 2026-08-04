#include "openarm_hardware/dynamics.hpp"

#include <iostream>
#include <memory>
#include <utility>

Dynamics::Dynamics(std::string urdf_path, std::string start_link,
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
    std::cerr << "Failed to get KDL chain: " << start_link_ << " -> "
              << end_link_ << std::endl;
    return false;
  }
  const auto n = kdl_chain_.getNrOfJoints();
  coriolis_forces_.resize(n);
  gravity_forces_.resize(n);
  inertia_matrix_.resize(n);
  coriolis_forces_.data.setZero();
  gravity_forces_.data.setZero();
  inertia_matrix_.data.setZero();
  solver_ =
      std::make_unique<KDL::ChainDynParam>(kdl_chain_, KDL::Vector(0.0, 0.0, -9.81));
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

void Dynamics::GetCoriolis(const double* joint_position,
                           const double* joint_velocity, double* coriolis) {
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

void Dynamics::GetMassMatrixDiagonal(const double* joint_position,
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

void Dynamics::GetJacobian(const double* joint_position,
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
