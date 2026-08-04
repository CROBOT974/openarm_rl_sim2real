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
  Dynamics(std::string urdf_path, std::string start_link,
           std::string end_link);
  ~Dynamics();

  bool Init();

  void GetGravity(const double* joint_position, double* gravity);

  void GetCoriolis(const double* joint_position, const double* joint_velocity,
                   double* coriolis);

  void GetMassMatrixDiagonal(const double* joint_position, double* inertia_diag);

  void GetJacobian(const double* joint_position, Eigen::MatrixXd& jacobian);

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
