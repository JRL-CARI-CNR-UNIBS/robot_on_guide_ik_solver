/*
Copyright (c) 2022, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ROBOT_ON_GUIDE_IK_SOLVER__ROBOT_ON_GUIDE_IK_SOLVER_H
#define ROBOT_ON_GUIDE_IK_SOLVER__ROBOT_ON_GUIDE_IK_SOLVER_H

#include <Eigen/Geometry>
#include <string>
#include <memory.h>
#include <boost/shared_ptr.hpp>
#include "rosdyn_core/internal/types.h"

#include <ros/node_handle.h>
#include <pluginlib/class_loader.h>
#include <rosdyn_core/primitives.h>
#include <ik_solver/ik_solver_base_class.h>

#define TOLERANCE 1e-3
namespace ik_solver
{

double axes_distance(const Eigen::Vector3d& V, const Eigen::Vector3d& uv, const Eigen::Vector3d& R, const Eigen::Vector3d& ur);
bool cylinder_ray_intersection(Eigen::Vector3d& K, const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& uv, const double& r, const Eigen::Vector3d& cylinder_ax, bool closest_to_lb=true);
bool cylinder_ray_intersection(Eigen::Vector3d& K, const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Affine3d& ub, const double& r, const Eigen::Vector3d& cylinder_ax, bool closest_to_lb=true);
double compute_polar_reaching(const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& ray, const Eigen::Vector3d& cylinder_ax);
double compute_polar_reaching(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Affine3d& ub, const Eigen::Vector3d& cylinder_ax);
Eigen::Vector3d project(const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& u);
Eigen::Vector3d project(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Affine3d& ub);

class RobotOnGuideIkSolver : public IkSolver
{
public:
  RobotOnGuideIkSolver() : ikloader_(nullptr) {};
  RobotOnGuideIkSolver(const RobotOnGuideIkSolver&) = delete;
  RobotOnGuideIkSolver(const RobotOnGuideIkSolver&&) = delete;
  RobotOnGuideIkSolver(RobotOnGuideIkSolver&&) = delete;
  virtual ~RobotOnGuideIkSolver() = default;

  virtual bool config(const ros::NodeHandle& nh, const std::string& params_ns) override;
  virtual Solutions getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds,
                                 const int& desired_solutions = -1, const int& min_stall_iterations = -1, const int& max_stall_iterations = -1) override;

  virtual Eigen::Affine3d getFK(const Configuration& s) override;

  rosdyn::ChainPtr guide() { return guide_.chain_;}
  Eigen::Affine3d le() { return guide_.le_;}
  Eigen::Affine3d ue() { return guide_.ue_;}

protected:
  
  ros::NodeHandle robot_on_guide_nh_; // all the information of the full chain
  ros::NodeHandle attached_robot_nh_; // only the attached robot infos

  std::unique_ptr<pluginlib::ClassLoader<ik_solver::IkSolver>> ikloader_;

  boost::shared_ptr<ik_solver::IkSolver> attached_robot_;

  double target_reaching_ = 2.0;
  struct Chain
  {
    rosdyn::ChainPtr chain_;
    Configuration    jstroke_;
    Configuration    jax_;
    Eigen::Affine3d  le_;
    Eigen::Affine3d  ue_;
    Eigen::Vector3d  pstroke_;
    Eigen::Vector3d  pax_;
  } guide_;

  std::string plugin_name_;
  double max_seek_range_m_ = 0.1;  //
  
};


}  //  namespace ik_solver

#endif  // ROBOT_ON_GUIDE_IK_SOLVER__ROBOT_ON_GUIDE_IK_SOLVER_H