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

#pragma once

#include <array>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <ik_solver_msgs/GetIk.h>
#include <ik_solver_msgs/GetIkArray.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <ik_solver/ik_solver_base_class.h>
#include <pluginlib/class_loader.h>
#include <rosdyn_core/primitives.h>

#include <ros/node_handle.h>

#define TOLERANCE 1e-3
namespace ik_solver
{
class RobotOnGuideIkSolver : public IkSolver
{
public:
  virtual std::vector<Eigen::VectorXd> getIk(const Eigen::Affine3d& T_base_flange,
                                             const std::vector<Eigen::VectorXd>& seeds, const int& desired_solutions,
                                             const int& max_stall_iterations) override;

  virtual std::vector<Eigen::VectorXd> getIkSafeMT(bool& stop, const size_t& thread_id, const Eigen::Affine3d& T_base_flange,
                                                   const std::vector<Eigen::VectorXd>& seeds,
                                                   const int& desired_solutions,
                                                   const int& max_stall_iterations) override;
                                                   
  virtual Eigen::Affine3d getFK(const Eigen::VectorXd& s) override;

protected:
  virtual bool customConfig() override;

  struct MT
  {
    rosdyn::ChainPtr chain_;
    boost::shared_ptr<ik_solver::IkSolver> robot_ik_solver_;
    Eigen::VectorXd guide_seed_;
    Eigen::VectorXd mean_q_;
    Eigen::VectorXd dq_;
  };

  const rosdyn::ChainPtr& chain(const size_t& i = 0) const { return mt_.at(i).chain_;}
  const boost::shared_ptr<ik_solver::IkSolver>& robot_ik_solver(const size_t& i = 0) const { return mt_.at(i).robot_ik_solver_;}
  const Eigen::VectorXd& guide_seed(const size_t& i = 0) const { return mt_.at(i).guide_seed_;}
  const Eigen::VectorXd& mean_q(const size_t& i = 0) const { return mt_.at(i).mean_q_;}
  const Eigen::VectorXd& dq(const size_t& i = 0) const { return mt_.at(i).dq_;}

  rosdyn::ChainPtr& chain(const size_t& i = 0) { return mt_.at(i).chain_;}
  boost::shared_ptr<ik_solver::IkSolver>& robot_ik_solver(const size_t& i = 0) { return mt_.at(i).robot_ik_solver_;}
  Eigen::VectorXd& guide_seed(const size_t& i = 0) { return mt_.at(i).guide_seed_;}
  Eigen::VectorXd& mean_q(const size_t& i = 0) { return mt_.at(i).mean_q_;}
  Eigen::VectorXd& dq(const size_t& i = 0) { return mt_.at(i).dq_;}

  ros::NodeHandle robot_nh_;

  std::array<MT,IkSolver::MAX_NUM_THREADS> mt_;


  std::unique_ptr<pluginlib::ClassLoader<ik_solver::IkSolver>> ikloader_;

};
}  //  namespace ik_solver
