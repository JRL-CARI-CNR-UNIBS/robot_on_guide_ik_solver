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
#include <vector>
#include "Eigen/src/Geometry/Transform.h"
#include <eigen_conversions/eigen_msg.h>
#include <ik_solver/ik_solver_base_class.h>
#include <ik_solver_msgs/GetIk.h>
#include <ik_solver_msgs/GetIkArray.h>
#include <pluginlib/class_loader.h>
#include <rosdyn_core/primitives.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <ros/node_handle.h>

#define TOLERANCE 1e-3
namespace ik_solver
{
class RobotOnGuideIkSolver : public IkSolver
{
public:
  virtual IkConfigurations getIk(const Eigen::Affine3d& T_base_flange, const IkConfigurations& seeds,
                                 const int& desired_solutions, const int& max_stall_iterations) override;

  virtual IkConfigurations getIkSafeMT(bool& stop, const Eigen::Affine3d& T_base_flange, const IkConfigurations& seeds,
                                       const int& desired_solutions, const int& max_stall_iterations) override;

  virtual Eigen::Affine3d getFK(const Eigen::VectorXd& s) override;

protected:
  virtual bool customConfig() override;

  ros::NodeHandle robot_nh_;

  std::unique_ptr<pluginlib::ClassLoader<ik_solver::IkSolver>> ikloader_;

  struct MT
  {
    MT() : guide_chain_(nullptr), robot_ik_solver_(nullptr), solutions_(0), running_(false)
    {

    }
    rosdyn::ChainPtr guide_chain_;
    boost::shared_ptr<ik_solver::IkSolver> robot_ik_solver_;
    Eigen::VectorXd guide_lb_;
    Eigen::VectorXd guide_ub_;
    Eigen::Affine3d guide_lb_pose_;
    Eigen::Affine3d guide_ub_pose_;
    IkConfigurations solutions_;
    bool running_;
  };
  std::vector<MT> mt_;

  const rosdyn::ChainPtr& guide_chain(const size_t& i = 0) const;
  rosdyn::ChainPtr& guide_chain(const size_t& i = 0);

  const boost::shared_ptr<ik_solver::IkSolver>& robot_ik_solver(const size_t& i = 0) const;
  boost::shared_ptr<ik_solver::IkSolver>& robot_ik_solver(const size_t& i = 0);

  const Eigen::VectorXd& guide_lb(const size_t& i = 0) const;
  Eigen::VectorXd& guide_lb(const size_t& i = 0);

  const Eigen::VectorXd& guide_ub(const size_t& i = 0) const;
  Eigen::VectorXd& guide_ub(const size_t& i = 0);

  const Eigen::Affine3d& guide_lb_pose(const size_t& i = 0) const;
  Eigen::Affine3d& guide_lb_pose(const size_t& i = 0);

  const Eigen::Affine3d& guide_ub_pose(const size_t& i = 0) const;
  Eigen::Affine3d& guide_ub_pose(const size_t& i = 0);


  IkConfigurations getIkSafeMT(bool& stop, const size_t& thread_id, const Eigen::Affine3d& T_base_flange,
                               const IkConfigurations& seeds, const int& desired_solutions,
                               const int& max_stall_iterations);

  IkConfigurations getIkSharedSeed(bool& stop, const size_t& thread_id, const Eigen::Affine3d& T_base_flange,
                                   const IkConfigurations& seeds, const int& desired_solutions,
                                   const int& max_stall_iterations);

  IkConfigurations getIkProjectedSeed(bool& stop, const size_t& thread_id, const Eigen::Affine3d& T_base_flange,
                                      const IkConfigurations& seeds, const int& desired_solutions,
                                      const int& max_stall_iterations);

  std::string seed_generation_algorithm_;
  struct ramndom_local_params
  {
    double max_range_weight_ = 0.1;  //
  } ramndom_local_params_;
};
}  //  namespace ik_solver

#endif  // ROBOT_ON_GUIDE_IK_SOLVER__ROBOT_ON_GUIDE_IK_SOLVER_H