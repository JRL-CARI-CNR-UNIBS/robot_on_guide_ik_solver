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

#include <robot_on_guide_ik_solver/robot_on_guide_ik_solver.h>
#include <pluginlib/class_list_macros.h>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Transform.h"

PLUGINLIB_EXPORT_CLASS(ik_solver::RobotOnGuideIkSolver, ik_solver::IkSolver)

namespace ik_solver
{

double percentage_distance_from(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Vector3d& v)
{
  Eigen::Vector3d d = p.translation() - lb.translation();
  return d.dot(v.normalized()) / v.norm();
}

Eigen::VectorXd local_random(const Eigen::VectorXd& v, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                             double range_percentage)
{
  Eigen::VectorXd ret = v;
  Eigen::VectorXd rand = Eigen::VectorXd(v.size()).setRandom();
  Eigen::VectorXd ub_dist_v = range_percentage * (ub - v);
  Eigen::VectorXd lb_dist_v = range_percentage * (v - lb);

  for (size_t i = 0; i < v.size(); i++)
  {
    ret(i) += rand(i) >= 0 ? rand(i) * ub_dist_v(i) : rand(i) * lb_dist_v(i);
  }

  return ret;
}

double linear_interpolation(const double& lb, const double& ub, const size_t& i_step, const size_t& n_steps)
{
  double dd = (ub - lb) / double(n_steps);
  return lb + i_step * dd;
}

double sin_interpolation(const double& lb, const double& ub, const size_t& i_step, const size_t& n_steps)
{
  auto dd = (ub - lb) * (1.0 - std::cos(double(i_step)/double(n_steps) * M_PI/2.0));
  return lb + dd;
}

inline bool RobotOnGuideIkSolver::customConfig()
{
  ROS_DEBUG("Start creating %s IK Solver", nh_.getNamespace().c_str());
  if (!nh_.hasParam(nh_.getNamespace() + "/mounted_robot_ik"))
  {
    ROS_INFO("%s/robot_ik is not defined", nh_.getNamespace().c_str());
    return false;
  }

  std::string robot_base_frame;
  if (!nh_.getParam("robot_base_frame", robot_base_frame))
  {
    ROS_ERROR("%s/robot_base_frame is not specified", nh_.getNamespace().c_str());
    return false;
  }
  robot_nh_ = ros::NodeHandle(nh_.getNamespace() + "/mounted_robot_ik");

  ikloader_.reset(new pluginlib::ClassLoader<ik_solver::IkSolver>("ik_solver", "ik_solver::IkSolver"));

  robot_nh_.setParam("base_frame", robot_base_frame);

  std::string tmp;
  nh_.getParam("flange_frame", tmp);
  robot_nh_.setParam("flange_frame", tmp);
  nh_.getParam("tool_frame", tmp);
  robot_nh_.setParam("tool_frame", tmp);

  std::string plugin_name;
  if (!robot_nh_.getParam("type", plugin_name))
  {
    ROS_ERROR("%s/type is not defined", robot_nh_.getNamespace().c_str());
    return -1;
  }

  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;

  guide_chain_ = rosdyn::createChain(model_, base_frame_, robot_base_frame, gravity);


  std::vector<std::string> guide_names = guide_chain_->getActiveJointsName();
  std::vector<std::string> ordered_guide_names;
  std::vector<std::string> ordered_robot_names;
  for (std::string& s : joint_names_)
  {
    if (std::find(guide_names.begin(), guide_names.end(), s) != guide_names.end())
      ordered_guide_names.push_back(s);
    else
      ordered_robot_names.push_back(s);
  }
  if (ordered_guide_names.size() != guide_names.size())
  {
    ROS_ERROR("not all the guides names are listed in %s/joint_names", nh_.getNamespace().c_str());
    ROS_ERROR("Guide names: ");
    for (std::string& s : guide_names)
      ROS_ERROR(" - %s", s.c_str());
    ROS_ERROR("joint_names: ");
    for (std::string& s : joint_names_)
      ROS_ERROR(" - %s", s.c_str());
    return false;
  }
  robot_nh_.setParam("joint_names", ordered_robot_names);

  joint_names_.clear();
  for (auto& s : ordered_guide_names)
    joint_names_.push_back(s);
  for (auto& s : ordered_robot_names)
    joint_names_.push_back(s);

  guide_chain_->setInputJointsName(ordered_guide_names);

  ROS_DEBUG("%s creating ik plugin for mounted robot", robot_nh_.getNamespace().c_str());
  robot_ik_solver_ = ikloader_->createInstance(plugin_name);

  if (!robot_ik_solver_->config(robot_nh_))
  {
    ROS_ERROR("%s: error configuring mounted robot ik", nh_.getNamespace().c_str());
    return false;
  }

  dq_ = 0.5 * (guide_chain_->getQMax() - guide_chain_->getQMin());
  mean_q_ = 0.5 * (guide_chain_->getQMax() + guide_chain_->getQMin());

  lower_guide_extremity_ = guide_chain_->getTransformation(guide_chain_->getQMin());
  upper_guide_extremity_ = guide_chain_->getTransformation(guide_chain_->getQMax());
  guide_main_direction_ = upper_guide_extremity_.translation() - lower_guide_extremity_.translation();

  ROS_DEBUG("Created %s IK Solver", nh_.getNamespace().c_str());

  return true;
}

std::vector<Eigen::VectorXd> RobotOnGuideIkSolver::getIkSharedSeed(const Eigen::Affine3d& T_base_flange,
                                                                   const std::vector<Eigen::VectorXd>& seeds,
                                                                   const int& desired_solutions,
                                                                   const int& max_stall_iterations)
{
  std::vector<Eigen::VectorXd> solutions;
  solutions.clear();
  std::vector<Eigen::VectorXd> seeds_robot;

  int idx = 0;
  Eigen::VectorXd last_guide_seed(guide_chain_->getActiveJointsNumber());
  Eigen::VectorXd guide_seed(guide_chain_->getActiveJointsNumber());
  guide_seed.setZero();
  int idx_seed = 0;

  for (idx = 0; idx < max_stall_iterations; idx++)
  {
    if (!nh_.ok())
      break;
    seeds_robot.clear();

    bool use_random = false;
    if (idx_seed < (int)seeds.size())
    {
      while (true)
      {
        const Eigen::VectorXd& full_seed = seeds.at(idx_seed);
        guide_seed = full_seed.head(guide_seed.size());
        seeds_robot.push_back(full_seed.tail(full_seed.size() - guide_seed.size()));
        if ((guide_seed - last_guide_seed).norm() > 1e-6)
          break;
        else
        {
          if (idx_seed < (int)seeds.size() - 1)
            idx_seed++;  // discard repeated seeds
          else
          {
            use_random = true;
            break;
          }
        }
      }

      last_guide_seed = guide_seed;
    }
    else
      use_random = true;

    if (use_random)
    {
      guide_seed.setRandom();
      guide_seed = mean_q_ + dq_.cwiseProduct(guide_seed);
    }

    Eigen::Affine3d T_base_robotbase = guide_chain_->getTransformation(guide_seed);
    Eigen::Affine3d T_robotbase_flange = T_base_robotbase.inverse() * T_base_flange;

    std::vector<Eigen::VectorXd> robot_sol =
        robot_ik_solver_->getIk(T_robotbase_flange, { seeds_robot }, desired_solutions, max_stall_iterations);

    for (const Eigen::VectorXd& q_robot : robot_sol)
    {
      Eigen::VectorXd q_tot(guide_seed.size() + q_robot.size());
      q_tot << guide_seed, q_robot;
      solutions.push_back(q_tot);
    }
    idx_seed++;

    if ((int)solutions.size() > desired_solutions)
    {
      break;
    }
  }

  if (solutions.size() == 0)
  {
    ROS_INFO("seed size =%zu, solution size=%zu, iteration=%d", seeds.size(), solutions.size(), idx);
  }

  return solutions;
}

std::vector<Eigen::VectorXd> RobotOnGuideIkSolver::getIkProjectedSeed(const Eigen::Affine3d& T_base_flange,
                                                                      const std::vector<Eigen::VectorXd>& seeds,
                                                                      const int& desired_solutions,
                                                                      const int& max_stall_iterations)
{
  std::vector<Eigen::VectorXd> solutions;

  Eigen::VectorXd _guide_q_seed_0 =
      2.0 * percentage_distance_from(T_base_flange, lower_guide_extremity_, guide_main_direction_) * dq_;

  double max_range_weight = 0.1;
  double range_weight = max_range_weight;
  for (size_t idx = 0; idx < max_stall_iterations; idx++)
  {
    Eigen::VectorXd _guide_q_seed;
    if (idx == 0)
    {
      _guide_q_seed = _guide_q_seed_0;
    }
    else
    {
      double range = sin_interpolation(0, range_weight * guide_main_direction_.norm(), idx, max_stall_iterations);
      _guide_q_seed = local_random(_guide_q_seed_0, guide_chain_->getQMin(),
                                   guide_chain_->getQMax(), range);
    }

    Eigen::Affine3d T_base_robotbase = guide_chain_->getTransformation(_guide_q_seed);
    Eigen::Affine3d T_robotbase_flange = T_base_robotbase.inverse() * T_base_flange;

    std::vector<Eigen::VectorXd> robot_sol = robot_ik_solver_->getIk(T_robotbase_flange, {}, desired_solutions, 0);

    for (const Eigen::VectorXd& q_robot : robot_sol)
    {
      Eigen::VectorXd q_tot(_guide_q_seed.size() + q_robot.size());
      q_tot << _guide_q_seed, q_robot;
      solutions.push_back(q_tot);
    }

    if ((int)solutions.size() > desired_solutions)
    { 
      break;
    }

    range_weight = robot_sol.size() ? max_range_weight / double(robot_sol.size()) : max_range_weight;
  }

  return solutions;
}

std::vector<Eigen::VectorXd> RobotOnGuideIkSolver::getIk(const Eigen::Affine3d& T_base_flange,
                                                         const std::vector<Eigen::VectorXd>& seeds,
                                                         const int& desired_solutions, const int& max_stall_iterations)
{
  int manuel = 1;
  if (manuel)
  {
    return getIkProjectedSeed(T_base_flange, seeds, desired_solutions, max_stall_iterations);
  }
  else
  {
    return getIkProjectedSeed(T_base_flange, seeds, desired_solutions, max_stall_iterations);
  }
}

Eigen::Affine3d RobotOnGuideIkSolver::getFK(const Eigen::VectorXd& s)
{
  Eigen::VectorXd s_guide = s.head(guide_chain_->getActiveJointsNumber());
  Eigen::VectorXd s_robot = s.tail(s.size() - guide_chain_->getActiveJointsNumber());
  Eigen::Affine3d T_base_robotbase = guide_chain_->getTransformation(s_guide);
  Eigen::Affine3d T_robotbase_flange = robot_ik_solver_->getFK(s_robot);
  return T_base_robotbase * T_robotbase_flange;
}
}  // end namespace ik_solver
