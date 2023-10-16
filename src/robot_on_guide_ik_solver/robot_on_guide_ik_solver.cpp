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
#include <iterator>
#include "Eigen/src/Core/Matrix.h"
#include "ros/node_handle.h"

PLUGINLIB_EXPORT_CLASS(ik_solver::RobotOnGuideIkSolver, ik_solver::IkSolver)

namespace ik_solver
{
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

  std::string flange_frame;
  nh_.getParam("flange_frame", flange_frame);
  robot_nh_.setParam("flange_frame", flange_frame);

  std::string tool_frame;
  nh_.getParam("tool_frame", tool_frame);
  robot_nh_.setParam("tool_frame", tool_frame);

  std::string plugin_name;
  if (!robot_nh_.getParam("type", plugin_name))
  {
    ROS_ERROR("%s/type is not defined", robot_nh_.getNamespace().c_str());
    return -1;
  }

  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;

  for(auto & t : mt_)
  {
    t.guide_chain_ = rosdyn::createChain(model_, base_frame_, robot_base_frame, gravity);
  }
  
  std::vector<std::string> guide_names = chain()->getActiveJointsName();
  std::vector<std::string> ordered_guide_names;
  std::vector<std::string> ordered_robot_names;
  for (std::string& s : joint_names_) // inehrited field
  {
    if (std::find(guide_names.begin(), guide_names.end(), s) != guide_names.end())
    {
      ordered_guide_names.push_back(s);
    }
    else
    {
      ordered_robot_names.push_back(s);
    }
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

  // RE FILL THE JOINT NAMES, ORDERED AS GUIDE - ROBOT
  joint_names_.clear();
  for (auto& s : ordered_guide_names)
  {
    joint_names_.push_back(s);
  }
  for (auto& s : ordered_robot_names)
  {
    joint_names_.push_back(s);
  }

  for(size_t i=0; i< mt_.size(); i++)
  {
    thread_status_.at(i) = false;
    mt_.at(i).guide_chain_->setInputJointsName(ordered_guide_names);
    ROS_DEBUG("%s creating ik plugin for mounted robot", robot_nh_.getNamespace().c_str());
    mt_.at(i).robot_ik_solver_ = ikloader_->createInstance(plugin_name);

    ros::NodeHandle ik_solver_nh(robot_nh_, "mt" + std::to_string(i));
    if (!mt_.at(i).robot_ik_solver_->config(ik_solver_nh, robot_nh_.getNamespace())) // all takes the data from the same naespace, but they create services in private ns
    {
      ROS_ERROR("%s: error configuring mounted robot ik", nh_.getNamespace().c_str());
      return false;
    }
  }

  // =============================
    for(auto & t : mt_)
  {
    t.robot_lb_.resize(ordered_robot_names.size());
    t.robot_ub_.resize(ordered_robot_names.size());
  }
  rosdyn::ChainPtr robot_chain = rosdyn::createChain(model_, robot_base_frame, flange_frame, gravity);
  const auto & robot_jns = robot_chain->getActiveJointsName();
  for(size_t k=0; k<ordered_robot_names.size(); k++)
  {
    auto it = std::find(robot_jns.begin(), robot_jns.end(), ordered_robot_names.at(k));
    if(it != robot_jns.end())
    {
      size_t ji=std::distance(robot_jns.begin(), it);
      for(auto & t : mt_)
      {
        t.robot_lb_(k) = robot_chain->getQMin()(ji);
        t.robot_ub_(k) = robot_chain->getQMax()(ji);
      }
    }
  
  }

  std::string jns = "[";
  for(const auto & jn : ordered_robot_names)
  {
    jns += jn + ",";
  }
  jns += "]";
  ROS_INFO("Robot Chain has %zu DOF: %s", ordered_robot_names.size(), jns.c_str());

  jns = "[";
  for(const auto & jn : ordered_guide_names)
  {
    jns += jn + ",";
  }
  jns += "]";
  ROS_INFO("Guide Chain has %u DOF: %s", chain()->getActiveJointsNumber(), jns.c_str());
  // =================================

  ROS_DEBUG("Created %s IK Solver", nh_.getNamespace().c_str());

  return true;
}

std::vector<Eigen::VectorXd> RobotOnGuideIkSolver::getIk(const Eigen::Affine3d& T_base_flange,
                                                         const std::vector<Eigen::VectorXd>& seeds,
                                                         const int& desired_solutions, const int& max_stall_iterations)
{
  bool stop = false;
  return getIkSafeMT(stop, 0, T_base_flange, seeds, desired_solutions, max_stall_iterations);
}

void filter_seeds(const std::vector<Eigen::VectorXd>& full_seeds, const size_t& guide_seed_dof, std::vector<Eigen::VectorXd>& guide_seeds, std::vector<Eigen::VectorXd>& robot_seeds)
{
  std::vector<Eigen::VectorXd> ret;
  for(size_t i=0; i<full_seeds.size(); i++)
  {
    Eigen::VectorXd guide_seed = full_seeds.at(i).head(guide_seed_dof);
    Eigen::VectorXd robot_seed = full_seeds.at(i).tail(full_seeds.at(i).size()-guide_seed_dof);
    if(i==0)
    {
      guide_seeds.push_back(guide_seed);
      robot_seeds.push_back( robot_seed);      
    }
    else
    {
      if (((guide_seed - guide_seeds.back()).norm() > 1e-6) || ((robot_seed - robot_seeds.back()).norm() > 1e-6) )
      {
        guide_seeds.push_back(guide_seed);
        robot_seeds.push_back( robot_seed);
      }
    }
  }
}

Eigen::VectorXd random(const Eigen::VectorXd& v, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, double range_percentage)
{
  Eigen::VectorXd ret = v;
  Eigen::VectorXd rand = Eigen::VectorXd(v.size()).setRandom();
  Eigen::VectorXd ub_dist_v = range_percentage * (ub - v);
  Eigen::VectorXd lb_dist_v = range_percentage * (v - lb);
  
  for(size_t i=0;i<v.size(); i++)
  {
    ret(i) += rand(i)>=0 ? rand(i) * ub_dist_v(i) : rand(i) * lb_dist_v(i);
  }
  
  return ret;
}

std::vector<Eigen::VectorXd> RobotOnGuideIkSolver::getIkSafeMT(bool& stop,
                                                               const Eigen::Affine3d& T_base_flange,
                                                               const std::vector<Eigen::VectorXd>& seeds,
                                                               const int& desired_solutions,
                                                               const int& max_stall_iterations)
{
  while(true)
  {
    for(size_t i=0;i<IkSolver::MAX_NUM_THREADS;i++)
    {
      if(!thread_status_.at(i))
      {
        thread_status_.at(i) = true;
        mt_solutions_.at(i) = this->getIkSafeMT(stop, i, T_base_flange, seeds, desired_solutions, max_stall_iterations);
        thread_status_.at(i) = false;
        return mt_solutions_.at(i);
      }
    }
  }
}

std::vector<Eigen::VectorXd> RobotOnGuideIkSolver::getIkSafeMT(bool& stop,
                                                               const size_t& thread_id,
                                                               const Eigen::Affine3d& T_base_flange,
                                                               const std::vector<Eigen::VectorXd>& seeds,
                                                               const int& desired_solutions,
                                                               const int& max_stall_iterations)
{
  std::vector<Eigen::VectorXd> solutions;
  solutions.clear();

  std::vector<Eigen::VectorXd> _robot_seeds;
  std::vector<Eigen::VectorXd> _guide_seeds;
  filter_seeds(seeds, guide_chain(thread_id)->getActiveJointsNumber(), _guide_seeds, _robot_seeds);
  

  for (size_t idx = 0; idx < (size_t)max_stall_iterations && nh_.ok(); idx++)
  {
    size_t seed_index = idx % _guide_seeds.size();

    Eigen::VectorXd robot_seed = _robot_seeds.at(seed_index);
    Eigen::VectorXd guide_seed = _guide_seeds.at(seed_index);
    
    if(idx > _guide_seeds.size())
    {
      double r = double(idx) / double(max_stall_iterations);
      double usage_range = 0.1 * (1.0 - r) + 0.9 * (r);
      guide_seed = random(guide_seed, guide_chain(thread_id)->getQMin(), guide_chain(thread_id)->getQMax(), usage_range); 
      robot_seed = random(robot_seed, robot_lb(thread_id), robot_ub(thread_id), usage_range); 
    }

    Eigen::Affine3d T_base_robotbase = chain(thread_id)->getTransformation(guide_seed);
    Eigen::Affine3d T_robotbase_flange = T_base_robotbase.inverse() * T_base_flange;

    std::vector<Eigen::VectorXd> robot_sol =
        robot_ik_solver(thread_id)->getIk(T_robotbase_flange, {robot_seed}, desired_solutions, max_stall_iterations);

    for (const Eigen::VectorXd& q_robot : robot_sol)
    {
      Eigen::VectorXd q_tot(guide_seed.size() + q_robot.size());
      q_tot << guide_seed(thread_id), q_robot;
      solutions.push_back(q_tot);
    }
    
    if ((int)solutions.size() > desired_solutions)
    {
      break;
    }
  }
  return solutions;
}

Eigen::Affine3d RobotOnGuideIkSolver::getFK(const Eigen::VectorXd& s)
{
  Eigen::VectorXd s_guide = s.head(chain()->getActiveJointsNumber());
  Eigen::VectorXd s_robot = s.tail(s.size() - chain()->getActiveJointsNumber());
  Eigen::Affine3d T_base_robotbase = chain()->getTransformation(s_guide);
  Eigen::Affine3d T_robotbase_flange = robot_ik_solver()->getFK(s_robot);
  return T_base_robotbase * T_robotbase_flange;
}
}  // end namespace ik_solver
