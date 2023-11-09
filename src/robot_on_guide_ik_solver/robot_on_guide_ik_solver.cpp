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

#include <algorithm>
#include <cstring>
#include <vector>
#include <Eigen/Geometry>
#include "Eigen/src/Core/Matrix.h"

#include <pluginlib/class_list_macros.h>
#include <robot_on_guide_ik_solver/robot_on_guide_ik_solver.h>

PLUGINLIB_EXPORT_CLASS(ik_solver::RobotOnGuideIkSolver, ik_solver::IkSolver)

namespace ik_solver
{

bool order_joint_names(std::vector<std::string>& joint_names, std::vector<std::string>& guide_names,
                       std::vector<std::string>& robot_names)
{
  // std::vector<std::string> guide_names = guide_.chain_->getActiveJointsName();
  std::vector<std::string> ordered_guide_names;
  std::vector<std::string> ordered_robot_names;
  for (const auto& s : joint_names)
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
    ROS_ERROR("Guide names: ");
    for (std::string& s : guide_names)
    {
      ROS_ERROR(" - %s", s.c_str());
    }
    ROS_ERROR("joint_names: ");
    for (std::string& s : joint_names)
    {
      ROS_ERROR(" - %s", s.c_str());
    }
    return false;
  }

  guide_names = ordered_guide_names;
  robot_names = ordered_robot_names;

  std::string jns = "Ordered Guide Names [";
  joint_names.clear();
  for (auto& s : ordered_guide_names)
  {
    jns += s + ",";
    joint_names.push_back(s);
  }
  jns += "] Ordered Robot Names [";
  for (auto& s : ordered_robot_names)
  {
    jns += s + ",";
    joint_names.push_back(s);
  }
  jns += "]";

  ROS_DEBUG("Robot Chain has %zu DOF, Guide Chain has %zu DOF - %s", robot_names.size(), ordered_guide_names.size(),
           jns.c_str());
  return true;
}

double percentage_distance_from(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Affine3d& ub)
{
  Eigen::Vector3d v  = ub.translation() - lb.translation();
  Eigen::Vector3d d  = p.translation() - lb.translation();
  double          dv = d.dot(v.normalized()) / v.norm();
  return dv > 1.0 ? 1.0 : dv < 0.0 ? 0.0 : dv;

}

double percentage_distance_from(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Vector3d& v)
{
  Eigen::Vector3d d = p.translation() - lb.translation();
  double          dv = d.dot(v.normalized()) / v.norm();
  return dv > 1.0 ? 1.0 : dv < 0.0 ? 0.0 : dv;

}

Eigen::Vector3d distance_from(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Vector3d& v)
{
  Eigen::Vector3d d = p.translation() - lb.translation();
  return d.dot(v.normalized()) * v.normalized();
}

Eigen::VectorXd force_inbound(const Eigen::VectorXd& p, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub)
{
  Eigen::VectorXd uv = (ub - lb).normalized();
  return  (p.dot(uv) > ub.dot(uv)) ? ub : 
          (p.dot(uv) < lb.dot(uv)) ? lb : 
          p;
}



Eigen::VectorXd local_random(const Eigen::VectorXd& p, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                             double range_percentage)
{
  Eigen::MatrixXd eye = Eigen::VectorXd(p.size()).setOnes().asDiagonal();
  Eigen::MatrixXd rand = Eigen::VectorXd(p.size()).setRandom().asDiagonal();

  return force_inbound(lb + range_percentage * 0.5 * (eye+rand) * (ub - lb), lb, ub);
}

double linear_interpolation(const double& lb, const double& ub, const size_t& i_step, const size_t& n_steps)
{
  double dd = (ub - lb) / double(n_steps);
  return lb + i_step * dd;
}

double sin_interpolation(const double& lb, const double& ub, const size_t& i_step, const size_t& n_steps)
{
  assert(ub - lb);
  auto dd = (ub - lb) * (1.0 - std::cos(double(i_step) / double(n_steps) * M_PI / 2.0));
  return lb + dd;
}

void filter_seeds(const Configurations& full_seeds, const size_t& guide_seed_dof, Configurations& guide_seeds,
                  Configurations& robot_seeds)
{
  Configurations ret;
  for (size_t i = 0; i < full_seeds.size(); i++)
  {
    Eigen::VectorXd guide_seed = full_seeds.at(i).head(guide_seed_dof);
    Eigen::VectorXd robot_seed = full_seeds.at(i).tail(full_seeds.at(i).size() - guide_seed_dof);
    if (i == 0)
    {
      guide_seeds.push_back(guide_seed);
      robot_seeds.push_back(robot_seed);
    }
    else
    {
      if (((guide_seed - guide_seeds.back()).norm() > 1e-6) || ((robot_seed - robot_seeds.back()).norm() > 1e-6))
      {
        guide_seeds.push_back(guide_seed);
        robot_seeds.push_back(robot_seed);
      }
    }
  }
}

Eigen::VectorXd random(const Eigen::VectorXd& v, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
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

//==========================================================================================================================
inline bool RobotOnGuideIkSolver::customConfig()
{
  
  ROS_DEBUG("Start creating %s IK Solver", robot_nh_.getNamespace().c_str());

  // ============================================================================
  robot_on_guide_nh_ = this->robot_nh_;
  attached_robot_nh_ = ros::NodeHandle(this->robot_nh_.getNamespace() + "/mounted_robot_ik");

  std::map<std::string, std::vector<std::string>> mandatory_params{
    { "/", {} },
    { robot_on_guide_nh_.getNamespace(), { "mounted_robot_ik", "seed_generation_algorithm" } },
    { attached_robot_nh_.getNamespace(), { "type", "base_frame" } }
  };

  for (const auto& pp : mandatory_params)
  {
    std::string ns = rtrim(trim(pp.first), "/") + "/";
    for (const auto& p : pp.second)
    {
      std::string pn = ns + ltrim(p, "/");
      if (!ros::param::has(ns + p))
      {
        ROS_ERROR("%s not found in rosparam server", pn.c_str());
        return false;
      }
    }
  }

  std::string root_ns = "/";
  std::string robot_on_guide_ns = rtrim(robot_on_guide_nh_.getNamespace(), "/") + "/";
  std::string attached_robot_ns = rtrim(attached_robot_nh_.getNamespace(), "/") + "/";

  std::map<std::string, std::string> _frames{
    { "flange_frame", flange_frame_ },
    { "tool_frame", tool_frame_ },
  };

  for (const auto& _f : _frames)
  {
    if (!check_and_set_duplicate_params(robot_on_guide_ns, attached_robot_ns, _f.first, _f.second))
    {
      return false;
    }
  }

  // INIT GUIDE CHAIN
  std::string mounted_robot_base_frame;
  ros::param::get(attached_robot_ns + "base_frame", mounted_robot_base_frame);

  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;

  guide_.chain_ = rosdyn::createChain(model_, base_frame_, mounted_robot_base_frame, gravity);
  
  std::vector<std::string> guide_names = guide_.chain_->getActiveJointsName();
  ROS_DEBUG_STREAM("Guide base_frame: " << base_frame_);
  ROS_DEBUG_STREAM("Guide mounted_robot_base_frame: " << mounted_robot_base_frame);
    
  std::vector<std::string> robot_names{};
  if (order_joint_names(joint_names_, guide_names, robot_names))
  {
    // reorder in the param array
    ros::param::set(robot_on_guide_ns + "/joint_names", joint_names_);
    ros::param::set(attached_robot_ns + "/joint_names", robot_names);
  }
  else
  {
    ROS_ERROR("Not all the guides names are listed in %s/joint_names", robot_on_guide_ns.c_str());
    return false;
  }

  // INIT MOUNTED ROBOT
  std::string plugin_name;
  ros::param::get(attached_robot_ns + "type", plugin_name);
  ikloader_.reset(new pluginlib::ClassLoader<ik_solver::IkSolver>("ik_solver", "ik_solver::IkSolver"));

  ROS_DEBUG("%s creating ik plugin for mounted robot", attached_robot_ns.c_str());

  guide_.chain_->setInputJointsName(guide_names);

  ROS_DEBUG("%s creating ik plugin for mounted robot", attached_robot_ns.c_str());
  attached_robot_ = ikloader_->createInstance(plugin_name);

  ros::NodeHandle ik_solver_nh(robot_nh_, "robot_on_guide");
  if(!attached_robot_->config(ik_solver_nh, attached_robot_ns))  // all takes the data from the same naespace,
                                                                    // but they create services in private ns
  {
    ROS_ERROR("%s: error configuring mounted robot ik", attached_robot_ns.c_str());
    return false;
  }

  guide_.range_   = guide_.chain_->getQMax() - guide_.chain_->getQMin();
  guide_.lb_pose_ = guide_.chain_->getTransformation(guide_.chain_->getQMin());
  guide_.ub_pose_ = guide_.chain_->getTransformation(guide_.chain_->getQMax());
  

  // =================================

  // FNISH GUIDE SETUP
  // ======================================================
  seed_generation_algorithm_ = "recursive";
  ros::param::get(robot_on_guide_ns + "seed_generation_algorithm", seed_generation_algorithm_);
  if (seed_generation_algorithm_ == "random_local")
  {
    ramndom_local_params_.max_range_weight_ = 0.01;
    ros::param::get(robot_on_guide_ns + "max_range_weight", ramndom_local_params_.max_range_weight_);
  }
  ROS_DEBUG("Created %s IK Solver: Algorithm %s Desired Solutions: %d, Max Stall Iter %d", robot_on_guide_ns.c_str(), seed_generation_algorithm_.c_str(), desired_solutions_, max_stall_iter_);

  return true;
}

Configurations RobotOnGuideIkSolver::getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds,
                                             const int& desired_solutions, const int& max_stall_iterations)
{
  bool stop = false;
  if (seed_generation_algorithm_ == "recursive")
  {
    return this->getIkSharedSeed(T_base_flange, seeds, desired_solutions, max_stall_iterations);
  }
  else  // if (seed_generation_algorithm_=="random_local")
  {
    return this->getIkProjectedSeed(T_base_flange, seeds, desired_solutions, max_stall_iterations);
  }
}


// ============================================================================================================
// ============================================================================================================
Configurations RobotOnGuideIkSolver::getIkSharedSeed(const Eigen::Affine3d& T_base_flange,
                                                       const Configurations& seeds, const int& desired_solutions,
                                                       const int& max_stall_iterations)
{
  Configurations solutions;
  solutions.clear();

  Configurations _robot_seeds;
  Configurations _guide_seeds;
  if(seeds.size())
  {
    filter_seeds(seeds, guide_.chain_->getActiveJointsNumber(), _guide_seeds, _robot_seeds);
  }

  for (size_t idx = 0; idx < (size_t)max_stall_iterations && robot_on_guide_nh_.ok(); idx++)
  {
    size_t seed_index = 0;
    Eigen::VectorXd robot_seed(attached_robot_->joint_names().size());
    Eigen::VectorXd guide_seed(guide_.chain_->getActiveJointsNumber());

    if(_guide_seeds.size() && idx < _guide_seeds.size() )
    {
      seed_index = idx % _guide_seeds.size();
      robot_seed = _robot_seeds.at(seed_index);
      guide_seed = _guide_seeds.at(seed_index);
    }

    if (_guide_seeds.size() == 0 || idx > _guide_seeds.size())
    {
      double r = double(idx) / double(max_stall_iterations);
      double usage_range = 0.1 * (1.0 - r) + 0.9 * (r);
      guide_seed = random(guide_seed, guide_.chain_->getQMin(), guide_.chain_->getQMax(), usage_range);
      robot_seed = random(robot_seed, attached_robot_->lb(), attached_robot_->ub(), usage_range);
    }

    Eigen::Affine3d T_base_robotbase = guide_.chain_->getTransformation(guide_seed);
    Eigen::Affine3d T_robotbase_flange = T_base_robotbase.inverse() * T_base_flange;

    Configurations robot_sol =
        attached_robot_->getIk(T_robotbase_flange, { robot_seed }, desired_solutions, max_stall_iterations);

    for (const Eigen::VectorXd& q_robot : robot_sol)
    {
      Eigen::VectorXd q_tot(guide_seed.size() + q_robot.size());
      q_tot << guide_seed, q_robot;
      solutions.push_back(q_tot);
    }

    if ((int)solutions.size() > desired_solutions)
    {
      break;
    }
  }

  return solutions;
}

Configurations RobotOnGuideIkSolver::getIkProjectedSeed(const Eigen::Affine3d& T_base_flange,
                                                          const Configurations& seeds, const int& desired_solutions,
                                                          const int& max_stall_iterations)
{
  Configurations solutions;
  solutions.clear();
  Configurations _robot_seeds;
  Configurations _guide_seeds;
  filter_seeds(seeds, guide_.chain_->getActiveJointsNumber(), _guide_seeds, _robot_seeds);

  Eigen::VectorXd _guide_q_seed_0 = guide_.chain_->getQMin() 
      + percentage_distance_from(T_base_flange, guide_.lb_pose_, guide_.ub_pose_) 
      * (guide_.range_);

  double range_weight = ramndom_local_params_.max_range_weight_;
  for (size_t idx = 0; idx < max_stall_iterations; idx++)
  {
    double range = 0.0;
    Eigen::VectorXd _guide_q_seed;
    if (idx == 0)
    {
      _guide_q_seed = _guide_q_seed_0;
    }
    else
    {
      range  = sin_interpolation(0, range_weight * guide_.range_.norm(), idx, max_stall_iterations);
      _guide_q_seed =
          local_random(_guide_q_seed_0, guide_.chain_->getQMin(), guide_.chain_->getQMax(), range);
    }
    if(_guide_q_seed.dot(guide_.range_.normalized())<guide_.chain_->getQMin().dot(guide_.range_.normalized())
    || _guide_q_seed.dot(guide_.range_.normalized())>guide_.chain_->getQMax().dot(guide_.range_.normalized()))
    {
      std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << ":" << idx << std::endl;
      std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << ":" << guide_.chain_->getQMin()  << std::endl;
      std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << ":" << guide_.chain_->getQMax()  << std::endl;
      std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << ":" << "percentage_distance_from: " << percentage_distance_from(T_base_flange, guide_.lb_pose_, guide_.ub_pose_) << std::endl;
      std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << ":" << "gude range: " << guide_.range_.transpose() << std::endl;
      std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << ":" << "range: " << range << std::endl;
      std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << ":" << "_guide_q_seed_0: " << _guide_q_seed_0.transpose() << std::endl;
      std::cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << ":" << "_guide_q_seed: " << _guide_q_seed.transpose() << std::endl;
      
      assert(0);
    }

    Eigen::Affine3d T_base_robotbase = guide_.chain_->getTransformation(_guide_q_seed);
    Eigen::Affine3d T_robotbase_flange = T_base_robotbase.inverse() * T_base_flange;

    Configurations robot_sol =
        attached_robot_->getIk(T_robotbase_flange, _robot_seeds, desired_solutions, max_stall_iterations);

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

    range_weight = robot_sol.size() ? ramndom_local_params_.max_range_weight_ / double(robot_sol.size()) :
                                      ramndom_local_params_.max_range_weight_;
  }

  return solutions;
}

Eigen::Affine3d RobotOnGuideIkSolver::getFK(const Eigen::VectorXd& s)
{
  Eigen::VectorXd s_guide = s.head(guide_.chain_->getActiveJointsNumber());
  Eigen::VectorXd s_robot = s.tail(s.size() - guide_.chain_->getActiveJointsNumber());
  Eigen::Affine3d T_base_robotbase = guide_.chain_->getTransformation(s_guide);
  Eigen::Affine3d T_robotbase_flange = attached_robot_->getFK(s_robot);
  return T_base_robotbase * T_robotbase_flange;
}
}  // end namespace ik_solver
