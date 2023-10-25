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

#include <pluginlib/class_list_macros.h>
#include <robot_on_guide_ik_solver/robot_on_guide_ik_solver.h>

PLUGINLIB_EXPORT_CLASS(ik_solver::RobotOnGuideIkSolver, ik_solver::IkSolver)

namespace ik_solver
{
const std::string WHITESPACE = " \n\r\t\f\v";

std::string ltrim(const std::string& s, const std::string& what = WHITESPACE)
{
  size_t start = s.find_first_not_of(what);
  return (start == std::string::npos) ? "" : s.substr(start);
}

std::string rtrim(const std::string& s, const std::string& what = WHITESPACE)
{
  size_t end = s.find_last_not_of(what);
  return (end == std::string::npos) ? "" : s.substr(0, end + 1);
}

std::string trim(const std::string& s, const std::string& what = WHITESPACE)
{
  return rtrim(ltrim(s));
}

template <typename T>
bool is_the_same(const T& lhs, const T& rhs)
{
  return rhs == lhs;
}

template <>
bool is_the_same(const std::vector<std::string>& lhs, const std::vector<std::string>& rhs)
{
  if (lhs.size() != rhs.size())
  {
    return false;
  }

  for (size_t i = 0; i < lhs.size(); i++)
  {
    if (lhs.at(i) != rhs.at(i))
    {
      return false;
    }
  }
  return true;
}

std::string to_string(const std::string& s)
{
  return s;
}

std::string to_string(const std::vector<std::string>& ss)
{
  std::string ret = "[";
  for (const auto& s : ss)
    ret += s + ",";
  return ret + "]";
}

template <typename T>
bool check_and_set_duplicate_params(const std::string& nh_ns, const std::string& robot_ns,
                                    const std::string& param_name, const T& nh_val)
{
  std::string pn = robot_ns + param_name;
  T val;
  if (ros::param::get(pn, val))
  {
    if (!is_the_same(val, nh_val))
    {
      ROS_ERROR(
          "%s has been found both in the Composed Robot namespace (%s) "
          "and in the Mounted Robot namespace (%s) but they are "
          "different (%s and %s). Put it only in the Composed Robot "
          "namespace, or alternatuively be sure they are the same!",
          pn.c_str(), nh_ns.c_str(), robot_ns.c_str(), to_string(val).c_str(), to_string(nh_val).c_str());
      return false;
    }
  }
  else
  {
    ros::param::set(pn, nh_val);
  }
  return true;
}

bool order_joint_names(std::vector<std::string>& joint_names, std::vector<std::string>& guide_names,
                       std::vector<std::string>& robot_names)
{
  // std::vector<std::string> guide_names = guide_chain_->getActiveJointsName();
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

  ROS_INFO("Robot Chain has %zu DOF, Guide Chain has %zu DOF - %s", robot_names.size(), ordered_guide_names.size(),
           jns.c_str());
  return true;
}

double percentage_distance_from(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Vector3d& v)
{
  Eigen::Vector3d d = p.translation() - lb.translation();
  return d.dot(v.normalized()) / v.norm();
}

Eigen::Vector3d distance_from(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Vector3d& v)
{
  Eigen::Vector3d d = p.translation() - lb.translation();
  return d.dot(v.normalized()) * v.normalized();
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
  auto dd = (ub - lb) * (1.0 - std::cos(double(i_step) / double(n_steps) * M_PI / 2.0));
  return lb + dd;
}

void filter_seeds(const IkConfigurations& full_seeds, const size_t& guide_seed_dof, IkConfigurations& guide_seeds,
                  IkConfigurations& robot_seeds)
{
  IkConfigurations ret;
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

// =========================================================================================================================
const rosdyn::ChainPtr& RobotOnGuideIkSolver::guide_chain(const size_t& i) const
{
  return mt_.at(i).guide_chain_;
}
const boost::shared_ptr<ik_solver::IkSolver>& RobotOnGuideIkSolver::robot_ik_solver(const size_t& i) const
{
  return mt_.at(i).robot_ik_solver_;
}
const Eigen::VectorXd& RobotOnGuideIkSolver::guide_lb(const size_t& i) const
{
  return mt_.at(i).guide_lb_;
}
const Eigen::VectorXd& RobotOnGuideIkSolver::guide_ub(const size_t& i) const
{
  return mt_.at(i).guide_ub_;
}

rosdyn::ChainPtr& RobotOnGuideIkSolver::guide_chain(const size_t& i)
{
  return mt_.at(i).guide_chain_;
}
boost::shared_ptr<ik_solver::IkSolver>& RobotOnGuideIkSolver::robot_ik_solver(const size_t& i)
{
  return mt_.at(i).robot_ik_solver_;
}
Eigen::VectorXd& RobotOnGuideIkSolver::guide_lb(const size_t& i)
{
  return mt_.at(i).guide_lb_;
}
Eigen::VectorXd& RobotOnGuideIkSolver::guide_ub(const size_t& i)
{
  return mt_.at(i).guide_ub_;
}


const Eigen::Affine3d& RobotOnGuideIkSolver::guide_lb_pose(const size_t& i) const
{
  return mt_.at(i).guide_lb_pose_;
}
const Eigen::Affine3d& RobotOnGuideIkSolver::guide_ub_pose(const size_t& i) const
{
  return mt_.at(i).guide_ub_pose_;
}

Eigen::Affine3d& RobotOnGuideIkSolver::guide_lb_pose(const size_t& i)
{
  return mt_.at(i).guide_lb_pose_;
}
Eigen::Affine3d& RobotOnGuideIkSolver::guide_ub_pose(const size_t& i)
{
  return mt_.at(i).guide_ub_pose_;
}

//==========================================================================================================================
inline bool RobotOnGuideIkSolver::customConfig()
{
  ROS_DEBUG("Start creating %s IK Solver", nh_.getNamespace().c_str());

  // ============================================================================
  robot_nh_ = ros::NodeHandle(nh_.getNamespace() + "/mounted_robot_ik");

  std::map<std::string, std::vector<std::string>> mandatory_params{
    { "/", {} },
    { nh_.getNamespace(), { "mounted_robot_ik", "seed_generation_algorithm" } },
    { robot_nh_.getNamespace(), { "type", "base_frame" } }
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
  std::string nh_ns = rtrim(nh_.getNamespace(), "/") + "/";
  std::string robot_ns = rtrim(robot_nh_.getNamespace(), "/") + "/";

  std::map<std::string, std::string> _frames{
    { "flange_frame", flange_frame_ },
    { "tool_frame", tool_frame_ },
  };

  for (const auto& _f : _frames)
  {
    if (!check_and_set_duplicate_params(nh_ns, robot_ns, _f.first, _f.second))
    {
      return false;
    }
  }

  // INIT GUIDE CHAIN
  std::string mounted_robot_base_frame;
  ros::param::get(robot_ns + "base_frame", mounted_robot_base_frame);

  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;

  mt_.resize(IkSolver::MAX_NUM_THREADS);
  for (auto& t : mt_)
  {
    t.guide_chain_ = rosdyn::createChain(model_, base_frame_, mounted_robot_base_frame, gravity);
  }

  std::vector<std::string> guide_names = guide_chain()->getActiveJointsName();
  std::vector<std::string> robot_names{};
  if (order_joint_names(joint_names_, guide_names, robot_names))
  {
    // reorder in the param array
    ros::param::set(nh_ns + "/joint_names", joint_names_);
    ros::param::set(robot_ns + "/joint_names", guide_names);
  }
  else
  {
    ROS_ERROR("Not all the guides names are listed in %s/joint_names", nh_.getNamespace().c_str());
    return false;
  }

  // INIT MOUNTED ROBOT
  std::string plugin_name;
  ros::param::get(robot_ns + "type", plugin_name);
  ikloader_.reset(new pluginlib::ClassLoader<ik_solver::IkSolver>("ik_solver", "ik_solver::IkSolver"));

  ROS_DEBUG("%s creating ik plugin for mounted robot", robot_ns.c_str());

  for (size_t i = 0; i < mt_.size(); i++)
  {
    mt_.at(i).running_ = false;
    mt_.at(i).guide_chain_->setInputJointsName(guide_names);

    ROS_DEBUG("%s creating ik plugin for mounted robot", robot_ns.c_str());
    mt_.at(i).robot_ik_solver_ = ikloader_->createInstance(plugin_name);

    ros::NodeHandle ik_solver_nh(robot_nh_, "mt" + std::to_string(i));
    if (!mt_.at(i).robot_ik_solver_->config(ik_solver_nh, robot_ns))  // all takes the data from the same naespace,
                                                                      // but they create services in private ns
    {
      ROS_ERROR("%s: error configuring mounted robot ik", nh_.getNamespace().c_str());
      return false;
    }

    mt_.at(i).guide_lb_.resize(guide_names.size());
    mt_.at(i).guide_ub_.resize(guide_names.size());
    for (size_t k = 0; k < guide_names.size(); k++)
    {
      auto guide_jns = mt_.at(i).guide_chain_->getActiveJointsName();
      auto it = std::find(guide_jns.begin(), guide_jns.end(), guide_names.at(k));
      if (it != guide_names.end())
      {
        size_t ji = std::distance(guide_names.begin(), it);
        mt_.at(i).guide_lb_(k) = mt_.at(i).guide_chain_->getQMin()(ji);
        mt_.at(i).guide_ub_(k) = mt_.at(i).guide_chain_->getQMax()(ji);
      }
    }
    mt_.at(i).guide_lb_pose_ = mt_.at(i).guide_chain_->getTransformation(mt_.at(i).guide_lb_);
    mt_.at(i).guide_ub_pose_ = mt_.at(i).guide_chain_->getTransformation(mt_.at(i).guide_ub_);
  }

  // =================================

  // FNISH GUIDE SETUP
  // ======================================================
  seed_generation_algorithm_ = "recursive";
  ros::param::get(nh_ns + "seed_generation_algorithm", seed_generation_algorithm_);
  if (seed_generation_algorithm_ == "random_local")
  {
    ramndom_local_params_.max_range_weight_ = 0.01;
    ros::param::get(nh_ns + "max_range_weight", ramndom_local_params_.max_range_weight_);
  }

  ROS_DEBUG("Created %s IK Solver", nh_.getNamespace().c_str());

  return true;
}

IkConfigurations RobotOnGuideIkSolver::getIk(const Eigen::Affine3d& T_base_flange, const IkConfigurations& seeds,
                                             const int& desired_solutions, const int& max_stall_iterations)
{
  bool stop = false;
  if (seed_generation_algorithm_ == "recursive")
  {
    return this->getIkSharedSeed(stop, 0, T_base_flange, seeds, desired_solutions, max_stall_iterations);
  }
  else  // if (seed_generation_algorithm_=="random_local")
  {
    return this->getIkProjectedSeed(stop, 0, T_base_flange, seeds, desired_solutions, max_stall_iterations);
  }
}

IkConfigurations RobotOnGuideIkSolver::getIkSafeMT(bool& stop, const Eigen::Affine3d& T_base_flange,
                                                   const IkConfigurations& seeds, const int& desired_solutions,
                                                   const int& max_stall_iterations)
{
  while (true)
  {
    for (size_t thread_id = 0; thread_id < IkSolver::MAX_NUM_THREADS; thread_id++)
    {
      if (!mt_.at(thread_id).running_)
      {
        mt_.at(thread_id).running_ = true;
        mt_.at(thread_id).solutions_ =
            this->getIkSafeMT(stop, thread_id, T_base_flange, seeds, desired_solutions, max_stall_iterations);
        mt_.at(thread_id).running_ = false;
        return mt_.at(thread_id).solutions_;
      }
    }
  }
}

IkConfigurations RobotOnGuideIkSolver::getIkSafeMT(bool& stop, const size_t& thread_id,
                                                   const Eigen::Affine3d& T_base_flange, const IkConfigurations& seeds,
                                                   const int& desired_solutions, const int& max_stall_iterations)
{
  if (seed_generation_algorithm_ == "recursive")
  {
    return this->getIkSharedSeed(stop, 0, T_base_flange, seeds, desired_solutions, max_stall_iterations);
  }
  else  // if (seed_generation_algorithm_=="random_local")
  {
    return this->getIkProjectedSeed(stop, 0, T_base_flange, seeds, desired_solutions, max_stall_iterations);
  }
}

// ============================================================================================================
// ============================================================================================================
IkConfigurations RobotOnGuideIkSolver::getIkSharedSeed(bool& stop, const size_t& thread_id,
                                                       const Eigen::Affine3d& T_base_flange,
                                                       const IkConfigurations& seeds, const int& desired_solutions,
                                                       const int& max_stall_iterations)
{
  IkConfigurations solutions;
  solutions.clear();

  IkConfigurations _robot_seeds;
  IkConfigurations _guide_seeds;
  filter_seeds(seeds, guide_chain(thread_id)->getActiveJointsNumber(), _guide_seeds, _robot_seeds);

  for (size_t idx = 0; idx < (size_t)max_stall_iterations && nh_.ok(); idx++)
  {
    size_t seed_index = idx % _guide_seeds.size();

    Eigen::VectorXd robot_seed = _robot_seeds.at(seed_index);
    Eigen::VectorXd guide_seed = _guide_seeds.at(seed_index);

    if (idx > _guide_seeds.size())
    {
      double r = double(idx) / double(max_stall_iterations);
      double usage_range = 0.1 * (1.0 - r) + 0.9 * (r);
      guide_seed =
          random(guide_seed, guide_chain(thread_id)->getQMin(), guide_chain(thread_id)->getQMax(), usage_range);
      robot_seed = random(robot_seed, guide_lb(thread_id), guide_ub(thread_id), usage_range);
    }

    Eigen::Affine3d T_base_robotbase = guide_chain(thread_id)->getTransformation(guide_seed);
    Eigen::Affine3d T_robotbase_flange = T_base_robotbase.inverse() * T_base_flange;

    IkConfigurations robot_sol =
        robot_ik_solver(thread_id)->getIk(T_robotbase_flange, { robot_seed }, desired_solutions, max_stall_iterations);

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

IkConfigurations RobotOnGuideIkSolver::getIkProjectedSeed(bool& stop, const size_t& thread_id,
                                                          const Eigen::Affine3d& T_base_flange,
                                                          const IkConfigurations& seeds, const int& desired_solutions,
                                                          const int& max_stall_iterations)
{
  IkConfigurations solutions;
  solutions.clear();
  IkConfigurations _robot_seeds;
  IkConfigurations _guide_seeds;
  filter_seeds(seeds, guide_chain(thread_id)->getActiveJointsNumber(), _guide_seeds, _robot_seeds);

  Eigen::VectorXd _guide_range = guide_ub_pose(thread_id).translation() - guide_lb_pose(thread_id).translation();
  Eigen::VectorXd _guide_q_seed_0 =
      mt_.at(thread_id).guide_lb_ + distance_from(T_base_flange, guide_lb_pose(thread_id), _guide_range);

  double range_weight = ramndom_local_params_.max_range_weight_;
  for (size_t idx = 0; idx < max_stall_iterations; idx++)
  {
    Eigen::VectorXd _guide_q_seed;
    if (idx == 0)
    {
      _guide_q_seed = _guide_q_seed_0;
    }
    else
    {
      double range = sin_interpolation(0, range_weight * _guide_range.norm(), idx, max_stall_iterations);
      _guide_q_seed =
          local_random(_guide_q_seed_0, guide_chain(thread_id)->getQMin(), guide_chain(thread_id)->getQMax(), range);
    }

    Eigen::Affine3d T_base_robotbase = guide_chain(thread_id)->getTransformation(_guide_q_seed);
    Eigen::Affine3d T_robotbase_flange = T_base_robotbase.inverse() * T_base_flange;

    IkConfigurations robot_sol =
        robot_ik_solver(thread_id)->getIk(T_robotbase_flange, _robot_seeds, desired_solutions, max_stall_iterations);

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
  Eigen::VectorXd s_guide = s.head(guide_chain()->getActiveJointsNumber());
  Eigen::VectorXd s_robot = s.tail(s.size() - guide_chain()->getActiveJointsNumber());
  Eigen::Affine3d T_base_robotbase = guide_chain()->getTransformation(s_guide);
  Eigen::Affine3d T_robotbase_flange = robot_ik_solver()->getFK(s_robot);
  return T_base_robotbase * T_robotbase_flange;
}
}  // end namespace ik_solver
