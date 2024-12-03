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
#include <cstddef>
#include <cstring>
#include <vector>
#include <Eigen/Geometry>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/AngleAxis.h"
#include "ik_solver/internal/types.h"
#include <ik_solver_core/ik_solver_base_class.h>

#include <pluginlib/class_list_macros.hpp>
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
    RCLCPP_ERROR(rclcpp::get_logger("RobotOnGuide-OrderJointNames"), "Guide names: ");
    for (std::string& s : guide_names)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotOnGuide-OrderJointNames"), " - %s", s.c_str());
    }
    RCLCPP_ERROR(rclcpp::get_logger("RobotOnGuide-OrderJointNames"), "joint_names: ");
    for (std::string& s : joint_names)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotOnGuide-OrderJointNames"), " - %s", s.c_str());
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

  RCLCPP_INFO(rclcpp::get_logger("RobotOnGuide-OrderJointNames"), "Robot Chain has %zu DOF, Guide Chain has %zu DOF - %s", robot_names.size(), ordered_guide_names.size(),
           jns.c_str());
  return true;
}

double percentage_distance_from(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Affine3d& ub)
{
  Eigen::Vector3d v = ub.translation() - lb.translation();
  Eigen::Vector3d d = p.translation() - lb.translation();
  double dv = d.dot(v.normalized()) / v.norm();
  return dv > 1.0 ? 1.0 : dv < 0.0 ? 0.0 : dv;
}

double percentage_distance_from(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Vector3d& v)
{
  Eigen::Vector3d d = p.translation() - lb.translation();
  double dv = d.dot(v.normalized()) / v.norm();
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
  return (p.dot(uv) > ub.dot(uv)) ? ub : (p.dot(uv) < lb.dot(uv)) ? lb : p;
}

Eigen::VectorXd local_random(const Eigen::VectorXd& p, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                             double range_percentage)
{
  Eigen::MatrixXd eye = Eigen::VectorXd(p.size()).setOnes().asDiagonal();
  Eigen::MatrixXd rand = Eigen::VectorXd(p.size()).setRandom().asDiagonal();

  return force_inbound(lb + range_percentage * 0.5 * (eye + rand) * (ub - lb), lb, ub);
}

double linear_interpolation(const double& lb, const double& ub, const size_t& i_step, const size_t& n_steps)
{
  double dd = (ub - lb) / double(n_steps);
  return lb + i_step * dd;
}

double half_sin_interpolation(const double& lb, const double& ub, const size_t& i_step, const size_t& n_steps)
{
  assert(ub - lb);
  auto dd = (ub - lb) * (1.0 - std::cos(double(i_step) / double(n_steps) * M_PI)) / 2.0;
  return lb + dd;
}

double symmetric_centered_sampler(const double& s0, const double& symmetric_delta, const size_t& i_step,
                                  const size_t& n_steps)
{
  int _i_step = (i_step % 2 == 0) ? -i_step / 2 : (i_step + 1 / 2);
  int _n_steps = n_steps / 2;
  double delta = std::fabs(symmetric_delta);
  double step = delta / _n_steps;

  return s0 + _i_step * step;
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

bool project_target_reaching(Eigen::Vector3d& target_intersection, const double& target_reaching,
                             const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Affine3d& ub,
                             const Eigen::Vector3d& AX0)
{
  bool ok = ik_solver::cylinder_ray_intersection(target_intersection, p, lb, ub, target_reaching, AX0, true);
  if (!ok)
  {
  }

  double s = 0.0;
  // Project ub and lp using AX0
  Eigen::Vector3d v = ub.translation() - lb.translation();
  Eigen::Vector3d uv = v.normalized();
  Eigen::Vector3d uz = AX0.normalized();
  Eigen::Vector3d ux = uv.cross(uz).normalized();
  Eigen::Vector3d uy = (uz.cross(uy)).normalized();

  Eigen::Vector3d lp = p.translation() - lb.translation();
  if (lp.dot(uv) < 0)
  {
    s = 0.0;
  }
  else if (lp.dot(uv) >= v.norm())
  {
    s = 1.0;
  }
  else
  {
    Eigen::Vector3d A = lb.translation();  // lower bound it the starting point
    Eigen::Vector3d B = v.dot(uy) * uy;    // B is the projection on a vector orthogonal to guide ed AX0
    Eigen::Vector3d P = p.translation();
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d AP = P - A;
    double p_x = AP.dot(ux);
    double p_y = AP.dot(uy);
    double ab = AB.norm();
    if (std::fabs(p_x) > target_reaching)
    {
      s = p_y / ab;
    }
    else
    {
      double dy = std::sqrt(target_reaching * target_reaching - p_y * p_y);
      s = (p_y - dy) / ab;
    }
  }

  return s < 0.0 ? 0.0 : s > 1.0 ? 1.0 : s;
}

Eigen::Vector3d project(const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& u)
{
  return A + (P - A).dot(u) * u;
}

Eigen::Vector3d project(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Affine3d& ub)
{
  return project(p.translation(), lb.translation(), (ub.translation() - lb.translation()).normalized());
}

double axes_distance(const Eigen::Vector3d& V, const Eigen::Vector3d& uv, const Eigen::Vector3d& R,
                     const Eigen::Vector3d& ur)
{
  double dist = 0.0;
  if (std::fabs(std::fabs(uv.dot(ur)) - 1.0) > 1e-4)  // uv and ur are not parallel
  {
    Eigen::Vector3d u_vr = uv.cross(ur).normalized();
    dist = std::fabs((R - V).dot(u_vr));
  }
  else  // uv and ur are parallel
  {
    Eigen::Vector3d uvr = (R - V).normalized();
    Eigen::Vector3d ux = uv.cross(uvr).normalized();
    Eigen::Vector3d uy = uv.cross(ux).normalized();
    dist = std::fabs((R - V).dot(uy));
  }
  return dist;
}

/**
 * @brief
 *
 * @param K
 * @param P
 * @param A
 * @param uv
 * @param r
 * @param cylinder_ax
 * @param closest_to_lb
 * @return true
 * @return false
 */
bool cylinder_ray_intersection(Eigen::Vector3d& K, const Eigen::Vector3d& P, const Eigen::Vector3d& A,
                               const Eigen::Vector3d& ray, const double& r, const Eigen::Vector3d& cylinder_ax,
                               bool closest_to_lb)
{
  Eigen::Vector3d uv = ray.normalized();
  Eigen::Vector3d uz = cylinder_ax.normalized();
  Eigen::Vector3d ux = uv.cross(uz).normalized();
  Eigen::Vector3d uy = (uz.cross(ux)).normalized();

  Eigen::Vector3d AP = (P - A);
  double ap_x = AP.dot(ux);
  if (std::fabs(ap_x) > r)
  {
    return false;
  }

  double ap_y = AP.dot(uy);
  double ap_z = AP.dot(uz);

  double delta = r * r - ap_x * ap_x;

  Eigen::Vector3d AP_zy = (ap_y * uy + ap_z * uz);
  Eigen::Vector3d AK_zy = AP_zy + (closest_to_lb ? -1 : 1) * std::sqrt(delta) * uy;

  Eigen::Vector3d AK = AK_zy.dot(uv) * uv;

  K = A + AK;
  return true;
}

/**
 * @brief
 *
 * @param K
 * @param p
 * @param lb
 * @param ub
 * @param r
 * @param cylinder_ax
 * @param closest_to_lb
 * @return true
 * @return false
 */
bool cylinder_ray_intersection(Eigen::Vector3d& K, const Eigen::Affine3d& p, const Eigen::Affine3d& lb,
                               const Eigen::Affine3d& ub, const double& r, const Eigen::Vector3d& cylinder_ax,
                               bool closest_to_lb)
{
  return cylinder_ray_intersection(K, p.translation(), lb.translation(),
                                   (ub.translation() - lb.translation()).normalized(), r, cylinder_ax, closest_to_lb);
}

/**
 * @brief
 *
 * @param P
 * @param A
 * @param uv
 * @param cylinder_ax
 * @return double
 */
double compute_polar_reaching(const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& uv,
                              const Eigen::Vector3d& cylinder_ax)
{
  Eigen::Vector3d uz = cylinder_ax.normalized();
  Eigen::Vector3d ux = uv.cross(uz).normalized();
  Eigen::Vector3d uy = (uz.cross(ux)).normalized();

  return (P - A).dot(ux);
}

double compute_polar_reaching(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Affine3d& ub,
                              const Eigen::Vector3d& cylinder_ax)
{
  return compute_polar_reaching(p.translation(), lb.translation(), ub.translation(), cylinder_ax);
}

//==========================================================================================================================
inline bool RobotOnGuideIkSolver::config(const std::string& params_ns)
{
  if (!IkSolver::config(params_ns))
  {
    return false;
  }

  // ============================================================================
  robot_on_guide_ns_ = params_ns;
//  attached_robot_nh_ = ros::NodeHandle(this->robot_nh_.getNamespace() + "/mounted_robot_ik");
  attached_robot_ns_ = params_ns_.append("/mounted_robot_ik");

  CNR_INFO(logger_, "Start creating %s IK Solver", robot_on_guide_ns_.c_str());

  std::map<std::string, std::vector<std::string>> mandatory_params{
    { "/", {} },
    { robot_on_guide_ns_, { "mounted_robot_ik" } },
    { attached_robot_ns_, { "type", "base_frame" } },
  };

  std::string what;
  for (const auto& pp : mandatory_params)
  {
    std::string ns = rtrim(trim(pp.first), "/") + "/";
    for (const auto& p : pp.second)
    {
      std::string pn = ns + ltrim(p, "/");
      if (!cnr::param::has(ns + p, what))
      {
        CNR_ERROR(logger_, "%s not found in rosparam server", pn.c_str());
        return false;
      }
    }
  }

  std::string root_ns = "/";
  std::string robot_on_guide_ns = rtrim(robot_on_guide_ns_, "/") + "/";
  std::string attached_robot_ns = rtrim(attached_robot_ns_, "/") + "/";

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

  CNR_INFO(logger_, "%s creating ik plugin for mounted robot", attached_robot_ns.c_str());
  std::string plugin_name;
  cnr::param::get(attached_robot_ns + "type", plugin_name, what);

  // INIT GUIDE CHAIN
  std::string mounted_robot_base_frame;
  cnr::param::get(attached_robot_ns + "base_frame", mounted_robot_base_frame, what);

  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;

  guide_.chain_ = rdyn::createChain(*model_, base_frame_, mounted_robot_base_frame, gravity);

  std::vector<std::string> guide_names = guide_.chain_->getActiveJointsName();
  CNR_INFO(logger_, "Guide base_frame: " << base_frame_);
  CNR_INFO(logger_, "Guide mounted_robot_base_frame: " << mounted_robot_base_frame);

  std::vector<std::string> robot_names{};
  if (order_joint_names(joint_names_, guide_names, robot_names))
  {
    // reorder in the param array
    cnr::param::set(robot_on_guide_ns + "/joint_names", joint_names_, what);
    cnr::param::set(attached_robot_ns + "/joint_names", robot_names , what);
  }
  else
  {
    CNR_ERROR(logger_, "Not all the guides names are listed in %s/joint_names", robot_on_guide_ns.c_str());
    return false;
  }

  // INIT MOUNTED ROBOT
  if (!ikloader_)
  {
    ikloader_.reset(new pluginlib::ClassLoader<ik_solver::IkSolver>("ik_solver", "ik_solver::IkSolver"));
  }


  guide_.chain_->setInputJointsName(guide_names, what);


  if (!attached_robot_)  //  in the case I call configure the second time this is not necessary
  {
    attached_robot_ = ikloader_->createUniqueInstance(plugin_name);
    plugin_name_ = plugin_name;
  }
  else
  {
    if (plugin_name != plugin_name_)
    {
      attached_robot_.reset();
      attached_robot_ = ikloader_->createUniqueInstance(plugin_name);
      plugin_name_ = plugin_name;
    }
  }
  CNR_INFO(logger_, "%s created ik plugin for mounted robot", attached_robot_ns.c_str());

//  ros::NodeHandle ik_solver_nh(robot_nh_, "robot_on_guide");
  const std::string ik_solver_ns {robot_on_guide_ns_ + "/" + "robot_on_guide"};
  attached_robot_->setBuffer(tf_buffer_);

  std::string param_what;
  if(!cnr::param::set(attached_robot_ns+std::string("/robot_description"), robot_description_, param_what))
  {
    CNR_ERROR(logger_, "Cannot set cnr::param(" << attached_robot_ns << std::string("/robot_description") << ") because: " << param_what);
  }
  CNR_DEBUG(logger_, "set robot_description as cnr::param");

  if (!attached_robot_->config(attached_robot_ns))  // all takes the data from the same naespace,
                                                                  // but they create services in private ns
  {
    CNR_ERROR(logger_, "%s: error configuring mounted robot ik", attached_robot_ns.c_str());
    return false;
  }

  guide_.jstroke_ = guide_.chain_->getQMax() - guide_.chain_->getQMin();
  guide_.jax_ = guide_.jstroke_.normalized();
  guide_.le_ = guide_.chain_->getTransformation(guide_.chain_->getQMin());
  guide_.ue_ = guide_.chain_->getTransformation(guide_.chain_->getQMax());
  guide_.pstroke_ = guide_.ue_.translation() - guide_.le_.translation();
  guide_.pax_ = guide_.pstroke_.normalized();

  // ======================================================
  // FNISH SETUP
  // ======================================================
  max_seek_range_m_ = 0.01;
  cnr::param::get(robot_on_guide_ns + "max_exploration_range", max_seek_range_m_, what);

  target_reaching_ = 2.0;
  cnr::param::get(robot_on_guide_ns + "target_reaching", target_reaching_, what);

  CNR_INFO(logger_, "%s configured", attached_robot_ns.c_str());
  return true;
}

Solutions RobotOnGuideIkSolver::getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds,
                                      const int& desired_solutions, const int& min_stall_iterations,
                                      const int& max_stall_iterations)
{
  Solutions solutions;
  solutions.clear();

  Configurations _robot_seeds;
  Configurations _guide_seeds;
  if (seeds.size())
  {
    filter_seeds(seeds, guide_.chain_->getActiveJointsNumber(), _guide_seeds, _robot_seeds);
  }

  Eigen::Vector3d Ax0 = Eigen::Vector3d::UnitZ();

  Eigen::Vector3d ptarget;
  bool ok = ik_solver::cylinder_ray_intersection(ptarget, T_base_flange, guide_.le_, guide_.ue_, target_reaching_, Ax0, true);
  if (!ok)
  {
    ptarget = ik_solver::project(T_base_flange, guide_.le_, guide_.ue_);
  }
  double starget0 = (ptarget - guide_.le_.translation()).dot(guide_.pax_) /
                    guide_.pstroke_.norm();  // curvilinear abscissa in Cartesian Space
  starget0 = starget0 < 0 ? 0 : starget0 > 1.0 ? 1.0 : starget0;
  Configuration jtarget = guide_.chain_->getQMin() + starget0 * guide_.jstroke_;

  int _desired_solutions = desired_solutions == -1 ? desired_solutions_ : desired_solutions;
  int _min_stall_iterations = min_stall_iterations == -1 ? min_stall_iter_ : min_stall_iterations;
  int _max_stall_iterations = max_stall_iterations == -1 ? max_stall_iter_ : max_stall_iterations;

  std::vector<double> ray;
  size_t idx = 0;
  for (idx = 0; idx < (size_t)_max_stall_iterations; idx++)
  {
    Eigen::VectorXd robot_seed(attached_robot_->joint_names().size());
    Eigen::VectorXd guide_seed(guide_.chain_->getActiveJointsNumber());

    if (_guide_seeds.size() && idx < _guide_seeds.size())
    {
      size_t seed_index = idx % _guide_seeds.size();
      robot_seed = _robot_seeds.at(seed_index);
      guide_seed = _guide_seeds.at(seed_index);
    }

    if (_guide_seeds.size() == 0 || idx > _guide_seeds.size())
    {
      double seek = 0.0;
      if (idx == 0 || idx == _guide_seeds.size())
      {
        guide_seed = jtarget;
      }
      else
      {
        double starget = symmetric_centered_sampler(starget0, 0.5 * max_seek_range_m_ / guide_.pstroke_.norm(), idx,
                                                    max_stall_iterations);

        guide_seed = guide_.chain_->getQMin() + starget * guide_.jstroke_;
        guide_seed = force_inbound(guide_seed, guide_.chain_->getQMin(), guide_.chain_->getQMax());
      }
    }

    Eigen::Affine3d T_base_robotbase = guide_.chain_->getTransformation(guide_seed);
    Eigen::Affine3d T_robotbase_flange = T_base_robotbase.inverse() * T_base_flange;

    Solutions robot_sol = attached_robot_->getIk(T_robotbase_flange, { robot_seed }, _desired_solutions,
                                                 _min_stall_iterations, _max_stall_iterations);
    for (const Configuration& q_robot : robot_sol.configurations())
    {
      Configuration q_tot(guide_seed.size() + q_robot.size());
      q_tot << guide_seed, q_robot;
      if (!ik_solver::isPresent(q_tot, solutions.configurations(), 2e-3))
      {
        auto T0f = this->getFK(q_tot);
        Eigen::AngleAxisd aa;
        aa = (T_base_flange.linear().inverse() * T0f.linear()).matrix();

        solutions.translation_residuals().push_back((T_base_flange.translation() - T0f.translation()).norm());
        solutions.rotation_residuals().push_back(aa.angle());

        auto T0b = this->guide()->getTransformation(guide_seed);
        solutions.configurations().push_back(q_tot);

        auto robot_base = guide()->getTransformation(guide_seed);
        auto dist = ik_solver::axes_distance(T_base_flange.translation(), Eigen::Vector3d::UnitZ(),
                                             robot_base.translation(), Eigen::Vector3d::UnitZ());
        ray.push_back(dist);
      }
    }

    if ((int)solutions.configurations().size() >= _desired_solutions)
    {
      break;
    }

    if (((int)solutions.configurations().size() > 0) && idx > _min_stall_iterations)
    {
      break;
    }
  }

  double max_tre =
      solutions.translation_residuals().size() ?
          *std::max_element(solutions.translation_residuals().begin(), solutions.translation_residuals().end()) :
          0.0;
  double max_rre = solutions.rotation_residuals().size() ?
                       *std::max_element(solutions.rotation_residuals().begin(), solutions.rotation_residuals().end()) :
                       0.0;
  double max_ray = ray.size() ? *std::max_element(ray.begin(), ray.end()) : 0.0;
  double min_ray = ray.size() ? *std::min_element(ray.begin(), ray.end()) : 0.0;

  solutions.message() = "It. " + std::to_string(idx + 1) + "(" + std::to_string(_min_stall_iterations) + "-" +
                        std::to_string(_max_stall_iterations) + ")" + "Seeds n. " + std::to_string(seeds.size()) +
                        ", Ray " + std::to_string(min_ray) + "-" + std::to_string(max_ray) + "(" +
                        std::to_string(target_reaching_) + ")";

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
