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

#include <urdf/model.h>

#include <Eigen/Geometry>

#include <ik_solver_core/types.h>
#include <ik_solver_core/utils.h>
#include <ik_solver_core/ik_solver_base_class.h>

#include <robot_on_guide_ik_solver/math.h>
#include <robot_on_guide_ik_solver/robot_on_guide_ik_solver.h>


namespace ik_solver
{
  
//==========================================================================================================================
bool RobotOnGuideIkSolver::config(IkSolverOptionsConstPtr opts, std::string& what)
{
 
  if (!IkSolver::config(opts, what ))
  {
    return false;
  }
  
  opts_ = std::dynamic_pointer_cast<const RobotOnGuideIkSolverOptions>(opts);


  // INIT GUIDE CHAIN
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;

  urdf::Model model;
  model.initString(opts_->robot_description_xmlstring_);

  guide_.chain_ = rosdyn::createChain(model, base_frame_, opts_->mounted_robot_base_frame_, gravity);

  std::vector<std::string> guide_names = guide_.chain_->getActiveJointsName();
  ROS_INFO_STREAM("Guide base_frame: " << base_frame_);
  ROS_INFO_STREAM("Guide opts_->mounted_robot_base_frame_: " << opts_->mounted_robot_base_frame_);

  // INIT MOUNTED ROBOT
  if (!ikloader_)
  {
    ikloader_.reset(new pluginlib::ClassLoader<ik_solver::IkSolver>("ik_solver", "ik_solver::IkSolver"));
  }

  guide_.chain_->setInputJointsName(opts_->guide_names_);

  if (!attached_robot_)  //  in the case I call configure the second time this is not necessary
  {
    attached_robot_ = ikloader_->createInstance(opts_->attached_robot_plugin_name_);
  }
  else
  {
    attached_robot_.reset();
    attached_robot_ = ikloader_->createInstance(opts_->attached_robot_plugin_name_);
  }

  if (!attached_robot_->config(opts_->attached_robot_options_, what))   {
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
  ros::param::get(robot_on_guide_ns + "max_exploration_range", max_seek_range_m_);

  target_reaching_ = 2.0;
  ros::param::get(robot_on_guide_ns + "target_reaching", target_reaching_);

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
  bool ok =
      ik_solver::cylinder_ray_intersection(ptarget, T_base_flange, guide_.le_, guide_.ue_, target_reaching_, Ax0, true);
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
