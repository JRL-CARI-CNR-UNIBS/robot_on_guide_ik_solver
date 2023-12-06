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


#include <ik_solver_core/types.h>
#include <ik_solver_core/ik_solver_base_class.h>

#include <pluginlib/class_list_macros.h>
#include <robot_on_guide_ik_solver/robot_on_guide_ik_solver.h>

PLUGINLIB_EXPORT_CLASS(ik_solver::RobotOnGuideIkSolver, ik_solver::IkSolver)
PLUGINLIB_EXPORT_CLASS(ik_solver::RobotOnGuideIkSolverConfigurator, ik_solver::IkSolverConfigurator)

namespace ik_solver
{

//==========================================================================================================================
bool RobotOnGuideIkSolverConfigurator::get_configuration(IkSolverOptionsPtr opts, const ros::NodeHandle& nh, const std::string& params_ns, std::string& what) 
{
  if (!IkSolverConfigurator::config(opts, nh, params_nswhat ))
  {
    return false;
  }

  ROS_INFO("Start creating %s IK Solver", robot_nh_.getNamespace().c_str());
  // ============================================================================
  attached_robot_nh_ = ros::NodeHandle(this->robot_nh_.getNamespace() + "/mounted_robot_ik");

  std::map<std::string, std::vector<std::string>> mandatory_params{
    { "/", {} },
    { nh.getNamespace(), { "mounted_robot_ik" } },
    { attached_robot_nh_.getNamespace(), { "type", "base_frame" } },
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
  ROS_INFO_STREAM("Guide base_frame: " << base_frame_);
  ROS_INFO_STREAM("Guide mounted_robot_base_frame: " << mounted_robot_base_frame);

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
  if (!ikloader_)
  {
    ikloader_.reset(new pluginlib::ClassLoader<ik_solver::IkSolver>("ik_solver", "ik_solver::IkSolver"));
  }

  ROS_INFO("%s creating ik plugin for mounted robot", attached_robot_ns.c_str());

  guide_.chain_->setInputJointsName(guide_names);

  ROS_INFO("%s creating ik plugin for mounted robot", attached_robot_ns.c_str());
  std::string plugin_name;
  ros::param::get(attached_robot_ns + "type", plugin_name);
  if (!attached_robot_)  //  in the case I call configure the second time this is not necessary
  {
    attached_robot_ = ikloader_->createInstance(plugin_name);
    plugin_name_ = plugin_name;
  }
  else
  {
    if (plugin_name != plugin_name_)
    {
      attached_robot_.reset();
      attached_robot_ = ikloader_->createInstance(plugin_name);
      plugin_name_ = plugin_name;
    }
  }

  ros::NodeHandle ik_solver_nh(robot_nh_, "robot_on_guide");
  if (!attached_robot_->config(ik_solver_nh, attached_robot_ns))  // all takes the data from the same naespace,
                                                                  // but they create services in private ns
  {
    ROS_ERROR("%s: error configuring mounted robot ik", attached_robot_ns.c_str());
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

}  // end namespace ik_solver
