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

PLUGINLIB_EXPORT_CLASS(ik_solver::RobotOnGuideIkSolver, ik_solver::IkSolver)

namespace ik_solver
{

  inline bool RobotOnGuideIkSolver::customConfig()
  {

    ROS_DEBUG("Start creating %s IK Solver",nh_.getNamespace().c_str());
    if (!nh_.hasParam(nh_.getNamespace()+"/mounted_robot_ik"))
    {
      ROS_INFO("%s/robot_ik is not defined",nh_.getNamespace().c_str());
      return false;
    }

    std::string robot_base_frame;
    if (!nh_.getParam("robot_base_frame",robot_base_frame))
    {
      ROS_ERROR("%s/robot_base_frame is not specified",nh_.getNamespace().c_str());
      return false;
    }
    robot_nh_=ros::NodeHandle(nh_.getNamespace()+"/mounted_robot_ik");

    ikloader_.reset(new pluginlib::ClassLoader<ik_solver::IkSolver>("ik_solver", "ik_solver::IkSolver"));


    robot_nh_.setParam("base_frame",robot_base_frame);

    std::string tmp;
    nh_.getParam("flange_frame",tmp);
    robot_nh_.setParam("flange_frame",tmp);
    nh_.getParam("tool_frame",tmp);
    robot_nh_.setParam("tool_frame",tmp);


    std::string plugin_name;
    if (!robot_nh_.getParam("type",plugin_name))
    {
      ROS_ERROR("%s/type is not defined",robot_nh_.getNamespace().c_str());
      return -1;
    }

    Eigen::Vector3d gravity;
    gravity << 0,0,-9.806;

    chain_ = rosdyn::createChain(model_,base_frame_,robot_base_frame,gravity);
    guide_seed_.resize(chain_->getActiveJointsNumber());
    guide_seed_.setZero();

    std::vector<std::string> guide_names=chain_->getActiveJointsName();
    std::vector<std::string> ordered_guide_names;
    std::vector<std::string> ordered_robot_names;
    for (std::string& s: joint_names_)
    {
      if (std::find(guide_names.begin(), guide_names.end(), s) != guide_names.end())
        ordered_guide_names.push_back(s);
      else
        ordered_robot_names.push_back(s);

    }
    if (ordered_guide_names.size()!=guide_names.size())
    {
      ROS_ERROR("not all the guides names are listed in %s/joint_names",nh_.getNamespace().c_str());
      ROS_ERROR("Guide names: ");
      for (std::string& s: guide_names)
        ROS_ERROR(" - %s",s.c_str());
      ROS_ERROR("joint_names: ");
      for (std::string& s: joint_names_)
        ROS_ERROR(" - %s",s.c_str());
      return false;
    }
    robot_nh_.setParam("joint_names",ordered_robot_names);

    joint_names_.clear();
    for (auto& s: ordered_guide_names)
      joint_names_.push_back(s);
    for (auto& s: ordered_robot_names)
      joint_names_.push_back(s);

    chain_->setInputJointsName(ordered_guide_names);

    ROS_DEBUG("%s creating ik plugin for mounted robot",robot_nh_.getNamespace().c_str());
    robot_ik_solver_ = ikloader_->createInstance(plugin_name);

    if (!robot_ik_solver_->config(robot_nh_))
    {
        ROS_ERROR("%s: error configuring mounted robot ik",nh_.getNamespace().c_str());
        return false;
    }




    dq_=0.5*(chain_->getQMax()-chain_->getQMin());
    mean_q_=0.5*(chain_->getQMax()+chain_->getQMin());

    ROS_DEBUG("Created %s IK Solver",nh_.getNamespace().c_str());

    return true;

  }


  std::vector<Eigen::VectorXd> RobotOnGuideIkSolver::getIk(const Eigen::Affine3d& T_base_flange,
                                                           const std::vector<Eigen::VectorXd> & seeds,
                                                           const int& desired_solutions,
                                                           const int& max_stall_iterations)
  {
    std::vector<Eigen::VectorXd > solutions;
    solutions.clear();
    std::vector<Eigen::VectorXd> seeds_robot;

    int idx =0;
    Eigen::VectorXd last_guide_seed(guide_seed_.size());
    int idx_seed=0;
    for (idx=0;idx<max_stall_iterations;idx++)
    {
      if (!nh_.ok())
        break;
      seeds_robot.clear();

      bool use_random=false;
      if (idx_seed<(int)seeds.size())
      {

        while(true)
        {
          const Eigen::VectorXd& full_seed=seeds.at(idx_seed);
          guide_seed_=full_seed.head(guide_seed_.size());
          seeds_robot.push_back(full_seed.tail(full_seed.size()-guide_seed_.size()));
          if ((guide_seed_-last_guide_seed).norm()>1e-6)
            break;
          else
          {
            if (idx_seed<(int)seeds.size()-1)
              idx_seed++; // discard repeated seeds
            else
            {
              use_random=true;
              break;
            }
          }
        }

        last_guide_seed=guide_seed_;
      }
      else
        use_random=true;

      if (use_random)
      {
        guide_seed_.setRandom();
        guide_seed_=mean_q_+dq_.cwiseProduct(guide_seed_);
      }

      Eigen::Affine3d T_base_robotbase=chain_->getTransformation(guide_seed_);
      Eigen::Affine3d T_robotbase_flange=T_base_robotbase.inverse()*T_base_flange;
      std::vector<Eigen::VectorXd> robot_sol=robot_ik_solver_->getIk(T_robotbase_flange,seeds_robot,desired_solutions,max_stall_iterations);

      for (const Eigen::VectorXd& q_robot: robot_sol)
      {
        Eigen::VectorXd q_tot(guide_seed_.size()+q_robot.size());
        q_tot<<guide_seed_,q_robot;
        solutions.push_back(q_tot);

      }
      idx_seed++;

      if ((int)solutions.size()>desired_solutions)
      {
        break;
      }

    }

    if (solutions.size()==0)
      ROS_INFO("seed size =%zu, solution size=%zu, iteration=%d",seeds.size(),solutions.size(),idx);

    return solutions;
  }


}   // end namespace ik_solver
