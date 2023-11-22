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

#ifndef ROBOT_ON_GUIDE_IK_SOLVER__INTERNAL__MATH_H
#define ROBOT_ON_GUIDE_IK_SOLVER__INTERNAL__MATH_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>

#include <ik_solver/internal/types.h>

namespace ik_solver
{
/**
 * @brief
 *
 * @param joint_names
 * @param guide_names
 * @param robot_names
 * @param what
 * @return true
 * @return false
 */
bool order_joint_names(std::vector<std::string>& joint_names, std::vector<std::string>& guide_names,
                       std::vector<std::string>& robot_names, std::string& what);

/**
 * @brief
 *
 * @param p
 * @param lb
 * @param ub
 * @return double
 */
double percentage_distance_from(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Affine3d& ub);

/**
 * @brief
 *
 * @param p
 * @param lb
 * @param v
 * @return double
 */
double percentage_distance_from(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Vector3d& v);

/**
 * @brief
 *
 * @param p
 * @param lb
 * @param v
 * @return Eigen::Vector3d
 */
Eigen::Vector3d distance_from(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Vector3d& v);

/**
 * @brief
 *
 * @param p
 * @param lb
 * @param ub
 * @return Eigen::VectorXd
 */
Eigen::VectorXd force_inbound(const Eigen::VectorXd& p, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);

/**
 * @brief
 *
 * @param p
 * @param lb
 * @param ub
 * @param range_percentage
 * @return Eigen::VectorXd
 */
Eigen::VectorXd local_random(const Eigen::VectorXd& p, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                             double range_percentage);

/**
 * @brief
 *
 * @param lb
 * @param ub
 * @param i_step
 * @param n_steps
 * @return double
 */
double linear_interpolation(const double& lb, const double& ub, const size_t& i_step, const size_t& n_steps);

/**
 * @brief
 *
 * @param lb
 * @param ub
 * @param i_step
 * @param n_steps
 * @return double
 */
double half_sin_interpolation(const double& lb, const double& ub, const size_t& i_step, const size_t& n_steps);

/**
 * @brief
 *
 * @param s0
 * @param symmetric_delta
 * @param i_step
 * @param n_steps
 * @return double
 */
double symmetric_centered_sampler(const double& s0, const double& symmetric_delta, const size_t& i_step,
                                  const size_t& n_steps);

/**
 * @brief
 *
 * @param full_seeds
 * @param guide_seed_dof
 * @param guide_seeds
 * @param robot_seeds
 */
void filter_seeds(const Configurations& full_seeds, const size_t& guide_seed_dof, Configurations& guide_seeds,
                  Configurations& robot_seeds);

/**
 * @brief
 *
 * @param v
 * @param lb
 * @param ub
 * @param range_percentage
 * @return Eigen::VectorXd
 */
Eigen::VectorXd random(const Eigen::VectorXd& v, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                       double range_percentage);

/**
 * @brief
 *
 * @param target_intersection
 * @param target_reaching
 * @param p
 * @param lb
 * @param ub
 * @param AX0
 * @return true
 * @return false
 */
bool project_target_reaching(Eigen::Vector3d& target_intersection, const double& target_reaching,
                             const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Affine3d& ub,
                             const Eigen::Vector3d& AX0);

/**
 * @brief
 *
 * @param K
 * @param P
 * @param A
 * @param ray
 * @param r
 * @param cylinder_ax
 * @param closest_to_lb
 * @return true
 * @return false
 */
bool cylinder_ray_intersection(Eigen::Vector3d& K, const Eigen::Vector3d& P, const Eigen::Vector3d& A,
                               const Eigen::Vector3d& ray, const double& r, const Eigen::Vector3d& cylinder_ax,
                               bool closest_to_lb = true);

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
                               bool closest_to_lb = true);

/**
 * @brief
 *
 * @param P
 * @param A
 * @param ray
 * @param cylinder_ax
 * @return double
 */
double compute_polar_reaching(const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& ray,
                              const Eigen::Vector3d& cylinder_ax);

/**
 * @brief
 *
 * @param p
 * @param lb
 * @param ub
 * @param cylinder_ax
 * @return double
 */
double compute_polar_reaching(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Affine3d& ub,
                              const Eigen::Vector3d& cylinder_ax);

/**
 * @brief
 *
 * @param V
 * @param uv
 * @param R
 * @param ur
 * @return double
 */
double axes_distance(const Eigen::Vector3d& V, const Eigen::Vector3d& uv, const Eigen::Vector3d& R,
                     const Eigen::Vector3d& ur);

/**
 * @brief
 *
 * @param P
 * @param A
 * @param u
 * @return Eigen::Vector3d
 */
Eigen::Vector3d project(const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& u);

/**
 * @brief
 *
 * @param p
 * @param lb
 * @param ub
 * @return Eigen::Vector3d
 */
Eigen::Vector3d project(const Eigen::Affine3d& p, const Eigen::Affine3d& lb, const Eigen::Affine3d& ub);

}  // namespace ik_solver

#endif  // ROBOT_ON_GUIDE_IK_SOLVER__INTERNAL__MATH_H
