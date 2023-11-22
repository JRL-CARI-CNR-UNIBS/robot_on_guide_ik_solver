#include <robot_on_guide_ik_solver/internal/math.h>

namespace ik_solver 
{


bool order_joint_names(std::vector<std::string>& joint_names, std::vector<std::string>& guide_names,
                       std::vector<std::string>& robot_names, std::string& what)
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
    what = "Guide names: [";
    for (std::string& s : guide_names)
    {
      what += s +",";
    }
    what = "] joint_names: [";
    for (std::string& s : joint_names)
    {
      what += s +",";
    }
    what +="]";
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

  what = "Robot Chain has "+std::to_string(robot_names.size())+ " DOF, Guide Chain has "+std::to_string(ordered_guide_names.size()) + " DOF - " + jns;
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

double symmetric_centered_sampler(const double& s0, const double& symmetric_delta, const size_t& i_step, const size_t& n_steps)
{
  int _i_step = (i_step % 2 == 0) ? -i_step / 2 :  ( i_step + 1 / 2); 
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

bool project_target_reaching(Eigen::Vector3d& target_intersection, const double& target_reaching, const Eigen::Affine3d& p, const Eigen::Affine3d& lb,
                               const Eigen::Affine3d& ub, const Eigen::Vector3d& AX0)
{

  bool ok = ik_solver::cylinder_ray_intersection(target_intersection, p, lb, ub, target_reaching, AX0, true);
  if(!ok)
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
  return cylinder_ray_intersection(K, p.translation(), lb.translation(), (ub.translation()-lb.translation()).normalized(), r, cylinder_ax,
                                   closest_to_lb);
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
}  // namespace ik_solver 