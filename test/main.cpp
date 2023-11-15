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

#include <iomanip> 

#include <Eigen/Geometry>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/AngleAxis.h"
#include "Eigen/src/Geometry/Transform.h"
#include "ros/node_handle.h"
#include "tf/LinearMath/Quaternion.h"
#include <ros/ros.h>

#include <tf_conversions/tf_eigen.h>

#include <ik_solver/internal/utils.h>

#include <robot_on_guide_ik_solver/robot_on_guide_ik_solver.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tapping_ik_solver");
  ros::NodeHandle nh("~");

  ik_solver::RobotOnGuideIkSolver ik_solver;
  if(ik_solver.config(nh, "/tapping_ik_solver"))
  {
    ik_solver::Configurations seeds;
    Eigen::VectorXd seed(7);
    seed << -0.045, -0.5514090237922974, 0.9093257585526082, -2.442220405508871, -0.12425511173628423,
        0.8912125632000252, -0.47083154881724176;
    seeds.push_back(seed);
    // seed << -0.044969118299902446, -0.5513972956964319, 0.9093144509666731, -2.4422318547449318, -0.12425333634341273,
    //     0.8912021379112449, -0.4708200932260197;
    // seeds.push_back(seed);
    // seed << -0.044876595075571575, -0.551362156681437, 0.9092805751220322, -2.442266154948986, -0.12424801650584642,
    //     0.8911709053999638, -0.4707857715496442;
    // seeds.push_back(seed);
    // seed << -0.044722795473903974, -0.5513037424023928, 0.9092242715361504, -2.442323163833534, -0.12423917123206599,
    //     0.8911189955245143, -0.47072871890376333;
    // seeds.push_back(seed);
    // seed << -0.04450832647166308, -0.5512222784090821, 0.9091457736661891, -2.442402645000833, -0.12422683215389374,
    //     0.8910466240273769, -0.47064915989438827;
    // seeds.push_back(seed);
    // seed << -0.04423403448001915, -0.551118079334335, 0.9090454068062923, -2.4425042690597736, -0.12421104345267586,
    //     0.8909540915080063, -0.47054740774012777;
    // seeds.push_back(seed);
    // seed << -0.04390100200415113, -0.5509915477604683, 0.9089235865540883, -2.4426276151790463, -0.12419186175524784,
    //     0.8908417819945627, -0.4704238630479819;
    // seeds.push_back(seed);
    // seed << -0.043510543371093204, -0.5508431727665946, 0.90878081685663, -2.442772173066272, -0.12416935599901642,
    //     0.8907101611242441, -0.47027901224735225;
    // seeds.push_back(seed);
    // seed << -0.04306419954268646, -0.5506735281603916, 0.9086176876486474, -2.4429373453600234, -0.12414360726533334,
    //     0.8905597739444309, -0.4701134256882089;
    // seeds.push_back(seed);
    // seed << -0.04256373203410654, -0.5504832703987722, 0.9084348720985861, -2.4431224504190676, -0.12411470858019813,
    //     0.8903912423492955, -0.46992775541065396;
    // seeds.push_back(seed);
    // seed << -0.042011115961967924, -0.5502731362027422, 0.9082331234803313, -2.4433267254907, -0.12408276468122353,
    //     0.8902052621688477, -0.4697227325943638;
    // seeds.push_back(seed);
    // seed << -0.0414085322494411, -0.5500439398726344, 0.9080132716906822, -2.4435493302378144, -0.1240478917497188,
    //     0.8900025999293956, -0.4694991646976309;
    // seeds.push_back(seed);
    // seed << -0.04075835901914509, -0.549796570310806, 0.9077762194346861, -2.443789350602334, -0.12401021710670379,
    //     0.8897840893063471, -0.46925793229691165;
    // seeds.push_back(seed);
    // seed << -0.04006316220778398, -0.549531987759834, 0.9075229381026744, -2.4440458029808143, -0.12396987887166853,
    //     0.8895506272918855, -0.46899998563894646;
    // seeds.push_back(seed);
    // seed << -0.03932568543956699, -0.5492512202652052, 0.9072544633644005, -2.444317638686509, -0.1239270255829285,
    //     0.8893031701014913, -0.46872634091863496;
    // seeds.push_back(seed);
    // seed << -0.038548839198377205, -0.5489553598724886, 0.9069718905069277, -2.4446037486708785, -0.12388181577850386,
    //     0.8890427288444693, -0.4684380762969213;
    // seeds.push_back(seed);
    // seed << -0.0377356893414213, -0.5486455585699911, 0.9066763695439283, -2.4449029684765247, -0.1238344175365803,
    //     0.8887703649845281, -0.4681363276739539;

    tf::Quaternion t(0.041, -0.042, -0.694, 0.717);
    Eigen::Quaterniond e;
    tf::quaternionTFToEigen(t, e);
    Eigen::Affine3d T06; T06 = ik_solver.getFK(seeds.front()).matrix();
    // T06.translation() << 2.038, 2.083, 0.331;
    // T06.linear().matrix() = e.matrix();

    T06.translation()(0)+=1.3;
    T06.translation()(1)+=0.5;
    T06.translation()(2)+=0.1;
    T06.linear() = T06.linear() * Eigen::AngleAxisd(0.01,Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.01,Eigen::Vector3d::UnitY());
    std::cout << "T06:\n" << T06.matrix() << std::endl;
    std::cout << "P   :" << T06.translation().transpose() << std::endl;
    std::cout << "LB  :" << ik_solver.le().translation().transpose() << std::endl;
    std::cout << "UB  :" << ik_solver.ue().translation().transpose() << std::endl;
    std::cout << "Seed0:\n" << ik_solver.getFK(seeds.front()).matrix() << std::endl;
    Eigen::Vector3d K;
    bool ok = ik_solver::cylinder_ray_intersection(K, T06, ik_solver.le(), ik_solver.ue(), 2.0, Eigen::Vector3d::UnitZ(), true);
    std::cout << "R     ok: " << int(ok) << ", " << K.transpose() << ", check    : " << 
      ik_solver::axes_distance(T06.translation(), Eigen::Vector3d::UnitZ(), K, Eigen::Vector3d::UnitZ()) - 2.0 
      << std::endl;
    std::cout << "reaching: " << ik_solver::compute_polar_reaching(T06, ik_solver.le(), ik_solver.ue(), Eigen::Vector3d::UnitZ()) << std::endl;
    auto sol = ik_solver.getIk(T06, {}, 32, 100, 200);
    for (const auto& q : sol)
    {
      auto robot_base = ik_solver.guide()->getTransformation(q.head(ik_solver.guide()->getJointsNumber()));
      auto dist = ik_solver::axes_distance(T06.translation(), Eigen::Vector3d::UnitZ(), robot_base.translation(), Eigen::Vector3d::UnitZ());
      std::cout << std::setw(10);
      std::cout << "IK/FK error: " << std::fixed << (T06.inverse() * ik_solver.getFK(q).matrix()).determinant() - 1.0 << " dst: " 
        << dist << " guide q:" << q.transpose() << std::endl;
    }
  }

  return 0;
}
