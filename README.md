# robot_on_guide_ik_solver

IK for robot mounted on guides. It creates to chain:

- 1st: __guide__ base_frame -> robot_base_frame using the library [`rosdyn`](https://github.com/CNR-STIIMA-IRAS/rosdyn)
- 2nd: __mounted robot__ robot_base_frame->tool_frame using another ik_solver plugins

When an IK is called:
**guide** generates a random FK
- transformation T_robotbase_tool=T_robotbase_base*T_base_tool
- IK of T_robotbase_tool for __mounted robot__ is computed
- the process is repeated untill desired solutions number is reached,



## Example

```yaml
###################################################
#
# The param needed by the plugin robot_on_guide_template
# The plugin inherit from ik_solver
# The plugin is agnostic about the robot mounted on the 
# linear axis
#
###################################################

# This param is inherited from the ik_solver base class.
# This param tells the plugin loader to load the 
# RobotOnGuideIkSolver
type: ik_solver/RobotOnGuideIkSolver

# Parameters used by the ik_solver node. The node creates a number of parallel threads for
# speed up the IK and FK computation.
# If the parallel mode is active, the speed is very high, but the previous IK solution cannot be exploited
# If the parallel mode is off, you can select if the previous IK solution is used as seed for the next IK solution
# or not
parallel_ik_mode: 2           # 0 default, 1 force parallelization, 2 disable parallelization
update_recursively_seeds: 1   # 0 default, 1 force update, 2 disable update !!! If paralle_ik_mode is 2, this is neglected

# Parameters inherited from the base class
# NOTE: here the parameter are for the whole chain, i.e., both axis and robot arm chain
group_name: manipulator
base_frame: world             # base frame of the chain
flange_frame: flange          # end frame of the chain
desired_solutions: 32         # number of desired solution
joint_names:                  # name of the whole chain 
- joint_7
- joint_1
- joint_2
- joint_3
- joint_4
- joint_5
- joint_6


# Parameters that are needed by this plugin
target_reaching: 1.8          # This is used to find a target solution point
                              # Specifically, it is the deisred distance of the tip from the required point, projected on the
                              # horizontal plane. Usually, it defines two points on the linear axis, computed as the interseciton
                              # of a cylinder centered in the tip and the linear axis
                              # Once these two points are defined, the solution is seeked around these two nominal points
max_exploration_range: 3.0    # This is the range of the guide that will be spanned around a local solution point 
                              # that is found using the target_reaching value
min_stall_iterations: 500     # The exploration range is split in 
max_stall_iterations: 3000

online_max_joint_elongation:
  joint_7: 0.010  # [m]


mounted_robot_ik:
  # This param is inherited from the ik_solver base class.
  # This param tells the plugin loader to load the 
  # Comau_NJ_Generic_IkSolver
  type: ik_solver/Comau_NJ_Generic_IkSolver
  
  # Parameters inherited from the base class. Note that if the needed param is not found here, 
  # we use the parameter defined for the whole chain
  base_frame: base_link # base frame of the mounted robot. 
                        # The chain of the guide is computed exploiting this information
  
  gamma_min_deg: 40     # This parameter is used from the Comau_NJ_Generic_IkSolver. It used to avoid collision of the parallelogram.
  epsilon_min_deg: 40   # This parameter is used from the Comau_NJ_Generic_IkSolver. It used to avoid collision of the parallelogram.
  desired_solutions: 32 # number of desired solution
  limits:
    joint_1: {upper: 1.5708, lower: -1.5708}
    joint_3: {upper: 0}
    joint_4: {upper: 1.77, lower: -0.2}
    joint_6: {upper: 1.57, lower: -1.57}

```
