# robot_on_guide_ik_solver

IK for robot mounted on guides. It creates to chain:

- 1st: __guide__ base_frame -> robot_base_frame using rosdyn
- 2nd: __mounted robot__ robot_base_frame->tool_frame using another ik_solver plugins

When an IK is called:
- __guide__ generate a random FK
- transformation T_robotbase_tool=T_robotbase_base*T_base_tool
- IK of T_robotbase_tool for __mounted robot__ is computed
- the process is repeated untill desired solutions number is reached,



## Example

```
base_frame: world # base frame of the chain
robot_base_frame: ur5_base_link # robot base frame
flange_frame: ur5_tool0 # end frame of the chain
tool_frame: tip # destination frame of the IK (it should be rigid attached to flange_frame)
type: ik_solver/RobotOnGuideIkSolver
desired_solutions: 1000 # number of desired solution
joint_names:
- linear_motor_cursor_joint
- ur5_shoulder_pan_joint
- ur5_shoulder_lift_joint
- ur5_elbow_joint
- ur5_wrist_1_joint
- ur5_wrist_2_joint
- ur5_wrist_3_joint

mounted_robot_ik:
  type: ik_solver/Ur5IkSolver
  desired_solutions: 32 # number of desired solution
```
