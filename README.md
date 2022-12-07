# robot_on_guide_ik_solver

IK for robot mounted on guides. It creates to chain:

- 1st: base_frame -> robot_base_frame using rosdyn
- 2nd: robot_base_frame->tool_frame using another ik_solver plugins


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
  # type: ik_solver/RosdynIkSolver
  type: ik_solver/Ur5IkSolver
  desired_solutions: 32 # number of desired solution
```
