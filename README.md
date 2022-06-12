# RBE-500-Foundations
Holds my classwork/homework for the foundations course in Robotics at WPI

#### Course Project

The objective of this project is to develop a joint space position
control for a 3-DOF revolute-revolute-prismatic (RRP) robot
manipulator. The task is to implement a PID controller for joint space
tracking of the robot in Gazebo. The control approach is considered a
position-based control method, that is, the desired set-points provided
to the controller are (xd, yd, zd) coordinates of the end-effector.

The program first calculates the corresponding desired joint configurations
using the inverse kinematics of the robot, and then uses a PID controller
to move each joint to the desired configuration. The robot joints are
controlled using an independent joint control framework, that is, each
joint of the robot is controlled via a separate PID controller.

![](/home/radha/Downloads/joint_space_pid_control.gif)


#### Prerequisites

```
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

#### How to use

1. Place the ros packages(folder- Project/rbe500_project, Project/rrp_robot)
   into the src directory of your catkin workspace.


2. Build the packages
   ```
   cd rbe500_ros  # catkin workspace
   source devel/setup.bash
   catkin_make
   source devel/setup.bash
   ```

3. Open a terminal and launch the RRP robot manipulator in Gazebo
   ```
   roslaunch rrp_gazebo gazebo.launch
   ```

4. Open a new terminal and launch the effort controller node and the
   joint state publisher
   ```
   roslaunch rrp_control rrp_effort_control.launch
   ```

5. Test the inverse kinematics and position control scripts in the
   package by running the `rrp.launch` file in another terminal
   ```
   roslaunch rbe500_project rrp.launch
   ```