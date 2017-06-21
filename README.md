# The mra7a_tutorials respository


**This repository shows some demos for controlling the mra7a by ROS.**<br>
1. The **moveit_visual_tools** package is the helper for displaying and debugging MoveIt! data in Rviz<br>
2. The **brain_control** stack is a demo for controling the motion of the mra7a by brain.<br>
3. The **mra7a_random_motion** package is a demo controlling the MRA7a random motion.<br>
        running command: roslaunch mra7a_random_motion mra7a_random_motion.launch  <br>
4. The **mra7a_eff_pose_plan** package is a simple program controlling the MRA7a's end effector to move to a specific point.<br>
        running command: <br>
           1.roslaunch mra_control mra7a_trajectory_rviz.launch   <br>
           2.roslaunch rosrun mra7a_eff_pose_plan mra7a_eff_pose_plan_node   <br>
