# The brain_control stack
**This stack is a demo for controling the motion of the mra7a by brain.**
## Details
1. The **marker_control_python** package is for subscribing “/joy”　topic and control the effector of the mra7a. </br>
2. The **deteck_ik_solution** package is for insuring the mraker in the workspace that the mra7a can arrive.</br>
3. the **message_router** package is the middle node for transforming the command of brain to the ros topic "/joy".</br>

## Run Steps
0. Power on the mra7a.</br>
1. roslaunch mra_control mra7a_hw_trajectory_bingup.launch</br>
2. roslaunch mra7a_gazebo mra7a_bringup_rviz.launch(choose "Allow external comm" and planning algorithm).</br>
3. rosrun marker_control_python marker_control_start.py</br>
4. rosrun detect_ik_solution detect_ik_solution_node</br>
5. rosrun message_router message_router_node 192.168.1.255 6000</br>
6. TEST: cd messsage_router/test, g++ sender.c -o sender, run: ./sender  192.168.1.255 6000</br>
