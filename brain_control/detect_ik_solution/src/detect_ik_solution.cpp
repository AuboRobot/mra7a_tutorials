#include <iostream>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <std_msgs/String.h>

std::deque<geometry_msgs::PoseStamped> joy_pose_deque;

/**
 * @brief joystick_pose_callback-->"/joy_pose"
 * @param msg
 */
void joystick_pose_callback(const geometry_msgs::PoseStamped msg) {
    joy_pose_deque.push_back(msg);
}


int main(int argc, char** argv) {

    //Initiate ROS
    ros::init(argc, argv, "joystick_control_server");
    ros::NodeHandle n_;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber joystick_pose_sub = n_.subscribe("/joy_pose",1,joystick_pose_callback);
    ros::Publisher ik_pub = n_.advertise<std_msgs::String>("/detect_ik_solution",1);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");


    /*get endlink pose*/
    //ROS_INFO("joint_model_group_right-->end effector name:%s",joint_model_group->.c_str());
    geometry_msgs::Pose pose_end_link;

    geometry_msgs::PoseStamped joy_pose_stamped;

    std_msgs::String ik_msg;

    joy_pose_deque.resize(1);


    while (ros::ok()) {
        ros::spinOnce();
        if (joy_pose_deque.size() != 0) {
            joy_pose_stamped = joy_pose_deque.front();//get the first of the queue;
            pose_end_link = joy_pose_stamped.pose;
            joy_pose_deque.pop_front();//delete the first of the queue;
  

            /*IK*/
            bool found_opt_ik  = kinematic_state->setFromIK(joint_model_group, pose_end_link);
            if(found_opt_ik){
                ik_msg.data = "has ik";
                ik_pub.publish(ik_msg);
            } else{
                ik_msg.data = "has no ik";
                ik_pub.publish(ik_msg);
            }
          
        }
    }

    return 0;
}

