#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <std_msgs/String.h>

#include <ros/ros.h>
#include <vector>
#include <std_msgs/Float32MultiArray.h>

#include <std_msgs/Int8.h>
#define GRIPPER_COMMAND "/mra/gripper_command"


void add_object(ros::NodeHandle &node_handle)
{
    ros::Duration sleep_time(5.0);
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    // Define the attached object message
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // We will use this message to add or
    // subtract the object from the world
    // and to attach the object to the robot
    moveit_msgs::AttachedCollisionObject attached_object;
    /* The header must contain a valid TF frame*/
    attached_object.object.header.frame_id = "base_link";
    /* The id of the object */
    attached_object.object.id = "box";

    /* A default pose */
    geometry_msgs::Pose pose;
    pose.position.x = 1.16;
    pose.position.z = 0.18;
    pose.orientation.w = 1.0;
    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.8;
    primitive.dimensions[1] = 1.8;
    primitive.dimensions[2] = 0.1;


    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);

    // Note that attaching an object to the robot requires
    // the corresponding operation to be specified as an ADD operation
    attached_object.object.operation = attached_object.object.ADD;

    // Add an object into the environment
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Add the object into the environment by adding it to
    // the set of collision objects in the "world" part of the
    // planning scene. Note that we are using only the "object"
    // field of the attached_object message here.
    ROS_INFO("Adding the object into the world at the location of the right wrist.");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    sleep_time.sleep();
}



int main(int argc, char** argv){
    ros::init(argc,argv,"circulation_control");
    ros::NodeHandle n_;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    sleep(10);
    add_object(n_);

    /*MoveGroup*/
    moveit::planning_interface::MoveGroup group("arm");
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setNumPlanningAttempts(5);
    group.allowReplanning(true);
    group.setPlanningTime(30);
    moveit::planning_interface::MoveGroup::Plan my_plan;

    /*inital_pose_values*/
    std::vector<double> inital_pose_values;
    inital_pose_values.resize(7);
    inital_pose_values[0] = 1.13;
    inital_pose_values[1] = -0.37;
    inital_pose_values[2] = 3.03;
    inital_pose_values[3] = 1.96;
    inital_pose_values[4] = -0.03;
    inital_pose_values[5] =  -0.79;
    inital_pose_values[6] = 0.04;

    /*inital_pose_values*/
    std::vector<double> joint_value2;
    joint_value2.resize(7);
    joint_value2[0] = 2.74;
    joint_value2[1] = -0.56;
    joint_value2[2] = -2.97;
    joint_value2[3] = 1.99;
    joint_value2[4] = 0.03;
    joint_value2[5] =  -1.04;
    joint_value2[6] = -0.06;

    /*gripper publisher*/
    ros::Publisher gripper_pub = n_.advertise<std_msgs::Int8>(GRIPPER_COMMAND,1);
    std_msgs::Int8 gripper_command;
    gripper_command.data = 1;//open
    gripper_pub.publish(gripper_command);
    bool success;

    while(ros::ok()){
        ros::spinOnce();

        group.setJointValueTarget(inital_pose_values);
        success = group.plan(my_plan);
        if(success){
            group.execute(my_plan);

            gripper_command.data = 0;//close
            gripper_pub.publish(gripper_command);
            sleep(2);
            gripper_command.data = 1;//open
            gripper_pub.publish(gripper_command);
            sleep(2);
        }

        group.setJointValueTarget(joint_value2);
        success = group.plan(my_plan);
        if(success){
            group.execute(my_plan);

            gripper_command.data = 0;//close
            gripper_pub.publish(gripper_command);
            sleep(2);
            gripper_command.data = 1;//open
            gripper_pub.publish(gripper_command);
            sleep(2);
        }
    }
    return 0;
}
