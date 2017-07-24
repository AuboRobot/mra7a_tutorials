#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <std_msgs/Float32MultiArray.h>


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
    attached_object.link_name = "r_wrist_roll_link";
    /* The header must contain a valid TF frame*/
    attached_object.object.header.frame_id = "base_link";
    /* The id of the object */
    attached_object.object.id = "box";

    /* A default pose */
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.5;
    primitive.dimensions[1] = 1.5;
    primitive.dimensions[2] = 0.17;

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

std_msgs::Float32MultiArray eff_pose;
void eff_pose_callback(const std_msgs::Float32MultiArrayConstPtr msg) {
    for(int i=0; i<6; i++){
        eff_pose.data.push_back(msg->data[i]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mra7a_eff_pose_plan");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(10);
    add_object(node_handle);

    ros::Subscriber eff_pose_sub = node_handle.subscribe("end_effector_pose", 1, eff_pose_callback);
    eff_pose.data.resize(0);

    /*moveit arm group*/
    moveit::planning_interface::MoveGroup group("arm");
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setNumPlanningAttempts(3);
    group.allowReplanning(true);
    group.setPlanningTime(10);//10s

    /*set end effector pose*/
    geometry_msgs::PoseStamped eff_target;
    eff_target.pose.position.x = 0.4;
    eff_target.pose.position.y = 0;
    eff_target.pose.position.z = 0.4;
    tf::Quaternion q = tf::createQuaternionFromRPY(0,1.57,0);
    eff_target.pose.orientation.x = q.getX();
    eff_target.pose.orientation.y = q.getY();
    eff_target.pose.orientation.z = q.getZ();
    eff_target.pose.orientation.w = q.getW();


    group.setPoseTarget(eff_target);


    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

    group.execute(my_plan);

    sleep(2);

        while(ros::ok()){
            ros::spinOnce();
            if(eff_pose.data.size()){
                eff_pose.data.resize(0);
                eff_target.pose.position.x = eff_pose.data[0];
                eff_target.pose.position.y = eff_pose.data[1];
                eff_target.pose.position.z = eff_pose.data[2];
                q = tf::createQuaternionFromRPY(eff_pose.data[3],eff_pose.data[4],eff_pose.data[5]);
                eff_target.pose.orientation.x = q.getX();
                eff_target.pose.orientation.y = q.getY();
                eff_target.pose.orientation.z = q.getZ();
                eff_target.pose.orientation.w = q.getW();

                group.setPoseTarget(eff_target);

                success = group.plan(my_plan);
                ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
                group.execute(my_plan);
                sleep(2);
            }
        }

    ros::spin();
    return 0;
}
