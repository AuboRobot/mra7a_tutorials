#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>



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


    geometry_msgs::Pose pose2;
    pose2.orientation.w = 1.0;
    pose2.position.x = 0.6;
    pose2.position.y = 0;
    pose2.position.z = 1;
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 1.5;
    primitive2.dimensions[1] = 0.1;
    primitive2.dimensions[2] = 1.5;

    geometry_msgs::Pose pose3;
    pose3.orientation.w = 1.0;
    pose3.position.x = -0.6;
    pose3.position.y = 0;
    pose3.position.z = 1;
    shape_msgs::SolidPrimitive primitive3;
    primitive3.type = primitive3.BOX;
    primitive3.dimensions.resize(3);
    primitive3.dimensions[0] = 1.5;
    primitive3.dimensions[1] = 0.1;
    primitive3.dimensions[2] = 1.5;

    geometry_msgs::Pose pose4;
    pose4.orientation.w = 1.0;
    pose4.position.x = 0.6;
    pose4.position.y = 0;
    pose4.position.z = 0.75;
    shape_msgs::SolidPrimitive primitive4;
    primitive4.type = primitive4.BOX;
    primitive4.dimensions.resize(3);
    primitive4.dimensions[0] = 0.01;
    primitive4.dimensions[1] = 1.5;
    primitive4.dimensions[2] = 1.5;

    geometry_msgs::Pose pose5;
    pose5.orientation.w = 1.0;
    pose5.position.x = -0.6;
    pose5.position.y = 0;
    pose5.position.z = 0.75;
    shape_msgs::SolidPrimitive primitive5;
    primitive5.type = primitive5.BOX;
    primitive5.dimensions.resize(3);
    primitive5.dimensions[0] = 0.1;
    primitive5.dimensions[1] = 1.5;
    primitive5.dimensions[2] = 1.5;


    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);
//    attached_object.object.primitives.push_back(primitive2);
//    attached_object.object.primitive_poses.push_back(pose2);
//    attached_object.object.primitives.push_back(primitive3);
//    attached_object.object.primitive_poses.push_back(pose3);
//    attached_object.object.primitives.push_back(primitive4);
//    attached_object.object.primitive_poses.push_back(pose4);
//    attached_object.object.primitives.push_back(primitive5);
//    attached_object.object.primitive_poses.push_back(pose5);

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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mra7a_random_motion");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
sleep(10);
    add_object(node_handle);

    moveit::planning_interface::MoveGroup group("arm");
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setNumPlanningAttempts(3);
    group.allowReplanning(true);
    group.setPlanningTime(10);//10s

    while(ros::ok()){
        group.setRandomTarget();

        moveit::planning_interface::MoveGroup::Plan my_plan;
        bool success = group.plan(my_plan);
        ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

        group.execute(my_plan);

        sleep(2);
    }

    return 0;
}
