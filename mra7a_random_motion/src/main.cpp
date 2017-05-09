#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mra7a_random_motion");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

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
