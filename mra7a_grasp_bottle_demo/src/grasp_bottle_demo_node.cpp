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

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <std_msgs/Int8.h>
#define GRIPPER_COMMAND "/mra/gripper_command"

geometry_msgs::Pose pose_ready_fetch;
geometry_msgs::Pose pose_ready_catch;
geometry_msgs::Pose pose_ready_pour;



tf::TransformListener* pListener = NULL;
tf::TransformBroadcaster* pBroadcaster = NULL;
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

void pose_transfer_gripper_to_link7(const geometry_msgs::Pose gripper_pose,geometry_msgs::Pose &link7_pose){

    tf::Transform gripper_to_base_transform;
    tf::Vector3 gripper_to_base_pos;
    tf::Quaternion gripper_to_base_ori;

    gripper_to_base_pos.setValue(gripper_pose.position.x,
                                 gripper_pose.position.y,
                                 gripper_pose.position.z);
    gripper_to_base_ori.setW(gripper_pose.orientation.w);
    gripper_to_base_ori.setX(gripper_pose.orientation.x);
    gripper_to_base_ori.setY(gripper_pose.orientation.y);
    gripper_to_base_ori.setZ(gripper_pose.orientation.z);

    gripper_to_base_transform.setOrigin(gripper_to_base_pos);
    gripper_to_base_transform.setRotation(gripper_to_base_ori);

    tf::Transform link7_to_gripper_transform;
    tf::Vector3 link7_to_gripper_pos;
    tf::Quaternion link7_to_gripper_ori;



    link7_to_gripper_pos.setValue(0.0,0.0,-0.125);
    link7_to_gripper_ori.setRPY(0,0,0);
    //    link7_to_gripper_ori.setW(1.0);
    //    link7_to_gripper_ori.setX(0.0);
    //    link7_to_gripper_ori.setY(0.0);
    //    link7_to_gripper_ori.setZ(0.0);

    link7_to_gripper_transform.setOrigin(link7_to_gripper_pos);
    link7_to_gripper_transform.setRotation(link7_to_gripper_ori);

    tf::Transform link7_to_base_transform;

    link7_to_base_transform = gripper_to_base_transform * link7_to_gripper_transform;

    pBroadcaster->sendTransform(tf::StampedTransform(link7_to_base_transform, ros::Time::now(), "base_link", "Link7"));

    link7_pose.position.x = link7_to_base_transform.getOrigin().x();
    link7_pose.position.y = link7_to_base_transform.getOrigin().y();
    link7_pose.position.z = link7_to_base_transform.getOrigin().z();

    link7_pose.orientation.w = link7_to_base_transform.getRotation().w();
    link7_pose.orientation.x = link7_to_base_transform.getRotation().x();
    link7_pose.orientation.y = link7_to_base_transform.getRotation().y();
    link7_pose.orientation.z = link7_to_base_transform.getRotation().z();
}
bool get_bottle_pose()
{
    ROS_INFO("get the pose of the bottle");

    /*定时1S取30次，得到平均值作为杯子早基坐标系下的XYZ*/
    tf::StampedTransform STtransform;
    double Avg_x,Avg_y,Avg_z;
    Avg_x = 0.0;
    Avg_y = 0.0;
    Avg_z = 0.0;

    try{
                ros::Rate readRate(30);
                for(size_t j =0;j<30;j++){

                    pListener->lookupTransform("base_link", "bottle", ros::Time(0), STtransform);
                    std::cout<<STtransform.getOrigin().getX()<<"  "<<STtransform.getOrigin().getY()<<"  "<<STtransform.getOrigin().getZ()<<"  "<<std::endl;
                    Avg_x += STtransform.getOrigin().getX();
                    Avg_y += STtransform.getOrigin().getY();
                    Avg_z += STtransform.getOrigin().getZ();
                    readRate.sleep();
                }

        Avg_x = Avg_x/30.0-0.07;
        Avg_y = Avg_y/30.0+0.165;
        Avg_z = Avg_z/30.0+0.28;
/*TEST
        Avg_x = 0.5;
        Avg_y = -0.4;
        Avg_z = 0.45;
*/
        std::cout<<"Result:"<<Avg_x<<"  "<<Avg_y<<"  "<<Avg_z<<"  "<<std::endl;


        tf::Transform transform_object_to_world;
        transform_object_to_world.setOrigin(tf::Vector3(Avg_x,Avg_y,Avg_z));

        tf::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        transform_object_to_world.setRotation(q);

        /*计算手抓抓水杯的起始位置和抓点位置*/
        geometry_msgs::Pose gripper_pose_start,gripper_pose_end;
        tf::Transform transform_gripper_to_object;
        transform_gripper_to_object.setOrigin(tf::Vector3(-0.08 , 0 , 0));
        q.setW(1.0);
        q.setRPY(0, M_PI_2, 0);
        transform_gripper_to_object.setRotation(q);
        //发布抓水杯的起始位置
        tf::Transform transform;
        transform = transform_object_to_world * transform_gripper_to_object;
        pBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "ee_1"));
        //抓水杯的起始位置
        gripper_pose_start.orientation.w = transform.getRotation().getW();
        gripper_pose_start.orientation.x = transform.getRotation().getX();
        gripper_pose_start.orientation.y = transform.getRotation().getY();
        gripper_pose_start.orientation.z = transform.getRotation().getZ();
        gripper_pose_start.position.x = transform.getOrigin().getX();
        gripper_pose_start.position.y = transform.getOrigin().getY();
        gripper_pose_start.position.z = transform.getOrigin().getZ();
        pose_transfer_gripper_to_link7(gripper_pose_start,pose_ready_fetch);
        //抓水杯的抓点位置
        transform_gripper_to_object.setOrigin(tf::Vector3(0 , 0 , 0));
        q.setW(1.0);
        q.setRPY(0, M_PI_2, 0);
        transform_gripper_to_object.setRotation(q);
        transform = transform_object_to_world * transform_gripper_to_object;
        gripper_pose_end.orientation.w = transform.getRotation().getW();
        gripper_pose_end.orientation.x = transform.getRotation().getX();
        gripper_pose_end.orientation.y = transform.getRotation().getY();
        gripper_pose_end.orientation.z = transform.getRotation().getZ();
        gripper_pose_end.position.x = transform.getOrigin().getX();
        gripper_pose_end.position.y = transform.getOrigin().getY();
        gripper_pose_end.position.z = transform.getOrigin().getZ();
        pose_transfer_gripper_to_link7(gripper_pose_end,pose_ready_catch);

        return true;

    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
}

bool get_glass_pose()
{
    ROS_INFO("get the pose of the glass");
    /*get glass pose*/
    tf::StampedTransform STtransform;
    double Avg_x,Avg_y,Avg_z;
    Avg_x = 0.0;
    Avg_y = 0.0;
    Avg_z = 0.0;
    try{
        //        ros::Rate readRate(30);
        //        for(size_t j =0;j<30;j++){

        //            pListener->lookupTransform("base_link", "glass", ros::Time(0), STtransform);
        //            Avg_x += STtransform.getOrigin().getX();
        //            Avg_y += STtransform.getOrigin().getY();
        //            Avg_z += STtransform.getOrigin().getZ();
        //            readRate.sleep();
        //        }
        Avg_x = Avg_x/30.0;
        Avg_y = Avg_y/30.0;
        Avg_z = Avg_z/30.0;
        Avg_x = 0.45;
        Avg_y = 0.1;
        Avg_z = 0.45;
        /*tf: glass to world*/
        tf::Transform transform_object_to_world;
        transform_object_to_world.setOrigin(tf::Vector3(Avg_x,Avg_y,Avg_z));
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        transform_object_to_world.setRotation(q);

        /*compute tf: glass to world*/
        tf::Transform transform_gripper_to_object;
        tf::Transform transform;
        transform_gripper_to_object.setOrigin(tf::Vector3( -0.05 ,-0.05, 0));
        q.setW(1.0);
        q.setRPY(0, M_PI_2, 0);
        transform_gripper_to_object.setRotation(q);
        transform = transform_object_to_world * transform_gripper_to_object;
        pBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "ee_2"));

        /*ready pour pose*/
        geometry_msgs::Pose gripper_pose_ready_pour;
        gripper_pose_ready_pour.orientation.w = transform.getRotation().getW();
        gripper_pose_ready_pour.orientation.x = transform.getRotation().getX();
        gripper_pose_ready_pour.orientation.y = transform.getRotation().getY();
        gripper_pose_ready_pour.orientation.z = transform.getRotation().getZ();
        gripper_pose_ready_pour.position.x = transform.getOrigin().getX();
        gripper_pose_ready_pour.position.y = transform.getOrigin().getY();
        gripper_pose_ready_pour.position.z = transform.getOrigin().getZ();


        pose_transfer_gripper_to_link7(gripper_pose_ready_pour,pose_ready_pour);

        return true;

    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
}


int main(int argc, char** argv){
    ros::init(argc,argv,"grasp_bottle_demo");
    ros::NodeHandle n_;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    sleep(10);
    add_object(n_);
    /*TF*/
    pListener = new tf::TransformListener;
    pBroadcaster = new tf::TransformBroadcaster;
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
    inital_pose_values[0] = 1.71;
    inital_pose_values[1] = 0.62;
    inital_pose_values[2] = -0.15;
    inital_pose_values[3] = -2.27;
    inital_pose_values[4] = -1.56;
    inital_pose_values[5] =  1.56;
    inital_pose_values[6] = -1.66;
    /*gripper publisher*/
    ros::Publisher gripper_pub = n_.advertise<std_msgs::Int8>(GRIPPER_COMMAND,1);
    std_msgs::Int8 gripper_command;
    gripper_command.data = 1;//open
    gripper_pub.publish(gripper_command);

    while(ros::ok()){
        ros::spinOnce();
        //input 123 to start
        ROS_INFO("get bottle and glass pose--input 1:");
        std::string mycmd;
        std::getline(std::cin,mycmd);
        if(!mycmd.compare("1")){

            bool getted_bottle = get_bottle_pose();
            bool getted_glass = get_glass_pose();

	    
            if(getted_bottle && getted_glass){
                //plan to the initial pose
                group.setJointValueTarget(inital_pose_values);
                bool success = group.plan(my_plan);
                if(success){
                    mycmd.clear();
                    while(mycmd.compare("2")){
                        ROS_INFO("go to init pose--input 2:");
                        std::getline(std::cin,mycmd);
                    }
                    group.execute(my_plan);
                    sleep(1);
                    //ready fetch
                    group.setPoseTarget(pose_ready_fetch);
                    success = group.plan(my_plan);
                    if(success){
                        mycmd.clear();
                        while(mycmd.compare("3")){
                            ROS_INFO("go to ready fetch pose--input 3:");
                            std::getline(std::cin,mycmd);
                        }
                        group.execute(my_plan);
sleep(1);
                        std::vector<double> joint_ready_fetch_value = group.getCurrentJointValues();
                        //ready catch
                        group.setPoseTarget(pose_ready_catch);
                        success = group.plan(my_plan);
                        if(success){
                            mycmd.clear();
                            while(mycmd.compare("4")){
                                ROS_INFO("go to ready catch pose--input 4:");
                                std::getline(std::cin,mycmd);
                            }
                            group.execute(my_plan);
                            std::vector<double> joint_ready_catch_value = group.getCurrentJointValues();
                            //gripper catch
                            mycmd.clear();
                            while(mycmd.compare("5")){
                                ROS_INFO("catch--input 5:");
                                std::getline(std::cin,mycmd);
                            }
                            gripper_command.data = 0;//close
                            gripper_pub.publish(gripper_command);
                            sleep(2);
                            //ready pour
//                            std::vector<double> rpy = group.getCurrentRPY("Link7");
//                            tf::Quaternion q = tf::createQuaternionFromRPY(rpy[0],rpy[1],rpy[2]);

//                            moveit_msgs::OrientationConstraint ocm;
//                            ocm.link_name = "Link7";
//                            ocm.header.frame_id = "base_link";
//                            ocm.orientation.x = q.getX();
//                            ocm.orientation.y = q.getY();
//                            ocm.orientation.z = q.getZ();
//                            ocm.orientation.w = q.getW();
//                            ocm.absolute_x_axis_tolerance = 0.1;
//                            ocm.absolute_y_axis_tolerance = 0.1;
//                            ocm.absolute_z_axis_tolerance = 0.1;
//                            ocm.weight = 1.0;
//                            moveit_msgs::Constraints test_constraints;
//                            test_constraints.orientation_constraints.push_back(ocm);
//                            group.setPathConstraints(test_constraints);

//                            pose_ready_pour.orientation.x = q.getX();
//                            pose_ready_pour.orientation.y = q.getY();
//                            pose_ready_pour.orientation.z = q.getZ();
//                            pose_ready_pour.orientation.w = q.getW();
                            group.setPoseTarget(pose_ready_pour);
                            group.setPlanningTime(10.0);
                            success = group.plan(my_plan);
                            if(success){
                                mycmd.clear();
                                while(mycmd.compare("6")){
                                    ROS_INFO("go to ready pour pose--input 6:");
                                    std::getline(std::cin,mycmd);
                                }
                                group.execute(my_plan);
                                group.clearPathConstraints();
                                //pour
                                std::vector<double> joint_pour_value = group.getCurrentJointValues();
                                joint_pour_value[6] -= M_PI/2;
                                group.setJointValueTarget(joint_pour_value);
                                success = group.plan(my_plan);
                                if(success){
                                    mycmd.clear();
                                    while(mycmd.compare("7")){
                                        ROS_INFO("pour--input 7:");
                                        std::getline(std::cin,mycmd);
                                    }
                                    group.execute(my_plan);
                                    //back to ready pour
                                    std::vector<double> joint_pour_value = group.getCurrentJointValues();
                                    joint_pour_value[6] += M_PI/2;
                                    group.setJointValueTarget(joint_pour_value);
                                    success = group.plan(my_plan);
                                    if(success){
                                        mycmd.clear();
                                        while(mycmd.compare("8")){
                                            ROS_INFO("back to ready pour--input 8:");
                                            std::getline(std::cin,mycmd);
                                        }
                                        group.execute(my_plan);
                                        //back to ready catch
                                        group.setJointValueTarget(joint_ready_catch_value);
                                        success = group.plan(my_plan);
                                        if(success){
                                            mycmd.clear();
                                            while(mycmd.compare("9")){
                                                ROS_INFO("back to ready catch--input 9:");
                                                std::getline(std::cin,mycmd);
                                            }
                                            group.execute(my_plan);
                                            //gripper open
                                            mycmd.clear();
                                            while(mycmd.compare("10")){
                                                ROS_INFO("gripper open--input 10:");
                                                std::getline(std::cin,mycmd);
                                            }
                                            gripper_command.data = 1;//close
                                            gripper_pub.publish(gripper_command);
                                            sleep(2);
                                            //back to ready fetch
                                            group.setJointValueTarget(joint_ready_fetch_value);
                                            success = group.plan(my_plan);
                                            if(success){
                                                mycmd.clear();
                                                while(mycmd.compare("11")){
                                                    ROS_INFO("back to ready fetch--input 11:");
                                                    std::getline(std::cin,mycmd);
                                                }
                                                group.execute(my_plan);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                else{
                    ROS_WARN("failed to plan the initial_pose!");
                }
            }

        }


    }

    return 0;
}
