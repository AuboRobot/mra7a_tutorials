#include <iostream>
#include <IKOptimalHandler.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <time.h>
#include <std_msgs/Float64.h>
#include <mra_basic/config.h>
#include <std_msgs/Float64MultiArray.h>
#include <smoothing_trajectory_filter_test/smoothing_trajectory_filter.h>

std::vector<std::vector<double> > interpolate1(const std::vector<double> &array_1,
                                               const std::vector<double> &array_2,
                                               int dof,
                                               double step)
{

    /*1.compute the distance between the two points*///两个路点间的距离＝各关节在这两点间的距离平方和再开根
    double distance_sum = 0;
    for(int i=0; i<dof; i++) {
        //ROS_INFO("\033[01;34m i:%d %s",i,Color_end);
        distance_sum += pow(array_2[i]-array_1[i],2);
    }
    ROS_INFO("\033[01;34m distance_sum0:%lf %s",distance_sum,Color_end);
    distance_sum = sqrt(distance_sum);
    ROS_INFO("\033[01;34m distance_sum:%lf %s",distance_sum,Color_end);
    /*2.compute the interpolate count*///(关节维数总长除以关节维数步长)-1-->插点个数
    unsigned int interpolate_count = distance_sum/sqrt(dof*pow(step,2));
    ROS_INFO("\033[01;34m interpolate_count:%d %s",interpolate_count,Color_end);

    /*interpolate*///先把起始点插入，然后循环插：起始点+步长×点数
    std::vector<std::vector<double> > trj_points;
    if(interpolate_count>10){
        return trj_points;//if interpolate_count>10 return empty, don't move
    }


    std::vector<double> joints;
    joints.resize(dof);

    trj_points.push_back(array_1);
    for (int i=0; i<interpolate_count; i++) {
        joints.clear();
        for (int j=0; j<dof; j++) {
            joints.push_back(array_1[j]+step*(i+1));
        }
        trj_points.push_back(joints);
    }
    trj_points.push_back(array_2);

    return trj_points;
}




/**
 * @brief getCurrentTime
 * @return usec
 */
long getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}


std::deque<geometry_msgs::PoseStamped> joy_pose_deque;

/**
 * @brief joystick_pose_callback-->"/joy_pose"
 * @param msg
 */
void joystick_pose_callback(const geometry_msgs::PoseStamped msg) {
    joy_pose_deque.push_back(msg);
}


int main(int argc, char** argv) {


    std::string group_name("arm");//modify to code to get

    //Initiate ROS
    ros::init(argc, argv, "joystick_control_server");
    ros::NodeHandle n_;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    /*SmoothingTrajectoryFilter*/
    std::vector<double> filter_coef_;
    industrial_trajectory_filters::SmoothingTrajectoryFilter smoothingTrajectoryFilter;
    filter_coef_ = smoothingTrajectoryFilter.get_coef_param(n_);
    for(int i=0; i<filter_coef_.size(); i++){
        std::cout<<filter_coef_[i]<<" "<<std::endl;
    }
    smoothingTrajectoryFilter.init(filter_coef_);

    /*subscribe /joy_pose topic*/
    ros::Subscriber joystick_pose_sub = n_.subscribe("/joy_pose",10,joystick_pose_callback);

    /*choose optimal ik solution*/
    IKOptimalHandler ikOptimalHandler;
    ikOptimalHandler.echo_hello();

    /*IK related*/
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_name);

    std::vector<double> current_joint_values;
    std::vector<double> single_optimal_joint_values;


    /*get endlink pose*/
    ROS_INFO("joint_model_group_right-->end effector name:%s",joint_model_group->getEndEffectorName().c_str());
    geometry_msgs::Pose pose_end_link;
    moveit::planning_interface::MoveGroup group(group_name);

    int DOF = group.getJoints().size();
    ROS_INFO("\033[01;34m DOF:%d %s",DOF,Color_end);

    /***
     * publisher for "moveJ"
     */
    ros::Publisher pub_moveJ = n_.advertise<std_msgs::Float32MultiArray>("moveJ",1000);
    ros::Publisher pub_joint1_position = n_.advertise<std_msgs::Float64>(JOINT1_POSITION_CONTROLLER,1000);
    ros::Publisher pub_joint2_position = n_.advertise<std_msgs::Float64>(JOINT2_POSITION_CONTROLLER,1000);
    ros::Publisher pub_joint3_position = n_.advertise<std_msgs::Float64>(JOINT3_POSITION_CONTROLLER,1000);
    ros::Publisher pub_joint4_position = n_.advertise<std_msgs::Float64>(JOINT4_POSITION_CONTROLLER,1000);
    ros::Publisher pub_joint5_position = n_.advertise<std_msgs::Float64>(JOINT5_POSITION_CONTROLLER,1000);
    ros::Publisher pub_joint6_position = n_.advertise<std_msgs::Float64>(JOINT6_POSITION_CONTROLLER,1000);
    ros::Publisher pub_joint7_position = n_.advertise<std_msgs::Float64>(JOINT7_POSITION_CONTROLLER,1000);

    std::vector<ros::Publisher> joint_position_publisher_v;
    joint_position_publisher_v.push_back(pub_joint1_position);
    joint_position_publisher_v.push_back(pub_joint2_position);
    joint_position_publisher_v.push_back(pub_joint3_position);
    joint_position_publisher_v.push_back(pub_joint4_position);
    joint_position_publisher_v.push_back(pub_joint5_position);
    joint_position_publisher_v.push_back(pub_joint6_position);
    joint_position_publisher_v.push_back(pub_joint7_position);


    std_msgs::Float32MultiArray joints_array;
    joints_array.data.resize(DOF);
    std::vector<double> current_state;

    std::vector<std::vector<double> > traj_interpolated;
    ros::Rate loop_rate(100);

    geometry_msgs::PoseStamped joy_pose_stamped;

    /*set init pose*/
//    for(int i=0; i<joints_array.data.size(); i++){
//        joints_array.data[i] = -0.015;
//    }
//    for(int j=0; j<50; j++){
//        for(int i=0; i<joints_array.data.size(); i++) {
//            std_msgs::Float64 joint_p;
//            joint_p.data = joints_array.data[i]*j;
//            joint_position_publisher_v[i].publish(joint_p);
//            double p = joint_p.data;
//            kinematic_state->setJointPositions(mra_basic_config::joint_names[i],&p);
//        }
//        loop_rate.sleep();
//    }


    while (ros::ok()) {
        ros::spinOnce();
        if (joy_pose_deque.size() != 0) {
            ROS_INFO("dequeue size:%d",int(joy_pose_deque.size()));
            joy_pose_stamped = joy_pose_deque.front();//get the first of the queue;
            pose_end_link = joy_pose_stamped.pose;
            joy_pose_deque.pop_front();//delete the first of the queue;

            /*get current endlink pose and print it*/
            kinematic_state->copyJointGroupPositions(group_name,current_state);
            for (int i=0; i<DOF; i++) {
                ROS_INFO("current state[%d]:%lf",i,current_state[i]);
            }

            group.setStartState(*kinematic_state);//the two step is important.
            group.setStartStateToCurrentState();


            /* 1.compute the new pose of endlink
             * 2.IK
             * 3.control
             */

            /*IK*/
            kinematic_state->copyJointGroupPositions(joint_model_group,current_joint_values);
//            bool found_opt_ik = ikOptimalHandler.getSingleOptimalIKSolution(kinematic_state,
//                                                                            joint_model_group,
//                                                                            pose_end_link,
//                                                                            current_joint_values,
//                                                                            single_optimal_joint_values
//                                                                            );
            bool found_opt_ik  = kinematic_state->setFromIK(joint_model_group, pose_end_link);
            long t = getCurrentTime();//compute IK time;
            if (found_opt_ik) {
                kinematic_state->copyJointGroupPositions(joint_model_group, single_optimal_joint_values);
                kinematic_state->setJointGroupPositions(joint_model_group,single_optimal_joint_values);

                ROS_INFO("time:%ld usec",getCurrentTime()-t);
                ROS_INFO("found ik!");

                ROS_INFO("-------------------interpolate---------------------");
                //                    traj_interpolated = ikOptimalHandler.interpolate(current_joint_values,single_optimal_joint_values,DOF)
                traj_interpolated = interpolate1(current_joint_values,single_optimal_joint_values,DOF,0.004);//0.004

                //Low-pass filter to smooth the trajectory, which will reduce the jump between adjacent two joints.
                smoothingTrajectoryFilter.applyFilter(traj_interpolated);

                /*cycle to publish the joint values*/
                std::vector<std::vector<double> >::iterator i;
                std::vector<double>::iterator j;
                std::vector<double> joints;
                for (i=traj_interpolated.begin(); i!=traj_interpolated.end(); i++) {
                    joints = *i;
                    int k = 0;
                    for(j=joints.begin(); j!=joints.end(); j++) {
                        joints_array.data[k] = *j;
                        std::cout<<"["<<joints_array.data[k] <<"]"<<std::endl;
                        k++;
                    }
                    std::cout<<std::endl;
                    //pub_moveJ.publish(joints_array);

                    for(int i=0; i<joints_array.data.size(); i++) {
                        std_msgs::Float64 joint_p;
                        joint_p.data = joints_array.data[i];
                        joint_position_publisher_v[i].publish(joint_p);
                    }
                    ros::spinOnce();
                    loop_rate.sleep();
                }

            } else {
                ROS_ERROR("not found ik!");
                joy_pose_deque.clear();
            }


        }
    }

    return 0;
}

